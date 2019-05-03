/*
 * PidState.cpp
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#include "PidState.h"
#include <EEPROM.h>
#include "UDPTacer.h"

PidState::PidState() : pid(&temperature, &Output, &DynamicSetpoint, kp, ki, kd,P_ON_E, DIRECT){
	pid.SetSampleTime(pidSampleTimeSecs*1000);
	pid.SetMode(MANUAL);
	servo.attach(D8);  // attaches the servo on pin 9 to the servo object
	//setServoPosition(0);
	currentMenu = new MainMenu();
	approacingStartMillis=0;

//	dTemperature=0;

	pid.myPTerm = &myPTerm;
	pid.myITerm = &myITerm;
	pid.myDTerm = &myDTerm;
	pid.myDInput= &myDInput;
	pid.myDTimeMillis= &myDTimeMillis;
	pid.myError = &myError;
	pid.myOutputSum = &myOutputSum;
	PrevSwitchOnOffMillis = 0;
}

MenuItem* PidState::decodeCurrentMenu(){
	MainMenu* pmm = (MainMenu*) topMenu;

	switch(state){
	case svUndefiend:
		case svMain :
			return pmm;
		case svRunAuto :
			return pmm->runMenu;
		case svRunAutoSetpoint :
			return pmm->runMenu->setpointMenu;
		case svRunAutoRamp :
			return pmm->runMenu->rampMenu;
		case svConfig:
			return pmm->configMenu;
		case svServo_Config:
			return pmm->configMenu->servoMenu;
		case svConfig_ServoDirection :
			return pmm->configMenu->servoMenu->dirMenu;
		case svConfig_ServoMin :
			return pmm->configMenu->servoMenu->minMenu;
		case svConfig_ServoMax:
			return pmm->configMenu->servoMenu->maxMenu;
		case svPidConfig:
			return pmm->configMenu->pidMenu;
		case svPidKpiConfig:
		case svPidKpdConfig:
			return pmm->configMenu->pidMenu->kpMenu;
		case svPidKiiConfig:
		case svPidKidConfig:
		case svPidKicConfig:
			return pmm->configMenu->pidMenu->kiMenu;
		case svPidKdiConfig:
		case svPidKddConfig:
			return pmm->configMenu->pidMenu->kdMenu;
		case svPidSampleTimeConfig:
			return pmm->configMenu->pidMenu->sampleTimeMenu;
	}
	return pmm;
}

long lastLog=0;



EncoderPushButtonState PidState::decodeEncoderPushBtnState (boolean encoderPress){
	if(encoderPress){
		lastPressMillis = millis();
		if(lastPressState == EncoderPushButtonNone){
			lastPressMillis = millis();
			lastPressState = EncoderPushButtonPressed;
			return EncoderPushButtonPressed;
		} else {
			//already pressed. hinibit this pressure until a release occours
			//pressed, but not yet long press
			lastPressState = EncoderPushButtonPressed;
			return EncoderPushButtonNone;
		}
	} else {
		lastPressMillis = -1;
		lastPressState = EncoderPushButtonNone;
		return EncoderPushButtonNone;
	}
}

boolean PidState::IsEncoderPressed(boolean encoderPress){
	return encoderPress && lastPressState == EncoderPushButtonPressed;
}

EncoderMovement PidState::decodeEncoderMoveDirection(int encoderPos){
	prevEncoderPos = currEncoderPos;
	currEncoderPos = encoderPos/4;
	if(currEncoderPos<prevEncoderPos){
		return EncMoveCCW;
	}else if(currEncoderPos>prevEncoderPos){
		return EncMoveCW;
	}
	return EncMoveNone;
}

void PidState::_writeServo(int degree){
	//Serial.print(F("WriteDegree:"));Serial.println(degree);
	servo.write(degree);
	delay(15);
	servoPosition = degree;
}

/**
 * here only the temperature flow is handled and the state transition should not depend
 * on other statuses. a burner switch off/on should always avoid flame bumps!!
 */
void PidState::writeServoPositionCW(int degree, bool minValueSwitchOff,bool log){
	float now = millis();
	if(log){
		Serial.print(F("Current temp state: "));Serial.println(TempState);
		Serial.print("TimeToLastSwitch:");Serial.print(now-PrevSwitchOnOffMillis);
		Serial.print(F(" Degree:"));Serial.print(degree);
		Serial.print(F(" ServoPosition:"));Serial.print(servoPosition);
		Serial.print(F(" ServoMinVal:"));Serial.println(servoMinValue);
	}
	if(degree<servoMinValue || (minValueSwitchOff && degree==servoMinValue)) degree=0; //switch off on minVal
	switch(TempState){
		case TempStateUndefined:
			PrevSwitchOnOffMillis=0;//this will force next on/off transition to happen immediately without waits
			if(degree>=servoMinValue){
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			if(degree<servoMinValue){
				Serial.println(F("Forward to TempStateSwitchingOff"));
				TempState = TempStateSwitchingOff;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			break;
		case TempStateSwitchingOn:
			if (degree>=servoMinValue){
				//now switchin on
				if(now-PrevSwitchOnOffMillis>10000){
					_writeServo(servoMaxValue);
					delay(1000);
					_writeServo(degree);
					PrevSwitchOnOffMillis = now;
					TempState = TempStateOn;
					return;
				}else{
					//skip and wait
					Serial.println(F("Switch on: wait 10 seconds..."));
					UdpTracer->println(F("Switch on: wait 5 seconds..."));
					return;
				}
			}
			Serial.println(F("Forward to TempStateSwitchingOff"));
			TempState = TempStateSwitchingOff;
			writeServoPosition(degree,minValueSwitchOff);
			break;
		case TempStateSwitchingOff:
			if (degree<servoMinValue){
				//now swtiching off
				if(now-PrevSwitchOnOffMillis>10000){
					_writeServo(servoMaxValue);
					delay(1000);
					_writeServo(0);
					PrevSwitchOnOffMillis = now;
					TempState = TempStateOff;
					return;
				}else{
					//skip and wait
					Serial.println(F("Switch off: wait 10 seconds..."));
					UdpTracer->println(F("Switch off: wait 10 seconds..."));
					return;
				}
			}
			//here temperature is greater that min meaning that before switch off is complete the temp has raised over min again
			//so abort and change state to on
			Serial.println(F("Forward to TempStateSwitchingOn"));
			TempState = TempStateSwitchingOn;
			writeServoPosition(degree,minValueSwitchOff);
			break;
		case TempStateOn:
			if (degree<servoMinValue){
				//now swtiching off
				Serial.println(F("Forward to TempStateSwitchingOff"));
				TempState = TempStateSwitchingOff;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(degree);
			break;
		case TempStateOff:
			if (degree>=servoMinValue ){
				//now swtiching off
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(degree);
			break;
	}
}

void PidState::writeServoPositionCCW(int degree, bool minValueSwitchOff){
	float now = millis();
//	Serial.print(F("Current temp state: "));Serial.println(TempState);
//	Serial.print("TimeToLastSwitch:");Serial.print(now-PrevSwitchOnOffMillis);
//	Serial.print(F(" Degree:"));Serial.print(degree);
//	Serial.print(F(" ServoPosition:"));Serial.print(servoPosition);
//	Serial.print(F(" ServoMinVal:"));Serial.println(servoMinValue);

	if(degree<servoMinValue || (minValueSwitchOff && degree==servoMinValue)) degree=0; //switch off on minVal
	switch(TempState){
		case TempStateUndefined:
			PrevSwitchOnOffMillis=0;//this will force next on/off transition to happen immediately without waits
			if(degree<=servoMaxValue){
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			if(degree>servoMinValue){
				Serial.println(F("Forward to TempStateSwitchingOff"));
				TempState = TempStateSwitchingOff;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			break;
		case TempStateSwitchingOn:
			if (degree<=servoMinValue){
				//now switchin on
				if(now-PrevSwitchOnOffMillis>10000){
					_writeServo(servoMaxValue);
					delay(1000);
					_writeServo(degree);
					PrevSwitchOnOffMillis = now;
					TempState = TempStateOn;
					return;
				}else{
					//skip and wait
					Serial.println(F("Switch on: wait 10 seconds..."));
					UdpTracer->println(F("Switch on: wait 5 seconds..."));
					return;
				}
			}
			Serial.println(F("Forward to TempStateSwitchingOff"));
			TempState = TempStateSwitchingOff;
			writeServoPosition(degree,minValueSwitchOff);
			break;
		case TempStateSwitchingOff:
			if (degree>servoMinValue){
				//now swtiching off
				if(now-PrevSwitchOnOffMillis>10000){
					_writeServo(servoMaxValue);
					delay(1000);
					_writeServo(0);
					PrevSwitchOnOffMillis = now;
					TempState = TempStateOff;
					return;
				}else{
					//skip and wait
					Serial.println(F("Switch off: wait 10 seconds..."));
					UdpTracer->println(F("Switch off: wait 10 seconds..."));
					return;
				}
			}
			//here temperature is greater that min meaning that before switch off is complete the temp has raised over min again
			//so abort and change state to on
			Serial.println(F("Forward to TempStateSwitchingOn"));
			TempState = TempStateSwitchingOn;
			writeServoPosition(degree,minValueSwitchOff);
			break;
		case TempStateOn:
			if (degree>servoMinValue){
				//now swtiching off
				Serial.println(F("Forward to TempStateSwitchingOff"));
				TempState = TempStateSwitchingOff;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(degree);
			break;
		case TempStateOff:
			if (degree<=servoMinValue ){
				//now swtiching off
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(degree);
			break;
	}
}

void PidState::writeServoPosition(int degree, bool minValueSwitchOff,bool log){
	if(servoDirection==ServoDirectionCW){
		writeServoPositionCW(degree, minValueSwitchOff,log);
	}else{
		writeServoPositionCCW(degree, minValueSwitchOff);
	}
}

void PidState::SetFsmState(FsmState value){
	fsmState = value;
	updatePidStatus();
}

void PidState::updatePidStatus(){
	Serial.print(F("State:"));Serial.println(fsmState);
	UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
	//looks not useful anymore. replace with fixed values when
	switch(fsmState){
	case psIdle:
			pid.SetTunings(kp,ki,kd);
			DynamicSetpoint=Setpoint;
		break;
	case psWaitDelay:
	case psRampimg:
			DynamicSetpoint=Setpoint; //this has to be recalculated time by time
			pid.SetTunings(kp,ki,kd);
		break;
	case psKeepTemp:
			DynamicSetpoint=Setpoint;
			pid.SetTunings(kp,ki,kd);
		break;
	}
}

void PidState::startRamp(){
	approacingStartMillis = millis();
	approacingStartTemp = temperature;
	DynamicSetpoint=temperature;
	if(DynamicSetpoint>Setpoint){
		DynamicSetpoint=Setpoint;
	}
	pDynamicSetpoint = temperature;
}
bool PidState::waitRampStart(){
	return temperature<approacingStartTemp+0.1;
}
void PidState::updateRamp(){
	float now = millis();
//	Serial.print(F("Update ramp: "));Serial.print(lastDynSetpointCalcMillis);

	if (lastDynSetpointCalcMillis==0)lastDynSetpointCalcMillis=now;
	float deltaSecs = (now-lastDynSetpointCalcMillis)/(float)1000.0;
//	Serial.print(F(" "));Serial.println(deltaSecs);
	if(deltaSecs>5){
		//		Serial.print(DynamicSetpoint);Serial.print(F(" "));Serial.print(pDynamicSetpoint);
		float deltaTemp = Ramp/(float)60.0*deltaSecs;

		if(fsmState==psWaitDelay|| abs(pDynamicSetpoint-Setpoint) <= deltaTemp){ //if the rate of change is going to drive past the setpoint, just make it equal the setpoint otherwise it'll oscillate
			DynamicSetpoint = Setpoint;
		} else{ //If more ramping is required, calculate the change required for the time period passed to keep the rate of change constant, and add it to the drive.
		  if(Setpoint>pDynamicSetpoint){  //positive direction
			  DynamicSetpoint = pDynamicSetpoint+deltaTemp;
		  }
		  else{   //negative direction
			  DynamicSetpoint = pDynamicSetpoint-deltaTemp;
		  }
		}
		//if(temperature>=DynamicSetpoint){
			//DynamicSetpoint=temperature;
//			pid.Reset();
		//}
		lastDynSetpointCalcMillis = now;
		pDynamicSetpoint = DynamicSetpoint;

		lastDynSetpointCalcMillis = now;
		Serial.print(F("New dyn setpoint:"));Serial.println(DynamicSetpoint,4);
		//UdpTracer->print(F("New dyn setpoint:"));UdpTracer->printFloat(DynamicSetpoint,4);UdpTracer->println();
	}
//	else{
//		Serial.println();
//	}
}

bool PidState::isAutoState(int state){
	return state==svRunAuto || state==svRunAutoSetpoint || state==svRunAutoRamp;
}

void PidState::sendStatus(){
	//Serial.print(DynamicSetpoint,4);Serial.print(F(" "));
	//Serial.print(Setpoint,4);Serial.print(F(" "));
	//Serial.print(temp,4);Serial.print(F(" "));
	//Serial.println(Output);
	float now = millis();
	UdpTracer->print(F("LOG:")      );UdpTracer->print(now,4);
	UdpTracer->print(F(";EXPRQID:") );UdpTracer->print((float)expectedReqId,0);
	UdpTracer->print(F(";STATE:")    );UdpTracer->print((float)autoModeOn,0);
	UdpTracer->print(F(";SETP:")    );UdpTracer->print(Setpoint,4);
	UdpTracer->print(F(";RAMP:")    );UdpTracer->print(Ramp,4);
	UdpTracer->print(F(";DSETP:")   );UdpTracer->print(DynamicSetpoint,4);
	UdpTracer->print(F(";TEMP:")    );UdpTracer->print(temperature,4);
	UdpTracer->print(F(";OUT:")     );UdpTracer->print(Output,4);
	UdpTracer->print(F(";OUTPERC:") );UdpTracer->print((float)getOutPerc(),0);
	UdpTracer->print(F(";SERVOPOS:"));UdpTracer->print((float)servoPosition,0);
	UdpTracer->print(F(";PGAIN:")   );UdpTracer->print(myPTerm,4);
	UdpTracer->print(F(";IGAIN:")   );UdpTracer->print(myITerm,4);
	UdpTracer->print(F(";DGAIN:")   );UdpTracer->print(myDTerm,4);
	UdpTracer->print(F(";OUTSUM:")  );UdpTracer->print(myOutputSum,4);
	UdpTracer->print(F(";PIDDTEMP:"));UdpTracer->println(myDInput,4);
}

int PidState::getOutPerc(){
	int o = 0;
	double outRange = servoMaxValue-servoMinValue;
	if(servoDirection==ServoDirectionCW){
		if(Output<servoMinValue) o = 0;
		else o = 100.0*(Output-servoMinValue)/outRange;
	}else{
		if(Output>servoMaxValue) o = 0;
		else o = 100.0*(servoMaxValue - Output)/outRange;
	}
	return o;
}

void PidState::setOutPerc(double val){
	Serial.print(F(">>>>>> OutputP: "));Serial.println(val);
	double outRange = servoMaxValue-servoMinValue;
	if(servoDirection==ServoDirectionCW){
		Output = servoMinValue + (val/100.0*outRange);
	}else{
		Output = servoMaxValue - (val/100.0*outRange);
	}
	Serial.print(F(">>>>>> Output: "));Serial.println(Output);
	writeServoPosition(Output,true,true);
}

void PidState::update(double temp,int encoderPos, boolean encoderPress){

	if(temp>-100){
		setTemperature(temp);
	}

	if(pid.GetMode()!=MANUAL && !isAutoState(state) &&
	   state!=svConfig_ServoDirection && state!=svConfig_ServoMin&&state!=svConfig_ServoMax)
	{
		pid.SetMode(MANUAL);
		pid.Initialize();

	}

	if(autoModeOn==0){
		pid.SetMode(MANUAL);
	}

	if(!isAutoState(state) &&
		state!=svConfig_ServoDirection && state!=svConfig_ServoMin&&state!=svConfig_ServoMax)
	{
//		Serial.println(F("xxx"));
		Output=0;
		writeServoPosition(Output,true,false);
	}



	//encoder position
	EncoderMovement encMovement = decodeEncoderMoveDirection(encoderPos);

	//encoder push button
	EncoderPushButtonState encoderPushButtonState = decodeEncoderPushBtnState(encoderPress);

//	Serial.print(F("State selection: "));Serial.println(stateSelection);
	//Serial.print(F("Encoder push: "));Serial.println(encoderPushButtonState);
	ESP.wdtFeed();
	setCurrentMenu(decodeCurrentMenu());

	if(encMovement!=EncMoveNone){
		Serial.print(F(">>>>>> Enc mov <<<<<<  "));
		if(encMovement==EncMoveCW){
			Serial.println(F(" CW "));
		}else{
			Serial.println(F(" CCW "));
		}
		Serial.print(F("Menu    :"));Serial.println(getCurrentMenu()->Caption);
		Serial.print(F("Menu len:"));Serial.println(getCurrentMenu()->subMenuItemsLen());
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		Serial.print(F("Selected :"));Serial.println(stateSelection);
		getCurrentMenu()->HandleEncoderMovement(encMovement);
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("New Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		Serial.print(F("New Selected :"));Serial.println(stateSelection);
	}
	if(encoderPushButtonState==EncoderPushButtonPressed) {
		Serial.print(F(">>>>>> Push <<<<<<  "));Serial.println(stateSelection);
		Serial.print(F("Menu     :"));Serial.println(getCurrentMenu()->Caption);
		Serial.print(F("Menu  len:"));Serial.println(getCurrentMenu()->subMenuItemsLen());
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		getCurrentMenu()->HandleEncoderPush(encoderPushButtonState);
	}

	float now = millis();
	ESP.wdtFeed();

	if(autoModeOn==0 || !isAutoState(state)){
		fsmState = psIdle;
	}

	switch(state){
		case svRunAuto :
		case svRunAutoSetpoint :
		case svRunAutoRamp : {

			if(autoModeOn==0){
				sendStatus();
				break;
			}

			if(temp<=-100){
				return;
			}

			if(autoModeOn==0){
				if(forcedOutput>0){
					pid.SetMode(MANUAL);
					setOutPerc(forcedOutput);
					return;
				}
			}

			if(pid.GetMode()!=AUTOMATIC){
				Serial.println(F("PID switched to AUTOMATIC"));
				pid.SetTunings(kp,ki,kd,P_ON_E);
				pid.SetControllerDirection(servoDirection==ServoDirectionCW?DIRECT:REVERSE);
				pid.SetOutputLimits(servoMinValue,servoMaxValue);
				pid.SetMode(AUTOMATIC);
			}

			if(Ramp<=0 && fsmState!=psKeepTemp){
				SetFsmState(psKeepTemp);
			}

			if(Ramp<=0){
				//no ramp is done, bit some logic is based on DynSetpoint, so set it to Setpoint
				DynamicSetpoint = Setpoint;
			}

//			Serial.print(F("Current state:"));Serial.println(fsmState);
//			UdpTracer->print(F("Current state:"));UdpTracer->println(fsmState);
			switch(fsmState){
			case psIdle:
				if(temp<=Setpoint-1){
					SetFsmState(psWaitDelay);
					startRamp();
				} else if(temp>Setpoint-1 /*&& temp<Setpoint*/){
					SetFsmState(psKeepTemp);
				}
//				else if(temp>=Setpoint){
//  				SetFsmState(psKeepTemp);
//					Output=0;
//					pid.Initialize();
//				}
				Serial.print(F("NEW State:"));Serial.println(fsmState);
				UdpTracer->print(F("NEW State:"));UdpTracer->println(fsmState);
				break;
			case psWaitDelay:
			case psRampimg:
				if(fsmState == psWaitDelay && waitRampStart()){
					SetFsmState(psRampimg);
					pid.Reset();
					startRamp();
				}
				if(temp<=Setpoint-1){
					updateRamp();
				} else if(temp>Setpoint-1 /*&& temp<Setpoint*/){
					SetFsmState(psKeepTemp);
					pid.Reset();
				}
				break;
			case psKeepTemp:
				if(Ramp>0 && temp<=Setpoint-1){
					SetFsmState(psIdle);
				}
				break;
			}

			bool computed = false;
			if((DynamicSetpoint - temperature)<3.5){
				//activate pid modulation
				computed = pid.Compute();
				writeServoPosition(Output,fsmState!=psRampimg);
			}else{
				//max fire applied
				setOutPerc(100);
				writeServoPosition(servoMaxValue,false);
			}
			if(computed || (now-lastUdpDataSent>1000)){
				sendStatus();
				lastUdpDataSent = now;
			}
			break;
		}
		break;
	}
}

void PidState::saveSetPointTotoEEprom(){
	EEPROM.begin(512);

	int addr = 0;
	addr+=1;
	addr+=sizeof(PidStateValue);
	addr+=sizeof(ServoDirection);
	addr+=sizeof(int);
	addr+=sizeof(int);
	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(Setpoint);
	EEPROM.put(addr, Setpoint);
	addr+=sizeof(double);

	EEPROM.commit();
//	EEPROM.end();

}

void PidState::loadFromEEProm(){
	Serial.print(F("EEPROMReadSettings: Expected EEPROM_VER:"));Serial.println(eepromVer);

	EEPROM.begin(512);
	byte temp=0;
	int addr = 0;
	temp = EEPROM.get(addr, temp);
	addr+=1;Serial.print(F("Size: "));Serial.println(addr);

	if(temp!=eepromVer && temp!=eepromVer-1){
		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
		return;
	}

	state=EEPROM.get(addr, state);
	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed state: "));Serial.println(state);
	if(state<0 || state>100){
		state=svMain;
		Serial.println(F(">>>>>>>> SOMETHING WENT WRONG.... ABORT READING!"));
		return;
	}

	servoDirection = EEPROM.get(addr, servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo dir: "));Serial.println(servoDirection);

	servoMinValue = EEPROM.get(addr, servoMinValue);
	addr+=sizeof(servoMinValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo min: "));Serial.println(servoMinValue);

	servoMaxValue = EEPROM.get(addr, servoMaxValue);
	addr+=sizeof(servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo max: "));Serial.println(servoMaxValue);

	Setpoint = EEPROM.get(addr, Setpoint);
	addr+=sizeof(Setpoint);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed setpoint: "));Serial.println(Setpoint);

	kp = EEPROM.get(addr, kp);
	addr+=sizeof(kp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kp: "));Serial.println(kp);

	ki = EEPROM.get(addr, ki);
	addr+=sizeof(ki);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed ki: "));Serial.println(ki);

	kd = EEPROM.get(addr, kd);
	addr+=sizeof(kd);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kd: "));Serial.println(kd);

	pidSampleTimeSecs = EEPROM.get(addr, pidSampleTimeSecs);
	addr+=sizeof(pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed Sample time secs: "));Serial.println(pidSampleTimeSecs);
	if(pidSampleTimeSecs==NAN)pidSampleTimeSecs=5;

	if(temp==4){
		int dummy = 0;
		dummy = EEPROM.get(addr, dummy);
		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Timer secs: "));Serial.println(dummy);
		if(dummy>60*24)dummy=0;
		Serial.print(F("Calculated Timer mins: "));Serial.println(dummy);

		dummy = EEPROM.get(addr, dummy);
		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Timer state: "));Serial.println(dummy);
		if(dummy<0||dummy>2)dummy=0;
		Serial.print(F("Calculated Timer state: "));Serial.println(dummy);

		dummy = EEPROM.get(addr, dummy);
		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Timer elapsed secs: "));Serial.println(dummy);
		if(dummy>60*60*24)dummy=0;
		if(dummy<0)dummy=0;
	}
	if(temp>=5){
		autoModeOn = EEPROM.get(addr, autoModeOn);
		addr+=sizeof(autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed autoModeOn: "));Serial.println(autoModeOn);

		forcedOutput = EEPROM.get(addr, forcedOutput);
		addr+=sizeof(forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed forcedOutput: "));Serial.println(forcedOutput);
	}
	EEPROM.commit();
	EEPROM.end();
}





void PidState::savetoEEprom(){
	ESP.wdtFeed();
	EEPROM.begin(512);

	int addr = 0;
	EEPROM.put(addr, eepromVer);
	addr+=1;Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. pid state: "));Serial.println(state);
	EEPROM.put(addr, state);
	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(servoDirection);
	EEPROM.put(addr, servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(servoMinValue);
	EEPROM.put(addr, servoMinValue);
	addr+=sizeof(int);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(servoMaxValue);
	EEPROM.put(addr, servoMaxValue);
	addr+=sizeof(servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(Setpoint);
	EEPROM.put(addr, Setpoint);
	addr+=sizeof(Setpoint);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(kp);
	EEPROM.put(addr, kp);
	addr+=sizeof(kp);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(ki);
	EEPROM.put(addr, ki);
	addr+=sizeof(ki);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(kd);
	EEPROM.put(addr, kd);
	addr+=sizeof(kd);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(pidSampleTimeSecs);
	EEPROM.put(addr, pidSampleTimeSecs);
	addr+=sizeof(pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. autoModeOn: "));Serial.println(autoModeOn);
	EEPROM.put(addr, autoModeOn);
	addr+=sizeof(autoModeOn);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. forcedOutput: "));Serial.println(forcedOutput);
	EEPROM.put(addr, forcedOutput);
	addr+=sizeof(forcedOutput);Serial.print(F("Size: "));Serial.println(addr);

	EEPROM.commit();
	EEPROM.end();

	ESP.wdtFeed();

}

void PidState::saveServoDirToEEprom(){
	EEPROM.begin(512);

	int addr = 0;
	addr+=1;
	addr+=sizeof(PidStateValue);
	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(servoDirection);
	EEPROM.put(addr, servoDirection);
	addr+=sizeof(ServoDirection);
	addr+=sizeof(int);
	addr+=sizeof(int);
	addr+=sizeof(double);

	EEPROM.commit();
	EEPROM.end();
}
