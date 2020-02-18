/*
 * PidState.cpp
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#include "Controller.h"

#include <EEPROM.h>
#include <gdb.h>
#include "TCPComm.h"

Controller::Controller() : pid(&temperature, &Output, &_dynamicSetpoint, kp, ki, kd,P_ON_E, DIRECT){
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

MenuItem* Controller::decodeCurrentMenu(){
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
		case svConfig_Probe:
			return pmm->configMenu->probeMenu;
	}
	return pmm;
}

void Controller::setCurrentMenu(MenuItem *m){
	if(currentMenu == m) return;
	currentMenu = m;
	stateSelection=0;
	currMenuStart = 0;
}

long lastLog=0;



EncoderPushButtonState Controller::decodeEncoderPushBtnState (boolean encoderPress){
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

boolean Controller::IsEncoderPressed(boolean encoderPress){
	return encoderPress && lastPressState == EncoderPushButtonPressed;
}

EncoderMovement Controller::decodeEncoderMoveDirection(int encoderPos){
	prevEncoderPos = currEncoderPos;
	currEncoderPos = encoderPos/4;
	if(currEncoderPos>prevEncoderPos){
		return EncMoveCCW;
	}else if(currEncoderPos<prevEncoderPos){
		return EncMoveCW;
	}
	return EncMoveNone;
}



void RAMFUNC Controller::_writeServo(int value){
	int degree = value;
	if(servoDirection==ServoDirectionCCW){
		degree = servoMaxValue-degree;
	}

	if(servoPosition!=degree){
		servo.write(degree);
		delay(15);
		servoPosition = degree;
	}
}

/**
 * here only the temperature flow is handled and the state transition should not depend
 * on other statuses. a burner switch off/on should always avoid flame bumps!!
 */
void Controller::writeServoPosition(int degree, bool minValueSwitchOff,bool log){
	float now = millis();
	if(log){
		Serial.print(F("Current temp state: "));Serial.println(TempState);
		Serial.print(F("TimeToLastSwitch:"));Serial.print(now-PrevSwitchOnOffMillis);
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
					TcpComm->println(F("Switch on: wait 5 seconds..."));
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
					TcpComm->println(F("Switch off: wait 10 seconds..."));
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
				//now swtiching on
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(0);
			break;
	}
}

void Controller::SetFsmState(FsmState value){
	fsmState = value;
	updatePidStatus();
}

void Controller::updatePidStatus(){
	Serial.print(F("State:"));Serial.println(fsmState);
	TcpComm->print(F("State:"));TcpComm->println(fsmState);
	//looks not useful anymore. replace with fixed values when
	switch(fsmState){
	case psIdle:
			pid.SetTunings(kp,ki,kd);
			_dynamicSetpoint=_setpoint;
		break;
	case psWaitDelay:
	case psRampimg:
			_dynamicSetpoint=_setpoint; //this has to be recalculated time by time
			pid.SetTunings(kp,ki,kd);
		break;
	case psKeepTemp:
			_dynamicSetpoint=_setpoint;
			pid.SetTunings(kp,ki,kd);
		break;
	}
}

void Controller::startRamp(){
	approacingStartMillis = millis();
	approacingStartTemp = temperature;
	_dynamicSetpoint=temperature; //gives an impulse so that it must start to move up
	if(_dynamicSetpoint>_setpoint){
		_dynamicSetpoint=_setpoint;
	}
	lastDynSetpointCalcMillis=0;
}
bool Controller::waitRampStart(){
	return temperature<approacingStartTemp+0.1;
}
void Controller::updateRamp(){
	float now = millis();
//	Serial.print(F("Update ramp: "));Serial.print(lastDynSetpointCalcMillis);

	if (lastDynSetpointCalcMillis==0)lastDynSetpointCalcMillis=now;
	float deltaSecs   = (now-lastDynSetpointCalcMillis)/(float)1000.0;
	float elapsedSecs = (now-approacingStartMillis)/(float)1000.0;
//	Serial.print(F(" "));Serial.println(deltaSecs);
	if(deltaSecs>5){
		//		Serial.print(DynamicSetpoint);Serial.print(F(" "));Serial.print(pDynamicSetpoint);
		float elapsedDeltaTemp = _ramp/(float)60.0*elapsedSecs; //delta temp after elapsedSecs from start

		_dynamicSetpoint = approacingStartTemp + elapsedDeltaTemp; //calc new setpoint according to ramp
		if(fsmState==psWaitDelay || _dynamicSetpoint>_setpoint){ //target is Setpoit: do not exeed it!
			_dynamicSetpoint = _setpoint;
		}

		lastDynSetpointCalcMillis = now;
		Serial.print(F("New dyn setpoint:"));Serial.println(_dynamicSetpoint,4);
		//UdpTracer->print(F("New dyn setpoint:"));UdpTracer->printFloat(DynamicSetpoint,4);UdpTracer->println();
	}
}

bool Controller::isAutoState(int state){
	return state==svRunAuto || state==svRunAutoSetpoint || state==svRunAutoRamp;
}

void Controller::sendStatus(){
//	Serial.print(DynamicSetpoint,4);Serial.print(F(" "));
	//Serial.print(Setpoint,4);Serial.print(F(" "));
	Serial.println(temperature,4);Serial.print(F(" "));
	//Serial.println(Output);
	float now = millis();
	TcpComm->print(F("LOG:")      );TcpComm->print(now,4);
	TcpComm->print(F(";EXPRQID:") );TcpComm->print((float)expectedReqId,0);
	TcpComm->print(F(";STATE:")   );TcpComm->print((float)autoModeOn,0);
	TcpComm->print(F(";FSM_STATE:")   );TcpComm->print(fsmState);
	TcpComm->print(F(";SETP:")    );TcpComm->print(_setpoint,4);
	TcpComm->print(F(";RAMP:")    );TcpComm->print(_ramp,4);
	TcpComm->print(F(";DSETP:")   );TcpComm->print(_dynamicSetpoint,4);
	TcpComm->print(F(";TEMP:")    );TcpComm->print(temperature,4);
	TcpComm->print(F(";OUT:")     );TcpComm->print(Output,4);
	TcpComm->print(F(";OUTPERC:") );TcpComm->print((float)getOutPerc(),0);
	TcpComm->print(F(";SERVOPOS:"));TcpComm->print((float)servoPosition,0);
	TcpComm->print(F(";PGAIN:")   );TcpComm->print(myPTerm,4);
	TcpComm->print(F(";IGAIN:")   );TcpComm->print(myITerm,4);
	TcpComm->print(F(";DGAIN:")   );TcpComm->print(myDTerm,4);
	TcpComm->print(F(";OUTSUM:")  );TcpComm->print(myOutputSum,4);
	TcpComm->print(F(";PIDDTEMP:"));TcpComm->println(myDInput,4);

	lastUdpDataSent = now;
}

int Controller::getOutPerc(){
	int o = 0;
	double outRange = servoMaxValue-servoMinValue;
	if(Output<servoMinValue) o = 0;
	else o = 100.0*(Output-servoMinValue)/outRange;
	return o;
}

void Controller::setOutPerc(double val){
	Serial.print(F(">>>>>> OutputP: "));Serial.println(val);
	double outRange = servoMaxValue-servoMinValue;
	Output = servoMinValue + (val/100.0*outRange);
	Serial.print(F(">>>>>> Output: "));Serial.println(Output);
	writeServoPosition(Output,true,true);
}

void Controller::update(int encoderPos, boolean encoderPress){
    double tempp = probe.readTemperature();
	setTemperature(tempp);

	//force to manual when:
	//- not in auto
	//- not configuring
	if(pid.GetMode()!=MANUAL && !isAutoState(state) &&
	   state!=svConfig_ServoDirection && state!=svConfig_ServoMin&&state!=svConfig_ServoMax)
	{
		pid.SetMode(MANUAL);
		pid.Initialize();

	}

	//in each case, also if in auto, but automode is disabled, then force pid in manual
	if(autoModeOn==0){
		pid.SetMode(MANUAL);
	}


	//this is not clear!
	//when in Auto, but automode==0, then it shuts down.... ???
	if(!isAutoState(state) &&
		state!=svConfig_ServoDirection && state!=svConfig_ServoMin&&state!=svConfig_ServoMax)
	{
		Output=0;
		writeServoPosition(Output,true,false);
	}



	//encoder position
	EncoderMovement encMovement = decodeEncoderMoveDirection(encoderPos);

	//encoder push button
	EncoderPushButtonState encoderPushButtonState = decodeEncoderPushBtnState(encoderPress);

    //Serial.print(F("State selection: "));Serial.println(stateSelection);
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

	//fsm idle in case not automatic
	if(autoModeOn==0 || !isAutoState(state)){
		TcpComm->print(F("2 :"));TcpComm->println(fsmState);
		fsmState = psIdle;
	}

	switch(state){
		case svMain:
			writeServoPosition(0,true);
		break;
		case svRunAuto :
		case svRunAutoSetpoint :
		case svRunAutoRamp : {

			//if autoModeOn is disabled, then force the manual output and return
			if(autoModeOn==0){
				if(forcedOutput>0){
					pid.SetMode(MANUAL);
					setOutPerc(forcedOutput);
				}
				break;
			}
			
			//if here, pid should be in auto. if not, then let's force it!
			if(pid.GetMode()!=AUTOMATIC){
				Serial.println(F("PID switched to AUTOMATIC"));
				pid.SetTunings(kp,ki,kd,P_ON_E);
				pid.SetControllerDirection(DIRECT);
				pid.SetOutputLimits(servoMinValue,servoMaxValue);
				pid.SetMode(AUTOMATIC);
			}

			if(_ramp<=0 && fsmState!=psKeepTemp){
				SetFsmState(psKeepTemp);
			}

			if(_ramp<=0){
				//no ramp is done, bit some logic is based on DynSetpoint, so set it to Setpoint
				_dynamicSetpoint = _setpoint;
			}

//			Serial.print(F("Current state:"));Serial.println(fsmState);
//			UdpTracer->print(F("Current state:"));UdpTracer->println(fsmState);
			switch(fsmState){
			case psIdle:
				if(!probe.isReady()) break; //temperature reading not yet ready, then a false reading is executed
				if(temperature<=_setpoint-1){
					SetFsmState(psWaitDelay);
					startRamp();
				} else if(temperature>_setpoint-1 /*&& temp<Setpoint*/){
					SetFsmState(psKeepTemp);
				}
				Serial.print(F("NEW State:"));Serial.println(fsmState);
				TcpComm->print(F("NEW State:"));TcpComm->println(fsmState);
				break;
			case psWaitDelay:
			case psRampimg:
				if(fsmState == psWaitDelay && waitRampStart()){
					SetFsmState(psRampimg);
					pid.Reset();
					startRamp();
				}
				if(temperature<=_setpoint-1){

					updateRamp();
				} else if(temperature>_setpoint){
					SetFsmState(psKeepTemp);
					pid.Reset();//avoid delay in switching off
				}
				break;
			case psKeepTemp:
				if(_ramp>0 && temperature<=_setpoint-1){
					TcpComm->print(F("1 :"));TcpComm->print(temperature,2);TcpComm->println(fsmState);
					SetFsmState(psIdle);//it's a way to restart
				}
				break;
			}
			bool computed = false;
			if((_dynamicSetpoint - temperature)<3.5){
				//activate pid modulation
				computed = pid.Compute();
				writeServoPosition(Output,true);
			}else{
				//max fire applied
				setOutPerc(100);
				writeServoPosition(servoMaxValue,true);
			}
			if(computed || (now-lastUdpDataSent>1000)){
				sendStatus();
			}
			break;
		}
		break;
	}
	if(now-lastUdpDataSent>1000){
		sendStatus();
	}
}

void Controller::saveSetPointTotoEEprom(){
	EEPROM.begin(512);

	int addr = 0;
	addr+=1;
	addr+=sizeof(PidStateValue);
	addr+=sizeof(ServoDirection);
	addr+=sizeof(int);
	addr+=sizeof(int);
	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(_setpoint);
	EEPROM.put(addr, _setpoint);
	addr+=sizeof(double);

	EEPROM.commit();
//	EEPROM.end();

}

void Controller::loadFromEEProm(){
	Serial.print(F("EEPROMReadSettings: Expected EEPROM_VER:"));Serial.println(eepromVer);

	EEPROM.begin(512);
	byte temp=0;
	int addr = 0;
	temp = EEPROM.get(addr, temp);
	addr+=1;Serial.print(F("Size: "));Serial.println(addr);

//	if(temp!=eepromVer && temp!=eepromVer-1){
//		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
//		return;
//	}

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

	_setpoint = EEPROM.get(addr, _setpoint);
	addr+=sizeof(_setpoint);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed setpoint: "));Serial.println(_setpoint);

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
	if(temp>=6){
		temperatureCorrection = EEPROM.get(addr, temperatureCorrection);
		addr+=sizeof(temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed temperatureCorrection: "));Serial.println(temperatureCorrection*0.1);
	}
	EEPROM.commit();
	EEPROM.end();
}





void Controller::savetoEEprom(){
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

	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(_setpoint);
	EEPROM.put(addr, _setpoint);
	addr+=sizeof(_setpoint);Serial.print(F("Size: "));Serial.println(addr);

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

	Serial.print(F("EEPROMWriteSettings. temperatureCorrection: "));Serial.println(temperatureCorrection*0.1);
	EEPROM.put(addr, temperatureCorrection);
	addr+=sizeof(temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);

	EEPROM.commit();
	EEPROM.end();

	ESP.wdtFeed();

}

void Controller::saveServoDirToEEprom(){
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
