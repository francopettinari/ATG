/*
 * PidState.cpp
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#include "PidState.h"
#include <EEPROM.h>
#include "UDPTacer.h"

PidState::PidState() : pid(&temperature, &Output, &DynamicSetpoint, kp, ki, kd,P_ON_E, DIRECT),aTune(&temperature, &Output){
	pid.SetSampleTime(pidSampleTimeSecs*1000);
	pid.SetMode(MANUAL);
	servo.attach(D8);  // attaches the servo on pin 9 to the servo object
	//setServoPosition(0);
	currentMenu = new MainMenu();
	approacingStartMillis=0;

	aTune.SetNoiseBand(0.2);
	aTune.SetLookbackSec(20);
//	dTemperature=0;

	pid.myPTerm = &myPTerm;
	pid.myITerm = &myITerm;
	pid.myDTerm = &myDTerm;
	pid.myDInput= &myDInput;
	pid.myDTimeMillis= &myDTimeMillis;
	pid.myError = &myError;
	pid.myOutputSum = &myOutputSum;
}

MenuItem* PidState::decodeCurrentMenu(){
	MainMenu* pmm = (MainMenu*) topMenu;
	switch(state){
	case svUndefiend:
		case svMain :
			return pmm;
		case svRun:
			return pmm->runMenu;
		case svRunAuto :
			return pmm->runMenu->runAutoMenu;
		case svRunAutoTimer :
			return pmm->runMenu->runAutoMenu->timerMenu;
		case svRunAutoSetpoint :
			return pmm->runMenu->runAutoMenu->setpointMenu;
		case svRunAutoTuneResult :
			return pmm->runMenu->runAutoTuneMenu->autoTuneResultMenu;
		case svRunAutoTune :
			return pmm->runMenu->runAutoTuneMenu;
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

void PidState::changeAutoTune(int value)
{
 if(value>0) {
    //Set the output to the desired starting frequency.
	//Output=(servoMax-servoMin)/2+servoMin;
	//Output=110;
    aTune.SetNoiseBand(0.2);
    aTune.SetOutputStep(15);
    aTune.SetLookbackSec(20);
    aTune.SetControlType(1);
    pid.SetMode(MANUAL);
    autoTune = true;
  } else { //cancel autotune
    aTune.Cancel();
    autoTune = false;
//    pid.SetMode(MANUAL);
  }
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

void PidState::writeServoPosition(int degree){
	servo.write(degree);
	delay(15);
	if(servoDirection==ServoDirectionCW){
		SetServoOff(degree<=servoMinValue);
	}else{
		SetServoOff(degree>=servoMaxValue);
	}
	servoPosition = degree;
}

void PidState::setServoPosition(int degree){
	float now = millis();
	if(servoDirection==ServoDirectionCW){
		if(degree<=servoMinValue && IsServoOff()) return; //burner is already off

		if (degree<=servoMinValue && prevDegree>servoMinValue){
			//now swtiching off
			if(now-PrevOutputChangeMillis>5000){
				writeServoPosition(0);
			}else{
				//skip and wait
				UdpTracer->println(F("Switch off: wait 5 seconds..."));
				return;
			}
		}else if (degree>servoMinValue && prevDegree<=servoMinValue){
			//now switchin on
			if(now-PrevOutputChangeMillis>5000){
				writeServoPosition(degree);
			}else{
				//skip and wait
				UdpTracer->println(F("Switch on: wait 5 seconds..."));
				return;
			}
		} else{
			//no problem, no transition ON->OFF or OFF->ON
			writeServoPosition(degree);
		}
	}else{
		if(degree>=servoMaxValue && IsServoOff()) return; //burner is already off
		if (degree>=servoMaxValue && prevDegree<servoMaxValue){
			//now swtiching off
			if(now-PrevOutputChangeMillis>5000){
				writeServoPosition(180);
			}else{
				//skip and wait
				UdpTracer->println(F("Switch off: wait 5 seconds..."));
				return;
			}
		}else if (degree<=servoMaxValue && prevDegree>servoMaxValue){
			//now switchin on
			if(now-PrevOutputChangeMillis>5000){
				writeServoPosition(degree);
			}else{
				//skip and wait
				UdpTracer->println(F("Switch on: wait 5 seconds..."));
				return;
			}
		} else{
			//no problem, no transition ON->OFF or OFF->ON
			writeServoPosition(degree);
		}
	}
	prevDegree = degree;
	PrevOutputChangeMillis = now;
}

void PidState::SetServoOff(bool value){
//	Serial.print(F("ServoOFF = "));
//	Serial.println(value?F("true"):F("false"));
	if(value!=servoOFF){
		if(!value)UdpTracer->println(F("Burner OFF->ON"));
		else UdpTracer->println(F("Burner ON->OFF"));
	}
	servoOFF=value;
}
bool PidState::IsServoOff(){return servoOFF;}

//bool PidState::IsServoUnderFireOff(){
//	if(servoDirection==ServoDirectionCW){
//		bool calculatedServoOff = Output<=servoMinValue;
//		if(calculatedServoOff){
//			if(!IsServoOff()){
////				Serial.println(F("SWITCH TO Fire OFF"));
//				setServoPosition(0);
//				SetServoOff(true);
//			}else{
////				Serial.println(F("Fire is OFF"));
//			}
//			return true;
//		}
//		SetServoOff(false);
//		return false;
//	}else{
//		if(servo.read()==180) return true;
//		if(Output>=servoMaxValue){
////			Serial.println(F("CCW Fire OFF"));
//			setServoPosition(180);
//			return true;
//		}
//		return false;
//	}
////	Serial.println(F("Fire is ON"));
//	return false;
//}

void PidState::updatePidStatus(){
	UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
	switch(fsmState){
	case psIdle:
			pid.SetTunings(kp,ki,kd);
			DynamicSetpoint=Setpoint;
		break;
	case psWaitDelay:
	case psRampimg:
			DynamicSetpoint=Setpoint; //this has to be recalculated time by time
			pid.SetTunings(kp,0.2,0);
		break;
	case psApproacing:
			DynamicSetpoint=Setpoint;
			pid.SetTunings(kp,ki,kd);
		break;
	case psKeepTemp:
			DynamicSetpoint=Setpoint;//FIXME build a ramp
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
	pDynamicSetpoint = DynamicSetpoint;
}
bool PidState::waitRampStart(){
	if(temperature-approacingStartTemp<0.1) return false;
	return true;
}
void PidState::updateRamp(){
	float now = millis();

	if (lastDynSetpointCalcMillis==0)lastDynSetpointCalcMillis=now;
	float deltaSecs = (now-lastDynSetpointCalcMillis)/(float)1000.0;
	if(deltaSecs>5){
		float roChange = 1.0/(float)60000.0;//°C/min
		if(abs(pDynamicSetpoint-Setpoint) <= roChange*(now-lastDynSetpointCalcMillis)){ //if the rate of change is going to push the drive past the setpoint, just make it equal the setpoint otherwise it'll oscillate
			DynamicSetpoint = Setpoint;
		}
		else{ //If more ramping is required, calculate the change required for the time period passed to keep the rate of change constant, and add it to the drive.
		  if(Setpoint>pDynamicSetpoint){  //positive direction
			  DynamicSetpoint = pDynamicSetpoint+(roChange*(now-lastDynSetpointCalcMillis));
		  }
		  else{   //negative direction
			  DynamicSetpoint = pDynamicSetpoint-(roChange*(now-lastDynSetpointCalcMillis));
		  }
		}
		if(temperature>=DynamicSetpoint){
			DynamicSetpoint=temperature;
			pid.Reset();
		}
		lastDynSetpointCalcMillis = now;;
		pDynamicSetpoint = DynamicSetpoint;

		lastDynSetpointCalcMillis = now;
		UdpTracer->print(F("New dyn setpoint:"));UdpTracer->printFloat(DynamicSetpoint,4);UdpTracer->println();
	}
}

void PidState::update(double temp,int encoderPos, boolean encoderPress){

	if(temp>-100){
		setTemperature(temp);
	}

	if(state!=svRunAuto && state!=svRunAutoTimer && state!=svRunAutoSetpoint &&
	   state!=svConfig_ServoDirection && state!=svConfig_ServoMin&&state!=svConfig_ServoMax)
	{
		pid.SetMode(MANUAL);
		pid.Initialize();
		Serial.println(F("PID switched to MANUAL"));
		Output=0;
		writeServoPosition(Output);
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

	ESP.wdtFeed();
//	Serial.print(F("State: "));Serial.println(state);
	switch(state){
		case svRunAuto :
		case svRunAutoSetpoint :
		case svRunAutoTimer : {
			if(temp<=-100){
				return;
			}

			if(pid.GetMode()!=AUTOMATIC){
				Serial.println(F("PID switched to AUTOMATIC"));
				pid.SetTunings(kp,ki,kd,P_ON_E);
				pid.SetControllerDirection(servoDirection==ServoDirectionCW?DIRECT:REVERSE);
				pid.SetOutputLimits(servoMinValue,servoMaxValue);
				pid.SetMode(AUTOMATIC);
			}

			float now = millis();
//			UdpTracer->print(F("Current state:"));UdpTracer->println(fsmState);
			switch(fsmState){
			case psIdle:
				if(temp<=Setpoint-1){
					fsmState = psWaitDelay;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					updatePidStatus();
					startRamp();
				} else if(temp>Setpoint-1 && temp<Setpoint){
					fsmState = psApproacing;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					updatePidStatus();
				} else if(temp>=Setpoint){
					fsmState = psKeepTemp;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					Output=0;
					pid.Initialize();
					updatePidStatus();
				}
				UdpTracer->print(F("NEW State:"));UdpTracer->println(fsmState);
				break;
			case psWaitDelay:
			case psRampimg:
				if(fsmState == psWaitDelay && waitRampStart()){
					fsmState = psRampimg;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					updatePidStatus();
//					pid.Initialize();
					startRamp();
				}
				if(temp<=Setpoint-1){
					//fsmState = psRampimg; //keep staying in ramping
					updateRamp();
				} else if(temp>Setpoint-1 && temp<Setpoint){
					fsmState = psApproacing;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
//					pid.Initialize();
					updatePidStatus();
				} else if(temp>=Setpoint){
					fsmState = psKeepTemp;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					Output=0;
					pid.Initialize();
					updatePidStatus();
				}
				break;
			case psApproacing:
				if(temp<=Setpoint-1){
					fsmState = psRampimg;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					updatePidStatus();
					startRamp();
				}  else if(temp>Setpoint-1 && temp<Setpoint){
					//fsmState = psApproacing; //remain in approacing
				} else if(temp>=Setpoint){
					fsmState = psKeepTemp;
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					Output=0;
					pid.Initialize();
					updatePidStatus();
				}
				break;
			case psKeepTemp:
				// a change in state only allowed if more than 30 seconds have passed
				if(temp<=Setpoint-1){
					fsmState = psRampimg; //possible? yes in case of power restore?
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
					updatePidStatus();
					startRamp();
				}  else if(temp>Setpoint-1 && temp<Setpoint){
					fsmState = psApproacing; //remain in approacing
					UdpTracer->print(F("State:"));UdpTracer->println(fsmState);
//					pid.Initialize();
					updatePidStatus();
				} else if(temp>=Setpoint){
					//fsmState = psKeepTemp;
				}
				break;
			}

			bool computed = pid.Compute();
//			if(!IsServoUnderFireOff()){
			setServoPosition(Output);
//			}

			if(computed){
				Serial.print(DynamicSetpoint,4);Serial.print(F(" "));Serial.print(Setpoint,4);Serial.print(F(" "));Serial.print(temp,4);Serial.print(F(" "));Serial.println(Output);
				UdpTracer->print(F("LOG:")      );UdpTracer->print(now,4);
			    UdpTracer->print(F(";SETP:")    );UdpTracer->print(Setpoint,4);
			    UdpTracer->print(F(";DSETP:")   );UdpTracer->print(DynamicSetpoint,4);
				UdpTracer->print(F(";TEMP:")    );UdpTracer->print(temp,4);
				UdpTracer->print(F(";OUT:")     );UdpTracer->print(Output,4);
				UdpTracer->print(F(";SERVOPOS:")     );UdpTracer->print((float)servoPosition,0);
				UdpTracer->print(F(";PGAIN:")   );UdpTracer->print(myPTerm,4);
				UdpTracer->print(F(";IGAIN:")   );UdpTracer->print(myITerm,4);
				UdpTracer->print(F(";DGAIN:")   );UdpTracer->print(myDTerm,4);
				UdpTracer->print(F(";OUTSUM:")  );UdpTracer->print(myOutputSum,4);
				UdpTracer->print(F(";PIDDTEMP:"));UdpTracer->println(myDInput,4);
			}
			break;
		}
		break;
		case svRunAutoTune:
			if(temp<=-100){
				return;
			}
			if(millis()-lastLog<1000 || temp<=-100){
				return;
			}
			lastLog = millis();
//			Serial.print(F("autoTune: "));Serial.println(autoTune);
			if(!autoTune)changeAutoTune(true);
			ESP.wdtFeed();
			if(!aTune.IsRunning()){
				autotuneSetPoint = temperature;
			}
			if (aTune.Runtime()!=0) {
//				Serial.print(F("AutoTune FINISHED!"));
				SetAutotuneResult(aTune.GetKp(),aTune.GetKi(),aTune.GetKd());
				changeAutoTune(false);
				SetState(svRunAutoTuneResult,false);
				break;
			}
//			else{
//				Serial.println(F("AutoTune Runtime done!"));
//			}

			setServoPosition(Output);
//			Serial.println(F("AutoTune Loop done!"));
			break;
	}
}

void PidState::loadFromEEProm(){
	Serial.print(F("EEPROMReadSettings: Expected EEPROM_VER:"));Serial.println(eepromVer);

	EEPROM.begin(512);
	byte temp=0;
	int addr = 0;
	temp = EEPROM.get(addr, temp);
	addr+=1;

	if(eepromVer!=03 && temp!=eepromVer){
		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
		return;
	}

	state=EEPROM.get(addr, state);
	//state=svRunAutoTuneResult;
	addr+=sizeof(PidStateValue);
	Serial.print(F("Readed state: "));Serial.println(state);
	if(state<0 || state>100){
		state=svMain;
		Serial.println(F(">>>>>>>> SOMETHING WENT WRONG.... ABORT READING!"));
		return;
	}

	servoDirection = EEPROM.get(addr, servoDirection);
	addr+=sizeof(ServoDirection);
	Serial.print(F("Readed servo dir: "));Serial.println(servoDirection);

	servoMinValue = EEPROM.get(addr, servoMinValue);
	addr+=sizeof(int);
	Serial.print(F("Readed servo min: "));Serial.println(servoMinValue);

	servoMaxValue = EEPROM.get(addr, servoMaxValue);
	addr+=sizeof(int);
	Serial.print(F("Readed servo max: "));Serial.println(servoMaxValue);

	Setpoint = EEPROM.get(addr, Setpoint);
	addr+=sizeof(double);
	Serial.print(F("Readed setpoint: "));Serial.println(Setpoint);

	kp = EEPROM.get(addr, kp);
	addr+=sizeof(double);
	Serial.print(F("Readed kp: "));Serial.println(kp);

	ki = EEPROM.get(addr, ki);
	addr+=sizeof(double);
	Serial.print(F("Readed ki: "));Serial.println(ki);

	kd = EEPROM.get(addr, kd);
	addr+=sizeof(double);
	Serial.print(F("Readed kd: "));Serial.println(kd);

	//if(eepromVer>3){
		pidSampleTimeSecs = EEPROM.get(addr, pidSampleTimeSecs);
		addr+=sizeof(double);
		Serial.print(F("Readed Sample time secs: "));Serial.println(pidSampleTimeSecs);
		if(pidSampleTimeSecs==NAN)pidSampleTimeSecs=5;
//	}

	EEPROM.commit();
	EEPROM.end();

	Serial.print(F("Size: "));Serial.println(addr);
}

void PidState::saveSetPointTotoEEprom(){
	EEPROM.begin(64);

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

void PidState::saveServoDirToEEprom(){
	EEPROM.begin(64);

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

void PidState::savetoEEprom(){
	ESP.wdtFeed();
	EEPROM.begin(64);

	int addr = 0;
	EEPROM.put(addr, eepromVer);
	addr+=1;

	Serial.print(F("EEPROMWriteSettings. pid state: "));Serial.println(state);
	EEPROM.put(addr, state);
	addr+=sizeof(PidStateValue);

	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(servoDirection);
	EEPROM.put(addr, servoDirection);
	addr+=sizeof(ServoDirection);

	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(servoMinValue);
	EEPROM.put(addr, servoMinValue);
	addr+=sizeof(int);

	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(servoMaxValue);
	EEPROM.put(addr, servoMaxValue);
	addr+=sizeof(int);

	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(Setpoint);
	EEPROM.put(addr, Setpoint);
	addr+=sizeof(double);

	Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(kp);
	EEPROM.put(addr, kp);
	addr+=sizeof(double);

	Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(ki);
	EEPROM.put(addr, ki);
	addr+=sizeof(double);

	Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(kd);
	EEPROM.put(addr, kd);
	addr+=sizeof(double);

	Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(pidSampleTimeSecs);
	EEPROM.put(addr, pidSampleTimeSecs);
	addr+=sizeof(double);

	EEPROM.commit();
	EEPROM.end();

	ESP.wdtFeed();
}


