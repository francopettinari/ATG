/*
 * PidState.cpp
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#include "PidState.h"
#include <EEPROM.h>

//PidState::PidState() : pid(&temperature, &Output, &Setpoint, 95,4,0,P_ON_E, DIRECT){
PidState::PidState() : pid(&temperature, &Output, &Setpoint, 2, 0.5, 2, DIRECT),aTune(&temperature, &Output){
	//pid.SetTunings(r_set(1) - 100, (double)((r_set(2) - 100.00) / 250.00), r_set(3) - 100); // send the PID settings to the PID
	//myPID.SetOutputLimits(0.0, 255.0);
	pid.SetSampleTime(4 * 250);
	pid.SetMode(AUTOMATIC);
	servo.attach(D8);  // attaches the servo on pin 9 to the servo object
	//setServoPosition(0);
	currentMenu = new MainMenu();

	aTune.SetNoiseBand(1);
	aTune.SetLookbackSec(200);
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
		case svRunAutoTune :
			return pmm->runMenu->runAutoTuneMenu;
		case svConfig:
			return pmm->configMenu;
		case svTempConfig:
			return pmm->configMenu->tempCorrectionMenu;
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
			return pmm->configMenu->pidMenu->kiMenu;
		case svPidKdiConfig:
		case svPidKddConfig:
			return pmm->configMenu->pidMenu->kdMenu;
	}
	return pmm;
}

void PidState::changeAutoTune(int value)
{
 if(value>0) {
    //Set the output to the desired starting frequency.
	Output=(servoMax-servoMin)/2+servoMin;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(50);
    aTune.SetLookbackSec(10);
    aTune.SetControlType(0);
    pid.SetMode(AUTOMATIC);
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
		if(lastPressMillis>0 ){
			if ((millis()-lastPressMillis)<500){
				return EncoderPushButtonKeepedPressed;
			}
		}
		lastPressMillis=millis();
		return EncoderPushButtonPressed;
	}else{
		lastPressMillis = -1;
	}
	return EncoderPushButtonNone;
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

void PidState::setServoPosition(int degree){
	if(servoDirection==ServoDirectionCW){
		servo.write(degree + servoMin);
//		Serial.print("Servo pos    :");Serial.println(degree + servoMin);
	}else{
		servo.write(servoMax  - degree);
//		Serial.print("Servo pos    :");Serial.println(servoMax  - degree);
	}
	delay(15);
}

void PidState::update(double temp,int encoderPos, boolean encoderPress){

	setTemperature(temp);

	//encoder position
	EncoderMovement encMovement = decodeEncoderMoveDirection(encoderPos);

	//encoder push button
	EncoderPushButtonState encoderPushButtonState = decodeEncoderPushBtnState(encoderPress);


	currentMenu = decodeCurrentMenu();

	switch(state){
		case svUndefiend:
		case svMain:
		case svConfig:
		case svPidConfig:
		case svServo_Config:
		case svRun:
			if(encMovement==EncMoveCCW){
				stateSelection--;
				if(stateSelection<0) stateSelection=0;
				if(stateSelection<currMenuStart)currMenuStart--;
			}else if(encMovement==EncMoveCW){
				stateSelection++;
				if(stateSelection>currentMenu->subMenuItemsLen-1) stateSelection=currentMenu->subMenuItemsLen-1;
				if(stateSelection-currMenuStart>=3)currMenuStart++;
			}
			if(encoderPushButtonState==EncoderPushButtonPressed){
				Serial.print(F(">>>>>> Push <<<<<<  "));Serial.println(stateSelection);
				Serial.print("Menu    :");Serial.println(getCurrentMenu()->Caption);
				Serial.print("Menu len:");Serial.println(getCurrentMenu()->subMenuItemsLen);
				MenuItem* selMI = currentMenu->subMenuItems[stateSelection];
				selMI->OnPress();
			}
			break;

		case svRunAuto :
			//TODO move into a dedicated function
			pid.Compute();
//			Serial.print("Range degree:");Serial.println(servoMax-servoMin);
//			Serial.print("Pos degree:");Serial.println(posDegree);
			servoPos = (Output*(servoMax-servoMin))/255;
			if(servoDirection==ServoDirectionCW){
				servoPos = servoPos + servoMin;
			}else{
				servoPos = servoMax -  servoPos;
			}
			setServoPosition(servoPos);

			if(millis()-lastLog>=1000){
				Serial.print(temp);Serial.print(F(" "));Serial.print(Output);Serial.print(F(" "));Serial.println(servoPos);
				lastLog = millis();
			}

			if(encMovement==EncMoveCCW){
				Setpoint -= 1;
				if(Setpoint<0)Setpoint=0;
//				saveSetPointTotoEEprom();
			}else if(encMovement==EncMoveCW){
				Setpoint+=1;
				if(Setpoint>=120)Setpoint=120;
//				saveSetPointTotoEEprom();
			}
			if(encoderPushButtonState==EncoderPushButtonPressed){
				SetState(svRun);
			}
			break;
		case svRunAutoTune:
			if(millis()-lastLog<1000){
				return;
			}
			lastLog = millis();
			Serial.print("autoTune: ");Serial.println(autoTune);
			if(!autoTune)changeAutoTune(true);
			if (aTune.Runtime()!=0) {
				state = svRun;
				kp = aTune.GetKp();
				ki = aTune.GetKi();
				kd = aTune.GetKd();
				pid.SetTunings(kp,ki,kd);
				pid.SetMode(AUTOMATIC);
				changeAutoTune(false);
			}

			servoPos = (Output*(servoMax-servoMin))/255;
			if(servoDirection==ServoDirectionCW){
				servoPos = servoPos + servoMin;
			}else{
				servoPos = servoMax -  servoPos;
			}
			setServoPosition(servoPos);

			if(encoderPushButtonState==EncoderPushButtonPressed){
				SetState(svRun);
			}

			break;
		case svConfig_ServoDirection:
			if(encMovement==EncMoveCCW){
				if(servoDirection==ServoDirectionCW)servoDirection=ServoDirectionCCW;
			}else if(encMovement==EncMoveCW){
				if(servoDirection==ServoDirectionCCW)servoDirection=ServoDirectionCW;
			}
			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = svServo_Config;
				saveServoDirToEEprom();
			}else{
				setServoPosition(servoMin);
			}
			break;
		case svConfig_ServoMin:
			if(encMovement==EncMoveCCW){
				servoMin--;
				if(servoMin<0)servoMin=0;
			}else if(encMovement==EncMoveCW){
				if(servoMin<servoMax-1)servoMin++;
				if(servoMin>=180)servoMin=179;
			}
			setServoPosition(servoMin);

			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = svServo_Config;
				savetoEEprom();
			}

			break;
		case svConfig_ServoMax:
			if(encMovement==EncMoveCCW){
				if(servoMax>servoMin+1)servoMax--;
				if(servoMax<=0)servoMax=1;
			}else if(encMovement==EncMoveCW){
				servoMax++;
				if(servoMax>180)servoMax=180;
			}
			setServoPosition(servoMax);

			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = svServo_Config;
				savetoEEprom();
			}
			break;

		//KP
		case svPidKpiConfig: {
				int ikp = (int)kp;
				float decPart = (kp-ikp);
				if(encMovement==EncMoveCCW){
					ikp--;
				}else if(encMovement==EncMoveCW){
					ikp++;
				}
				kp=ikp+decPart;
				if(encoderPushButtonState==EncoderPushButtonPressed){
					state = svPidKpdConfig;
					savetoEEprom();
				}
			}
			break;
		case svPidKpdConfig: {
				int ikp = (int)kp;
				float decPart = (kp-ikp);
				decPart = (int)(decPart*100);
				if(encMovement==EncMoveCCW){
					Serial.println(F("CCW"));
					decPart-=1.0;
				}else if(encMovement==EncMoveCW){
					Serial.println(F("CW"));
					decPart+=1.0;
				}
				kp=ikp+(decPart/100.0);
				if(encoderPushButtonState==EncoderPushButtonPressed){
					state = svPidConfig;
					savetoEEprom();
				}
			}
			break;

		//KI
		case svPidKiiConfig: {
				int iki = (int)ki;
				float decPart = (ki-iki);
				if(encMovement==EncMoveCCW){
					iki--;
				}else if(encMovement==EncMoveCW){
					iki++;
				}
				ki=iki+decPart;
				if(encoderPushButtonState==EncoderPushButtonPressed){
					state = svPidKidConfig;
					savetoEEprom();
				}
			}
			break;
		case svPidKidConfig: {
				int iki = (int)ki;
				float decPart = (ki-iki);
				decPart = (int)(decPart*100.0);
				if(encMovement==EncMoveCCW){
					decPart-=1.0;
				}else if(encMovement==EncMoveCW){
					decPart+=1.0;
				}
				ki=iki+(decPart/100.0);
				if(encoderPushButtonState==EncoderPushButtonPressed){
					state = svPidConfig;
					savetoEEprom();
				}
			}
			break;

		//KD
		case svPidKdiConfig: {
				int ikd = (int)kd;
				float decPart = (kd-ikd);
				if(encMovement==EncMoveCCW){
					ikd-=1.0;
				}else if(encMovement==EncMoveCW){
					ikd+=1.0;
				}
				kd=ikd+decPart;
				if(encoderPushButtonState==EncoderPushButtonPressed){
					state = svPidKddConfig;
					savetoEEprom();
				}
			}
			break;
		case svPidKddConfig: {
				int ikd = (int)kd;
				float decPart = (kd-ikd);
				decPart = (int)(decPart*100.0);
				if(encMovement==EncMoveCCW){
					decPart-=1.0;
				}else if(encMovement==EncMoveCW){
					decPart+=1.0;
				}
				ki=ikd+(decPart/100.0);
				if(encoderPushButtonState==EncoderPushButtonPressed){
					state = svPidConfig;
					savetoEEprom();
				}
			}
			break;
		case svTempConfig :
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

	if(temp!=eepromVer){
		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
		return;
	}

	state=EEPROM.get(addr, state);
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

	servoMin = EEPROM.get(addr, servoMin);
	addr+=sizeof(int);
	Serial.print(F("Readed servo min: "));Serial.println(servoMin);

	servoMax = EEPROM.get(addr, servoMax);
	addr+=sizeof(int);
	Serial.print(F("Readed servo max: "));Serial.println(servoMax);

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

	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(servoMin);
	EEPROM.put(addr, servoMin);
	addr+=sizeof(int);

	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(servoMax);
	EEPROM.put(addr, servoMax);
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



	EEPROM.commit();
	EEPROM.end();
}


