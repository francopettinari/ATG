/*
 * PidState.cpp
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#include "PidState.h"
#include <EEPROM.h>

//PID myPID(&Input, &Output, &Setpoint, 100, 40, 0, DIRECT);

PidState::PidState() : pid(&temperature, &Output, &Setpoint, 100,40,0, DIRECT){
	//pid.SetTunings(r_set(1) - 100, (double)((r_set(2) - 100.00) / 250.00), r_set(3) - 100); // send the PID settings to the PID
	//myPID.SetOutputLimits(0.0, 255.0);
	pid.SetSampleTime(4 * 250);
	pid.SetMode(AUTOMATIC);
	servo.attach(D8);  // attaches the servo on pin 9 to the servo object
	currentMenu = new MainMenu();
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
	}
	return pmm;
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
		servo.write(degree);
	}else{
		servo.write(180-degree);
	}
}

void PidState::update(double temp,int encoderPos, boolean encoderPress){

	setTemperature(temp);

	//encoder position
	EncoderMovement encMovement = decodeEncoderMoveDirection(encoderPos);

	//encoder push button
	EncoderPushButtonState encoderPushButtonState = decodeEncoderPushBtnState(encoderPress);
	int servoPos;

	currentMenu = decodeCurrentMenu();

	switch(state){
		case svMain:
		case svConfig:
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
			pid.Compute();   // was 6, getting close, start feeding the PID -mdw
			servoPos = Output*(servoMax-servoMin)/254;
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
				savetoEEprom();
			}else if(encMovement==EncMoveCW){
				Setpoint+=1;
				if(Setpoint>=120)Setpoint=120;
				savetoEEprom();
			}

			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = svRun;
//				savetoEEprom();
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
				savetoEEprom();
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
	}


}

void PidState::loadFromEEProm(){
	Serial.print(F("EEPROMReadSettings: Expected EEPROM_VER:"));Serial.println(eepromVer);

	EEPROM.begin(512);
	byte temp;
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

	EEPROM.commit();
	EEPROM.end();


}

void PidState::savetoEEprom(){

	EEPROM.begin(512);

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

	EEPROM.commit();
	EEPROM.end();
}


