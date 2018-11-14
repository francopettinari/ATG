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

	switch(state){
		case None:
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
				MenuItem* selMI = currentMenu->subMenuItems[stateSelection];
				if(selMI->callBack!=NULL){
					currentMenu->subMenuItems[stateSelection]->callBack(selMI->parent);
				}
			}
			break;
		case RunAuto :
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
				Setpoint -= 0.25;
				if(Setpoint<0)Setpoint=0;
				savetoEEprom();
			}else if(encMovement==EncMoveCW){
				Setpoint+=0.25;
				if(Setpoint>=120)Setpoint=120;
				savetoEEprom();
			}

			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = None;
//				savetoEEprom();
			}
			break;
		case Config_ServoDirection:
			if(encMovement==EncMoveCCW){
				if(servoDirection==ServoDirectionCW)servoDirection=ServoDirectionCCW;
			}else if(encMovement==EncMoveCW){
				if(servoDirection==ServoDirectionCCW)servoDirection=ServoDirectionCW;
			}
			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = None;
				savetoEEprom();
			}else{
				setServoPosition(servoMin);
			}
			break;
		case Config_ServoMin:
			if(encMovement==EncMoveCCW){
				servoMin--;
				if(servoMin<0)servoMin=0;
			}else if(encMovement==EncMoveCW){
				if(servoMin<servoMax-1)servoMin++;
				if(servoMin>=180)servoMin=179;
			}
			setServoPosition(servoMin);

			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = None;
				savetoEEprom();
			}

			break;
		case Config_ServoMax:
			if(encMovement==EncMoveCCW){
				if(servoMax>servoMin+1)servoMax--;
				if(servoMax<=0)servoMax=1;
			}else if(encMovement==EncMoveCW){
				servoMax++;
				if(servoMax>180)servoMax=180;
			}
			setServoPosition(servoMax);

			if(encoderPushButtonState==EncoderPushButtonPressed){
				state = None;
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
	if(state<0 || state>10){
		state=None;
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


