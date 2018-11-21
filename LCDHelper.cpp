/*
 * LCD.cpp
 *
 *  Created on: 29 ott 2018
 *      Author: franc
 */

//LCD 20x4

#include "LCDHelper.h"
#include "PidState.h"

LCDHelper::LCDHelper():lcd(20,4) {
}

void LCDHelper::display(PidState pstate){
//	Serial.print(F(" "));Serial.print(pstate.stateSelection);
//	Serial.print(F(" "));
	lcd.clear();
	lcd.PrintString(0,0,pstate.getCurrentMenu()->Caption.c_str());
	if(pstate.getState()!=svRunAuto){
		for(int idx = pstate.currMenuStart;idx<pstate.currMenuStart+3;idx++){
			int y=idx+1-pstate.currMenuStart;
			if(idx == pstate.stateSelection){
				lcd.PrintF(0,y,F(">"));
			}else{
				lcd.PrintF(0,y,F(" "));
			}
			if(idx<pstate.getCurrentMenu()->subMenuItemsLen){
				//Serial.println(pstate.getCurrentMenu()->subMenuItemsLen);
				lcd.PrintString(1,y,pstate.getCurrentMenu()->subMenuItems[idx]->Caption.c_str());
//				Serial.print(idx);Serial.print(F(" "));Serial.println(pstate.getCurrentMenu()->subMenuItems[idx]->Caption.c_str());
			}
		}
	}
	switch(pstate.getState()) {
		case svRunAuto:
		case svRunAutoTune:
			displayRun(pstate);
			break;
		case svPidConfig:
		case svPidKpiConfig:
		case svPidKpdConfig:
		case svPidKiiConfig:
		case svPidKidConfig:
		case svPidKdiConfig:
		case svPidKddConfig:
			displayConfigPid(pstate);
			break;
		case svServo_Config:
		case svConfig_ServoDirection:
		case svConfig_ServoMin:
		case svConfig_ServoMax:
			displayConfigServo(pstate);
			break;
	}
	lcd.render();
//	lcd.setCursor(0, 3);lcd.print(pstate.currEncoderPos);
//	lcd.setCursor(10, 3);lcd.print(pstate.stateSelection);
//	Serial.println(F(" "));
}

void LCDHelper::displayConfigPid(PidState pstate){
	lcd.PrintF(11, 0,F("Kp"));lcd.PrintFloat(14, 0,pstate.kp);
	lcd.PrintF(11, 1,F("Ki"));lcd.PrintFloat(14, 1,pstate.ki);
	lcd.PrintF(11, 2,F("Kd"));lcd.PrintFloat(14, 2,pstate.kd);
}

void LCDHelper::displayRun(PidState pstate){
	lcd.PrintDouble(14, 0,pstate.getTemperature(),2);
	lcd.PrintDouble(14, 1,pstate.Setpoint,2);
	lcd.PrintDouble(14, 2,pstate.Output,2);
	lcd.PrintDouble(14, 3,pstate.servoPos,2);
//	int degree = pstate.servoPos;
//	if(pstate.servoDirection==ServoDirectionCCW){
//		degree = pstate.servoMax  - degree;
//	}
//	lcd.PrintDouble(14, 3,degree,2);
}

void LCDHelper::displayConfigServo(PidState pstate){
	if(pstate.servoDirection==ServoDirectionCW){
		lcd.PrintF(11, 0,F("Dir CW"));
	}else if(pstate.servoDirection==ServoDirectionCCW){
		lcd.PrintF(11, 0,F("Dir CCW"));
	}
	lcd.PrintF(11, 1,F("Min "));lcd.PrintDouble(15, 1,pstate.servoMin);
	lcd.PrintF(11, 2,F("Max "));lcd.PrintDouble(15, 2,pstate.servoMax);
}

//void LCDHelper::print(byte col, byte row, int val){
//	lcd.Print(col, row,val);
//}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.PrintF(col, row,ifsh);
}

