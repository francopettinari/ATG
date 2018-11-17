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
			displayRun(pstate);
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

void LCDHelper::displayRun(PidState pstate){
	lcd.PrintDouble(14, 0,pstate.getTemperature(),2);
	lcd.PrintDouble(14, 1,pstate.Setpoint,2);
	lcd.PrintDouble(14, 2,pstate.Output,2);

}

void LCDHelper::displayConfigServo(PidState pstate){
	if(pstate.servoDirection==ServoDirectionCW){
		lcd.PrintF(15, 0,F("CW   "));
	}else if(pstate.servoDirection==ServoDirectionCCW){
		lcd.PrintF(15, 0,F("CCW  "));
	}

	lcd.PrintDouble(15, 1,pstate.servoMin);

	lcd.PrintDouble(15, 2,pstate.servoMax);
}

//void LCDHelper::print(byte col, byte row, int val){
//	lcd.Print(col, row,val);
//}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.PrintF(col, row,ifsh);
}

