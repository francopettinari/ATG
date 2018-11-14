/*
 * LCD.cpp
 *
 *  Created on: 29 ott 2018
 *      Author: franc
 */

//LCD 20x4
#include <Wire.h>
#include "LCDHelper.h"
#include "PidState.h"

LCDHelper::LCDHelper():lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE) {
	Wire.begin(D2,D1);
	lcd.begin(20,4);
	lcd.clear();
}

void LCDHelper::Space (byte num){
  for(byte i = 0; i < num; i++){
    lcd.print(F(" "));
  }
}

void LCDHelper::Clear(byte row) {
  lcd.setCursor(0, row);
  Space(20);
}

void LCDHelper::display(PidState pstate){
//	Serial.print(F(" "));Serial.print(pstate.stateSelection);
//	Serial.print(F(" "));
	lcd.setCursor(0, 0);
	lcd.print(pstate.getCurrentMenu()->Caption);lcd.print(F("   "));

	if(pstate.state!=RunAuto){
		for(int idx = pstate.currMenuStart;idx<pstate.currMenuStart+3;idx++){
			lcd.setCursor(0, idx+1-pstate.currMenuStart);
			if(idx>pstate.getCurrentMenu()->subMenuItemsLen-1){
				lcd.print(F("     "));
				continue;
			}
			if(idx == pstate.stateSelection){
				lcd.print(F(">"));
			}else{
				lcd.print(F(" "));
			}
			lcd.print(pstate.getCurrentMenu()->subMenuItems[idx]->Caption);
			lcd.print(F("    "));
		}
	}
	switch(pstate.state) {
		case None:
		case RunAuto:
			displayRun(pstate);
			break;
		case Config_ServoDirection:
		case Config_ServoMin:
		case Config_ServoMax:
			displayConfigServo(pstate);
			break;
	}

//	lcd.setCursor(0, 3);lcd.print(pstate.currEncoderPos);
//	lcd.setCursor(10, 3);lcd.print(pstate.stateSelection);
//	Serial.println(F(" "));
}

void LCDHelper::displayRun(PidState pstate){

	lcd.setCursor(14, 0);
	lcd.print(pstate.getTemperature(),2);

	lcd.setCursor(14, 1);
	lcd.print(pstate.Setpoint,2);

	lcd.setCursor(14, 2);
	lcd.print(pstate.Output,2);

}

void LCDHelper::displayConfigServo(PidState pstate){
	lcd.setCursor(15, 0);
	if(pstate.servoDirection==ServoDirectionCW){
		lcd.print(F("CW   "));
	}else if(pstate.servoDirection==ServoDirectionCCW){
		lcd.print(F("CCW  "));
	}

	lcd.setCursor(15, 1);
	lcd.print(pstate.servoMin);lcd.print(F("     "));

	lcd.setCursor(15, 2);
	lcd.print(pstate.servoMax);lcd.print(F("     "));
}

void LCDHelper::print(byte col, byte row, int val){
	lcd.setCursor(col, row);
	lcd.print(val);
}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.setCursor(col, row);
	lcd.print(ifsh);
}

