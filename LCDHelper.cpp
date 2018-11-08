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
	lcd.print(pstate.getCurrentMenu()->Caption);lcd.print(F("              "));
	for(int idx = 0;idx<3;idx++){
		lcd.setCursor(0, idx+1);
		if(idx>pstate.getCurrentMenu()->subMenuItemsLen-1){
			lcd.print(F("              "));
			continue;
		}
		if(idx == pstate.stateSelection){
			lcd.print(F(">"));
		}else{
			lcd.print(F(" "));
		}
		lcd.print(pstate.getCurrentMenu()->subMenuItems[idx]->Caption);
		lcd.print(F("         "));
	}
	lcd.setCursor(0, 3);lcd.print(pstate.currEncoderPos);
	lcd.setCursor(10, 3);lcd.print(pstate.stateSelection);
//	Serial.println(F(" "));
}

void LCDHelper::print(byte col, byte row, int val){
	lcd.setCursor(col, row);
	lcd.print(val);
}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.setCursor(col, row);
	lcd.print(ifsh);
}

