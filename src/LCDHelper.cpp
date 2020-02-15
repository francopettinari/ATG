/*
 * LCD.cpp
 *
 *  Created on: 29 ott 2018
 *      Author: franc
 */

//LCD 20x4

#include "LCDHelper.h"

#include "Controller.h"

byte tempCustomChar[] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B01010,
  B10001,
  B10001,
  B01110
};

byte degreeCustomChar[] = {
		B01100,
		  B10010,
		  B10010,
		  B01100,
		  B00000,
		  B00000,
		  B00000,
		  B00000
		};
byte setpointCustomChar[] = {
		B11100,
		B10000,
		B11100,
		B00111,
		B11101,
		B00111,
		B00100,
		B00100
};

byte heatCustomChar[] = {
		 B01110,
		  B10001,
		  B00000,
		  B01110,
		  B10001,
		  B00000,
		  B01110,
		  B10001
};

byte derivateCustomChar[] = {
		  B00001,
		  B01101,
		  B10011,
		  B10011,
		  B01100,
		  B00000,
		  B00000,
		  B00000
};

byte degMinChar[] = {
  B11000,
  B11000,
  B00000,
  B11111,
  B00000,
  B01010,
  B10101,
  B10101
};

byte timerChar[] = {
  B00000,
  B00000,
  B01110,
  B10011,
  B10101,
  B10001,
  B01110,
  B00000
};


static const int TEMPERATURE_CHAR = 0;
static const int DEGREE_CHAR = 1;
static const int SETPOINT_CHAR = 2;
static const int HEAT_CHAR = 3;
static const int DERIV_CHAR = 4;
static const int DEGMIN_CHAR = 5;
static const int TIMER_CHAR = 6;

LCDHelper::LCDHelper():lcd(20,4) {
	lcd.getLcd()->createChar(TEMPERATURE_CHAR, tempCustomChar);
	lcd.getLcd()->createChar(DEGREE_CHAR, degreeCustomChar);
	lcd.getLcd()->createChar(SETPOINT_CHAR, setpointCustomChar);
	lcd.getLcd()->createChar(HEAT_CHAR, heatCustomChar);
	lcd.getLcd()->createChar(DERIV_CHAR, derivateCustomChar);
	lcd.getLcd()->createChar(DEGMIN_CHAR, degMinChar);
	lcd.getLcd()->createChar(TIMER_CHAR, timerChar);
}

void LCDHelper::display(PidState pstate){
	lcd.clear();
	lcd.PrintString(0,0,pstate.getCurrentMenu()->Caption.c_str());
	std::vector<MenuItem *> subMenuItems = pstate.getCurrentMenu()->subMenuItems;
	int max = subMenuItems.size()-1;
	int y=1;
	for(int idx = pstate.currMenuStart;idx<=max;idx++){
		if(y>3) break;
		if(!subMenuItems[idx]->Visible) {
			//Serial.print(subMenuItems[idx]->Caption.c_str());Serial.println(F(" NOT VISIBLE"));
			continue;
		}
		//int y=idx+1-pstate.currMenuStart;
		if(idx == pstate.stateSelection){
			lcd.PrintF(0,y,F(">"));
		}else{
			lcd.PrintF(0,y,F(" "));
		}

		if(idx<=max && subMenuItems[idx]->Selected){
			lcd.PrintF(0,y,F("o"));
		}
		if(idx<=max){
			lcd.PrintString(1,y,pstate.getCurrentMenu()->subMenuItems[idx]->Caption.c_str());
		}
		y++;
	}
	switch(pstate.getState()) {
		case svRunAuto:
		case svRunAutoSetpoint:
			displayRun(pstate);
			break;
		case svPidConfig:
		case svPidKpiConfig:
		case svPidKpdConfig:
		case svPidKiiConfig:
		case svPidKidConfig:
		case svPidKicConfig:
		case svPidKdiConfig:
		case svPidKddConfig:
		case svPidSampleTimeConfig:
			displayConfigPid(pstate);
			break;
		case svServo_Config:
		case svConfig_ServoDirection:
		case svConfig_ServoMin:
		case svConfig_ServoMax:
			displayConfigServo(pstate);
			break;
		case svConfig_Probe:
			displayConfigProbe(pstate);
			break;
		default:
			displayDefault(pstate);
			break;
	}
	lcd.render();
//	lcd.setCursor(0, 3);lcd.print(pstate.currEncoderPos);
//	lcd.setCursor(10, 3);lcd.print(pstate.stateSelection);
//	Serial.println(F(" "));
}

void LCDHelper::displayDefault(PidState pstate){
	lcd.PrintChar(13, 0,(char)TEMPERATURE_CHAR); lcd.PrintDoubleFD(14, 0,pstate.getTemperature(),2,1);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);
}

void LCDHelper::displayConfigPid(PidState pstate){
	lcd.PrintF(10, 0,F("Kp"));lcd.PrintDoubleFD(13, 0,pstate.kp,2,3);
	lcd.PrintF(10, 1,F("Ki"));lcd.PrintDoubleFD(13, 1,pstate.ki,2,3);
	lcd.PrintF(10, 2,F("Kd"));lcd.PrintDoubleFD(13, 2,pstate.kd,2,3);
	lcd.PrintF(10, 3,F("St"));lcd.PrintDoubleFD(16, 3,pstate.pidSampleTimeSecs,2,0);
}

void LCDHelper::displayRun(PidState pstate){
	int spPos = pstate.Setpoint<100?17:16;
	int tPos = pstate.getTemperature()<100?spPos-5:spPos-6;
	/*lcd.PrintChar(tPos-1, 0,(char)TEMPERATURE_CHAR);*/lcd.PrintDoubleFD(tPos, 0,pstate.getTemperature(),2,1);lcd.PrintChar(spPos-1, 0,'/'); lcd.PrintDoubleFD(spPos, 0,pstate.Setpoint,2,0);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);
	int o = pstate.getOutPerc();
	/*lcd.PrintChar(13, 1,(char)HEAT_CHAR);*/ lcd.PrintDoubleD(15, 1,o,0); 	lcd.PrintF(19, 1,F("%"));
	/*lcd.PrintF(13, 2,F("/"));*/ lcd.PrintDoubleD(15, 2,pstate.Ramp,0);lcd.PrintChar(19, 2,(char)DEGMIN_CHAR);
}

void LCDHelper::displayConfigServo(PidState pstate){
	if(pstate.servoDirection==ServoDirectionCW){
		lcd.PrintF(11, 0,F("Dir CW"));
	}else if(pstate.servoDirection==ServoDirectionCCW){
		lcd.PrintF(11, 0,F("Dir CCW"));
	}
	lcd.PrintF(11, 1,F("Min "));lcd.PrintDoubleFD(15, 1,pstate.servoMinValue,3,0);
	lcd.PrintF(11, 2,F("Max "));lcd.PrintDoubleFD(15, 2,pstate.servoMaxValue,3,0);
}

void LCDHelper::displayConfigProbe(PidState pstate){
	lcd.PrintF(5, 0,F("Temp correction"));
	lcd.PrintDoubleFD(15, 2,pstate.temperatureCorrection,3,0);
}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.PrintF(col, row,ifsh);
}
