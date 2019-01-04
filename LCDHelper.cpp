/*
 * LCD.cpp
 *
 *  Created on: 29 ott 2018
 *      Author: franc
 */

//LCD 20x4

#include "LCDHelper.h"
#include "PidState.h"

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


static const int TEMPERATURE_CHAR = 0;
static const int DEGREE_CHAR = 1;
static const int SETPOINT_CHAR = 2;
static const int HEAT_CHAR = 3;
static const int DERIV_CHAR = 4;
static const int DEGMIN_CHAR = 5;

LCDHelper::LCDHelper():lcd(20,4) {
	lcd.getLcd()->createChar(TEMPERATURE_CHAR, tempCustomChar);
	lcd.getLcd()->createChar(DEGREE_CHAR, degreeCustomChar);
	lcd.getLcd()->createChar(SETPOINT_CHAR, setpointCustomChar);
	lcd.getLcd()->createChar(HEAT_CHAR, heatCustomChar);
	lcd.getLcd()->createChar(DERIV_CHAR, derivateCustomChar);
	lcd.getLcd()->createChar(DEGMIN_CHAR, degMinChar);
}

void LCDHelper::display(PidState pstate){
//	Serial.print(F(" "));Serial.print(pstate.stateSelection);
//	Serial.print(F(" "));
	//Serial.println(F("LCDHelper::display"));
	lcd.clear();
	lcd.PrintString(0,0,pstate.getCurrentMenu()->Caption.c_str());

	//if(pstate.getState()!=svRunAuto){
		for(int idx = pstate.currMenuStart;idx<pstate.currMenuStart+3;idx++){
			int y=idx+1-pstate.currMenuStart;
			if(idx == pstate.stateSelection){
				lcd.PrintF(0,y,F(">"));
			}else{
				lcd.PrintF(0,y,F(" "));
			}

			if(idx<pstate.getCurrentMenu()->subMenuItemsLen() && pstate.getCurrentMenu()->subMenuItems[idx]->Selected){
				lcd.PrintF(0,y,F("o"));
			}
			if(idx<pstate.getCurrentMenu()->subMenuItemsLen()){
				//Serial.println(pstate.getCurrentMenu()->subMenuItemsLen);
				lcd.PrintString(1,y,pstate.getCurrentMenu()->subMenuItems[idx]->Caption.c_str());
//				Serial.print(idx);Serial.print(F(" "));Serial.println(pstate.getCurrentMenu()->subMenuItems[idx]->Caption.c_str());
			}
		}
//	}
	switch(pstate.getState()) {
		case svRunAuto:
		case svRunAutoSetpoint:
		case svRunAutoTimer:
			displayRun(pstate);
			break;
		case svRunManual:
			displayManual(pstate);
			break;
		case svRunAutoTune:
			displayAutoTune(pstate);
			break;
		case svRunAutoTuneResult:
			displayAutoTuneResult(pstate);
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
	lcd.PrintChar(13, 0,(char)TEMPERATURE_CHAR); lcd.PrintDouble(14, 0,pstate.getTemperature(),1);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);
}

void LCDHelper::displayConfigPid(PidState pstate){
	lcd.PrintF(10, 0,F("Kp"));lcd.PrintDouble(13, 0,pstate.kp,3);
	lcd.PrintF(10, 1,F("Ki"));lcd.PrintDouble(13, 1,pstate.ki,3);
	lcd.PrintF(10, 2,F("Kd"));lcd.PrintDouble(13, 2,pstate.kd,3);
	lcd.PrintF(10, 3,F("St"));lcd.PrintDouble(16, 3,pstate.pidSampleTimeSecs,0);
}

void LCDHelper::displayAutoTuneResult(PidState pstate){
	lcd.PrintF(10, 0,F("Confirm ?"));
	lcd.PrintF(10, 1,F("Kp"));lcd.PrintDouble(14, 1,pstate.akp,3);
	lcd.PrintF(10, 2,F("Ki"));lcd.PrintDouble(14, 2,pstate.aki,3);
	lcd.PrintF(10, 3,F("Kd"));lcd.PrintDouble(14, 3,pstate.akd,3);
}

void LCDHelper::displayRun(PidState pstate){
	lcd.PrintChar(12, 0,(char)TEMPERATURE_CHAR); lcd.PrintDouble(13, 0,pstate.getTemperature(),2);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);
	lcd.PrintChar(12, 1,(char)SETPOINT_CHAR);    lcd.PrintDouble(13, 1,pstate.Setpoint        ,2);lcd.PrintChar(19, 1,(char)DEGREE_CHAR);

	int o = 0;
	double outRange = pstate.servoMaxValue-pstate.servoMinValue;
	if(pstate.servoDirection==ServoDirectionCW){
			if(pstate.Output<pstate.servoMinValue) o = 0;
			else o = 100.0*(pstate.Output-pstate.servoMinValue)/outRange;
		}else{
			if(pstate.Output>pstate.servoMaxValue) o = 0;
			else o = 100.0*(pstate.servoMaxValue - pstate.Output)/outRange;
		}
	lcd.PrintChar(12, 2,(char)HEAT_CHAR); lcd.PrintDouble(13, 2,o,2); 	lcd.PrintF(19, 2,F("%"));

//	lcd.PrintF(12, 3,F("d"));
//	float tempDerivate = pstate.myDInput/pstate.myDTimeMillis*(float)60000.0; //DTemp/DTime(minutes) °C/min
//	lcd.PrintDouble(13, 3,tempDerivate,2);lcd.PrintChar(19, 3,(char)DERIV_CHAR);

	lcd.PrintF(12, 3,F("/"));
	lcd.PrintDouble(13, 3,pstate.Ramp,0);lcd.PrintChar(19, 3,(char)DEGMIN_CHAR);

}

void LCDHelper::displayManual(PidState pstate){
	lcd.PrintChar(12, 0,(char)TEMPERATURE_CHAR); lcd.PrintDouble(13, 0,pstate.getTemperature(),2);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);
//	lcd.PrintChar(12, 1,(char)SETPOINT_CHAR);    lcd.PrintDouble(13, 1,pstate.Setpoint        ,2);lcd.PrintChar(19, 1,(char)DEGREE_CHAR);

	int o = 0;
	double outRange = pstate.servoMaxValue-pstate.servoMinValue;
	if(pstate.servoDirection==ServoDirectionCW){
		if(pstate.Output<pstate.servoMinValue) o = 0;
		else o = 100.0*(pstate.Output-pstate.servoMinValue)/outRange;
	}else{
		if(pstate.Output>pstate.servoMaxValue) o = 0;
		else o = 100.0*(pstate.servoMaxValue - pstate.Output)/outRange;
	}
	lcd.PrintChar(12, 2,(char)HEAT_CHAR); lcd.PrintDouble(13, 2,o,2); 	lcd.PrintF(19, 2,F("%"));

	//lcd.PrintDouble(14, 3,pstate.Output,1);
//	lcd.PrintF(12, 3,F("d"));
//	float tempDerivate = pstate.myDInput/pstate.myDTimeMillis*(float)60000.0; //DTemp/DTime(minutes) °C/min
//	lcd.PrintDouble(13, 3,tempDerivate,2);lcd.PrintChar(19, 3,(char)DERIV_CHAR);

}


void LCDHelper::displayAutoTune(PidState pstate){
	double tp = pstate.getTemperature();
	lcd.PrintChar(13, 0,(char)TEMPERATURE_CHAR); lcd.PrintDouble(14, 0,tp,1);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);

	int o = 0;
	double outRange = pstate.servoMaxValue-pstate.servoMinValue;
	if(pstate.servoDirection==ServoDirectionCW){
		o = 100.0*(pstate.Output-pstate.servoMinValue)/outRange;
	}else{
		o = 100.0*(pstate.servoMaxValue - pstate.Output)/outRange;
	}
	lcd.PrintDouble(14, 2,o,1);
	lcd.PrintF(19, 2,F("%"));
	lcd.PrintDouble(14, 3,pstate.Output,1);
}

void LCDHelper::displayConfigServo(PidState pstate){
	if(pstate.servoDirection==ServoDirectionCW){
		lcd.PrintF(11, 0,F("Dir CW"));
	}else if(pstate.servoDirection==ServoDirectionCCW){
		lcd.PrintF(11, 0,F("Dir CCW"));
	}
	lcd.PrintF(11, 1,F("Min "));lcd.PrintDouble(15, 1,pstate.servoMinValue,0);
	lcd.PrintF(11, 2,F("Max "));lcd.PrintDouble(15, 2,pstate.servoMaxValue,0);
}

//void LCDHelper::print(byte col, byte row, int val){
//	lcd.Print(col, row,val);
//}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.PrintF(col, row,ifsh);
}

