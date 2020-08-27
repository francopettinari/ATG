#include "LCDHelper.h"

#include "ATG.h"

#include "Controller.h"

uint8_t tempCustomChar[] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B01010,
  B10001,
  B10001,
  B01110
};

uint8_t degreeCustomChar[8] = {
		B01100,
		B10010,
		B10010,
		B01100,
		B00000,
		B00000,
		B00000,
		B00000
};
uint8_t setpointCustomChar[8] = {
		B11100,
		B10000,
		B11100,
		B00111,
		B11101,
		B00111,
		B00100,
		B00100
};

uint8_t heatCustomChar[8] = {
		 B01110,
		  B10001,
		  B00000,
		  B01110,
		  B10001,
		  B00000,
		  B01110,
		  B10001
};

uint8_t derivateCustomChar[8] = {
		  B00001,
		  B01101,
		  B10011,
		  B10011,
		  B01100,
		  B00000,
		  B00000,
		  B00000
};

uint8_t degMinChar[8] = {
  B11000,
  B11000,
  B00000,
  B11111,
  B00000,
  B01010,
  B10101,
  B10101
};

uint8_t timerChar[8] = {
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

LCDHelper::LCDHelper(LiquidCrystal_I2C& l):lcd(l,20,4) {
//	l.print("2 Hello, world!");

}

void LCDHelper::createCustomChars(){
	this->lcd.getLcd()->begin();
	this->lcd.getLcd()->backlight();
	this->lcd.getLcd()->createChar(TEMPERATURE_CHAR, tempCustomChar);
	this->lcd.getLcd()->createChar(DEGREE_CHAR, degreeCustomChar);
	this->lcd.getLcd()->createChar(SETPOINT_CHAR, setpointCustomChar);
	this->lcd.getLcd()->createChar(HEAT_CHAR, heatCustomChar);
	this->lcd.getLcd()->createChar(DERIV_CHAR, derivateCustomChar);
	this->lcd.getLcd()->createChar(DEGMIN_CHAR, degMinChar);
	this->lcd.getLcd()->createChar(TIMER_CHAR, timerChar);
}
void LCDHelper::display(){
	this->lcd.getLcd()->home();
	this->lcd.clear();
	if(atg.isMenuActive()){
		lcd.PrintString(0,0,atg.getCurrentMenu()->Caption.c_str());
		std::vector<MenuItem *> subMenuItems = atg.getCurrentMenu()->subMenuItems;
		int max = subMenuItems.size()-1;
		int y=1;
		for(int idx = atg.currMenuStart;idx<=max;idx++){
			if(y>3) break;
			if(!subMenuItems[idx]->Visible) {
				//Serial.print(subMenuItems[idx]->Caption.c_str());Serial.println(F(" NOT VISIBLE"));
				continue;
			}
			//int y=idx+1-pstate.currMenuStart;
			if(idx == atg.stateSelection){
				lcd.PrintF(0,y,F(">"));
			}else{
				lcd.PrintF(0,y,F(" "));
			}

			if(idx<=max && subMenuItems[idx]->Selected){
				lcd.PrintF(0,y,F("o"));
			}
			if(idx<=max){
				lcd.PrintString(1,y,atg.getCurrentMenu()->subMenuItems[idx]->Caption.c_str());
			}
			y++;
		}
	}
	switch(atg.getState()) {
		case svRunAuto:

		case svRunAutoSetpoint:
			displayRun();
			break;
//		case svRunAutoCtrlSel:
//			displayAutoCtrlSel();
//			break;
		case svConfig:
			displayConfig();
			break;
		case svConfigController:
			displayConfig();
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
			displayConfigPid();
			break;
		case svServo_Config:
		case svConfig_ServoDirection:
		case svConfig_ServoMin:
		case svConfig_ServoMax:
			displayConfigServo();
			break;
		case svConfig_ProbeCorrection:
			displayConfigProbeCorrection();
			break;
		default:
			displayDefault();
			break;
	}
	lcd.render();
//	lcd.setCursor(0, 3);lcd.print(pstate.currEncoderPos);
//	lcd.setCursor(10, 3);lcd.print(pstate.stateSelection);
//	Serial.println(F(" "));
}

void LCDHelper::displayDefault(){
	float t0 = atg.getController(0)->getTemperature();
	float t1 = atg.getController(1)->getTemperature();

	int tPos = t0<100?14:13;
	if(t0<0)tPos--;
	/*lcd.PrintChar(tPos-1, 0,(char)TEMPERATURE_CHAR);*/lcd.PrintDoubleFD(tPos, 0,t0,2,1);lcd.PrintChar(19, 0,(char)DEGREE_CHAR);
	tPos = t1<100?14:13;
	if(t1<0)tPos--;
	/*lcd.PrintChar(tPos-1, 0,(char)TEMPERATURE_CHAR);*/lcd.PrintDoubleFD(tPos, 1,t1,2,1);lcd.PrintChar(19, 1,(char)DEGREE_CHAR);
}

void LCDHelper::displayConfig(){
	if(atg.getSelectedControllerIdx()==0){
		lcd.PrintF(10, 0,F("Ctrl 1"));
	}else{
		lcd.PrintF(10, 0,F("Ctrl 2"));
	}
}

void LCDHelper::displayConfigPid(){
	if(atg.getSelectedControllerIdx()==0){
		lcd.PrintF(10, 0,F("Ctrl 1"));
	}else{
		lcd.PrintF(10, 0,F("Ctrl 2"));
	}
	Controller ctrl = *atg.getSelectedController();
	lcd.PrintF(10, 0,F("Kp"));lcd.PrintDoubleFD(13, 0,ctrl.GetKp(),2,3);
	lcd.PrintF(10, 1,F("Ki"));lcd.PrintDoubleFD(13, 1,ctrl.GetKi(),2,3);
	lcd.PrintF(10, 2,F("Kd"));lcd.PrintDoubleFD(13, 2,ctrl.GetKd(),2,3);
	lcd.PrintF(10, 3,F("St"));lcd.PrintDoubleFD(16, 3,ctrl.pidSampleTimeSecs,2,0);
}

void LCDHelper::displayRun(int idx,Controller ctrl, bool showCtrlLabel){
	if(showCtrlLabel){
		if(atg.getSelectedControllerIdx()==0){
			lcd.PrintF(5, 0, F("Mash"));
		}else{
			lcd.PrintF(5, 0, F("Sparge"));
		}
	}
	int base =idx*10;
	int spPos = ctrl.setpoint()<100?base+7:base+6;
	int tPos = ctrl.getTemperature()<100?spPos-5:spPos-6;
	/*lcd.PrintChar(tPos-1, 0,(char)TEMPERATURE_CHAR);*/lcd.PrintDoubleFD(tPos, 0,ctrl.getTemperature(),2,1);lcd.PrintChar(spPos-1, 0,'/'); lcd.PrintDoubleFD(spPos, 0,ctrl.setpoint(),2,0);lcd.PrintChar(base+9, 0,(char)DEGREE_CHAR);
	int o = ctrl.getOutPerc();
	/*lcd.PrintChar(13, 1,(char)HEAT_CHAR);*/ lcd.PrintDoubleD(base+5, 1,o,0); 	lcd.PrintF(base+9, 1,F("%"));
	/*lcd.PrintF(13, 2,F("/"));*/ lcd.PrintDoubleD(base+5, 2,ctrl.ramp(),0);lcd.PrintChar(base+9, 2,(char)DEGMIN_CHAR);
}

void LCDHelper::displayRun(){
	if(atg.isMenuActive()){
		displayRun(1,*atg.getSelectedController(),true);
	}else{
		displayRun(0,*atg.getController(0),false);
		displayRun(1,*atg.getController(1),false);
	}
}

//void LCDHelper::displayAutoCtrlSel(){
//	if(atg.getSelectedControllerIdx()==0){
//		lcd.PrintF(10, 0,F("Ctrl 1"));
//	}else{
//		lcd.PrintF(10, 0,F("Ctrl 2"));
//	}
//}

void LCDHelper::displayConfigServo(){
	Controller ctrl = *atg.getSelectedController();
	if(ctrl.servoDirection==ServoDirectionCW){
		lcd.PrintF(11, 0,F("Dir CW"));
	}else if(ctrl.servoDirection==ServoDirectionCCW){
		lcd.PrintF(11, 0,F("Dir CCW"));
	}
	lcd.PrintF(11, 1,F("Min "));lcd.PrintDoubleFD(15, 1,ctrl.servoMinValue,3,0);
	lcd.PrintF(11, 2,F("Max "));lcd.PrintDoubleFD(15, 2,ctrl.servoMaxValue,3,0);
}

void LCDHelper::displayConfigProbeCorrection(){
	Controller ctrl = *atg.getSelectedController();
	lcd.PrintF(5, 0,F("Temp correction"));
	lcd.PrintDoubleFD(15, 2,ctrl.temperatureCorrection,3,0);
}

void LCDHelper::print(byte col, byte row,  __FlashStringHelper *ifsh){
	lcd.PrintF(col, row,ifsh);
}
