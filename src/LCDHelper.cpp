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

byte manCustomChar[] = {
  B01110,
  B10001,
  B01110,
  B00100,
  B11111,
  B00100,
  B00100,
  B11011
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
static const int MAN_CHAR = 4;
//static const int DERIV_CHAR = 4;
static const int DEGMIN_CHAR = 5;
static const int TIMER_CHAR = 6;

LCDHelper::LCDHelper(LiquidCrystal_I2C& l):lcd(l,20,4) {
//	l.print("2 Hello, world!");

}

void LCDHelper::begin(){
	createCustomChars();
}

void LCDHelper::createCustomChars(){
	this->lcd.getLcd()->begin();
	this->lcd.getLcd()->backlight();
	this->lcd.getLcd()->createChar(TEMPERATURE_CHAR, tempCustomChar);
	this->lcd.getLcd()->createChar(DEGREE_CHAR, degreeCustomChar);
	this->lcd.getLcd()->createChar(SETPOINT_CHAR, setpointCustomChar);
	this->lcd.getLcd()->createChar(HEAT_CHAR, heatCustomChar);
	this->lcd.getLcd()->createChar(MAN_CHAR, derivateCustomChar);
//	this->lcd.getLcd()->createChar(DERIV_CHAR, derivateCustomChar);
	this->lcd.getLcd()->createChar(DEGMIN_CHAR, degMinChar);
	this->lcd.getLcd()->createChar(TIMER_CHAR, timerChar);
}
void LCDHelper::display(){
	this->lcd.getLcd()->home();
	this->lcd.clear();

	if(atg.getState()!= svRunAuto &&
	   atg.getState()!= svRunAutoSetpoint0 &&
	   atg.getState()!= svRunAutoRamp0 &&
	   atg.getState()!= svRunAutoSetpoint1 &&
	   atg.getState()!=svRunAutoRamp1)
	{
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
		case svRunAutoSetpoint0:
		case svRunAutoRamp0:
		case svRunAutoSetpoint1:
		case svRunAutoRamp1:
			displayRun();
			break;
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

//void LCDHelper::displayRun(int idx,Controller ctrl, int menuPosIdx, bool selected){
//}
//
//void LCDHelper::displayRun(){
//	lcd.PrintF(1, 0, F("Up"));
//
//	lcd.PrintChar(9, 1,(char)TEMPERATURE_CHAR);
//	lcd.PrintChar(13, 1,(char)SETPOINT_CHAR);
//	lcd.PrintF   (17, 1,F("%"));
//	lcd.PrintChar(19, 1,(char)DEGMIN_CHAR);
//
//	Controller* pCtrl0 = atg.getController(0);
//	Controller* pCtrl1 = atg.getController(1);
//
//
//	bool menuSel = false;
//
//	if(atg.pMainMenu->runMenu->setpointMenu0->Selected){
//		menuSel = true;
//	}else if(atg.pMainMenu->runMenu->switchMenu0->Selected){
//		menuSel = true;
//	}else if(atg.pMainMenu->runMenu->rampMenu0->Selected){
//		menuSel = true;
//	}
//
//	const __FlashStringHelper *charPos0 = menuSel?F("o"):F(">");
//
//	lcd.PrintF(0, 2, F("M"));
//	lcd.PrintF(2, 2, pCtrl0->autoModeOn?F("a"):F("m"));
//	lcd.PrintDoubleFD( 5, 2,pCtrl0->getTemperature(),3,1);
//	if(atg.stateSelection==1){
//		lcd.PrintF(10, 2,charPos0);
//	}
//	lcd.PrintDoubleFD(11, 2,pCtrl0->setpoint(),2,0);
//	if(atg.stateSelection==2){
//		lcd.PrintF(15, 2,charPos0);
//	}
//	lcd.PrintDoubleFD(15, 2,pCtrl0->getOutPerc(),2,0);
//	if(atg.stateSelection==3){
//		lcd.PrintF(18, 2,charPos0);
//	}
//	lcd.PrintDoubleD (16, 2,pCtrl0->ramp(),0);
//
//
//	menuSel = false;
//	if(atg.pMainMenu->runMenu->setpointMenu1->Selected){
//		menuSel = true;
//	}else if(atg.pMainMenu->runMenu->switchMenu1->Selected){
//		menuSel = true;
//	}else if(atg.pMainMenu->runMenu->rampMenu1->Selected){
//		menuSel = true;
//	}
//	const __FlashStringHelper *charPos1 = menuSel?F("o"):F(">");
//	lcd.PrintF(0, 3, F("S"));
//	lcd.PrintF(2, 3, pCtrl0->autoModeOn?F("a"):F("m"));
//	lcd.PrintDoubleFD( 5, 3, pCtrl1->getTemperature(),3,1);
//	if(atg.stateSelection==4){
//		lcd.PrintF(10, 3,charPos1);
//	}
//	lcd.PrintDoubleFD(11, 3, pCtrl1->setpoint(),2,0);
//	if(atg.stateSelection==5){
//		lcd.PrintF(14, 3,charPos1);
//	}
//	lcd.PrintDoubleFD(15, 3, pCtrl1->getOutPerc(),2,0);
//	if(atg.stateSelection==6){
//		lcd.PrintF(18, 3,charPos1);
//	}
//	lcd.PrintDoubleD (16, 3, pCtrl1->ramp(),0);
//}

void LCDHelper::displayRun(int idx,Controller ctrl, int menuPosIdx, bool selected){
	int base =idx*10;
	int spPos = ctrl.setpoint()<100?base+7:base+6;
	int tPos = ctrl.getTemperature()<100?spPos-5:spPos-6;
	const __FlashStringHelper *charPos = selected?F("o"):F(">");
	//for(int i=1;i<4;i++)lcd.PrintF(base, i,F("|"));
	if(menuPosIdx==0){
		lcd.PrintF(base, 1,charPos);
	}
	lcd.PrintDoubleFD(tPos, 1,ctrl.getTemperature(),2,1);lcd.PrintChar(spPos-1, 1,'/'); lcd.PrintDoubleFD(spPos, 1,ctrl.setpoint(),2,0);lcd.PrintChar(base+9, 1,(char)DEGREE_CHAR);
	int o = ctrl.getOutPerc();
	if(menuPosIdx==1){
		lcd.PrintF(base, 2,charPos);
	}
	if(ctrl.autoModeOn){
		lcd.PrintF(base+2, 2,F("Aut"));
	}else{
		lcd.PrintF(base+2, 2,F("Man"));
	}
//    if(!ctrl.autoModeOn){
//    	switch(ctrl.TempState){
//    	case TempStateUndefined:
//    		lcd.PrintF(base+5, 2,F("?"));
//		break;
//    	case TempStateOff:
//    		lcd.PrintF(base+5, 2,F("-"));
//		break;
//    	case TempStateOn:
//    		lcd.PrintChar(base+5, 2,(char)HEAT_CHAR);
//		break;
//    	case TempStateSwitchingOn:
//    		lcd.PrintF(base+5, 2,F("^"));
//		break;
//    	case TempStateSwitchingOff:
//    		lcd.PrintF(base+5, 2,F("x"));
//		break;
//    	}
//    }
	lcd.PrintDoubleD(base+6, 2,o,0); 	lcd.PrintF(base+9, 2,F("%"));
	if(menuPosIdx==2){
		lcd.PrintF(base, 3,charPos);
	}
	if(ctrl.ramp<=0){
		lcd.PrintF(base+3, 3,F("No ramp"));
	}else{
		lcd.PrintDoubleD(base+4, 3,ctrl.ramp,1);lcd.PrintChar(base+9, 3,(char)DEGMIN_CHAR);
	}
}

void LCDHelper::displayRun(){
	if(atg.stateSelection==0){
		lcd.PrintF(0, 0,F(">"));
	}
	lcd.PrintF(1, 0, F("Up"));

	lcd.PrintF(6, 0, F("Mash"));
	bool menuSel = false;
	int menuPosIdx = -1;
	if (atg.stateSelection>=1 && atg.stateSelection<=3){
		menuPosIdx = atg.stateSelection-1;
	}

	if(atg.pMainMenu->runMenu->setpointMenu0->Selected){
		menuSel = true;
	}else if(atg.pMainMenu->runMenu->switchMenu0->Selected){
		menuSel = true;
	}else if(atg.pMainMenu->runMenu->rampMenu0->Selected){
		menuSel = true;
	}
	displayRun(0,*atg.getController(0),menuPosIdx,menuSel);

	lcd.PrintF(14, 0, F("Sparge"));
	menuSel = false;
	menuPosIdx = -1;
	if (atg.stateSelection>=4 && atg.stateSelection<=6){
		menuPosIdx = atg.stateSelection-4;
	}
	if(atg.pMainMenu->runMenu->setpointMenu1->Selected){
		menuSel = true;
	}else if(atg.pMainMenu->runMenu->switchMenu1->Selected){
		menuSel = true;
	}else if(atg.pMainMenu->runMenu->rampMenu1->Selected){
		menuSel = true;
	}
	displayRun(1,*atg.getController(1),menuPosIdx,menuSel);
}

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
