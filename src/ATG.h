#ifndef _Pid_H_
#define _Pid_H_

#include "Controller.h"
#include "LCDHelper.h"

#define SERVO1_PIN   14 //GPIO14
#define SERVO2_PIN   27 //GPIO27
#define ROTARY_PINA  15
#define ROTARY_PINB  4
#define ROTARY_PINSW 5

//extern Controller pidStates[2];
//extern Controller pidState;
extern LCDHelper lcdHelper;
//extern Servo_ESP32 servo;

class ATG {
private:
	int              currEncoderPos=0,prevEncoderPos=0;    // a counter for the rotary encoder dial

	byte eepromVer = 10;  // eeprom data tracking

	int  selectedCtrlIdx=0;
	Controller ctrl0;
	Controller ctrl1;
public:

	MenuItem   *currentMenu=NULL;
	float lastUdpDataSent = 0;
	int expectedReqId = 1; //expected request id
	int  nOfControllers=2, stateSelection = 0, currMenuStart=0;
	PidStateValue state = svMain;
	MainMenu*   pMainMenu;
	MenuItem* decodeCurrentMenu();
	void setCurrentMenu(MenuItem *m);
	MenuItem  *getCurrentMenu(){ return currentMenu; }
	EncoderMovement decodeEncoderMoveDirection(int encoderPos);
	ATG();
	Controller* getController(int idx){
		if(idx==0) return &ctrl0;
		return &ctrl1;
	}
	Controller* getSelectedController(){
		if(selectedCtrlIdx==0) return &ctrl0;
		return &ctrl1;
	}
	int getSelectedControllerIdx(){return selectedCtrlIdx;}
	void setSelectedController(int idx){
		Serial.print(F("===>"));Serial.println(idx);
		if(idx<0)idx=0;
		if(idx>1)idx=1;
		selectedCtrlIdx = idx;
		Serial.print(F("===>"));Serial.println(selectedCtrlIdx);
		if(selectedCtrlIdx<0){
			Serial.println(F("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxx"));
		}
	}
	void update(int encoderPos, EncoderSwStates encoderPress);
	void SetState(PidStateValue value, boolean save=true){
		state = value;
		if(save)savetoEEprom();
		stateSelection=0;
		setCurrentMenu(decodeCurrentMenu());
	}
	PidStateValue getState(){ return state; }
	void loadFromEEProm();
	void savetoEEprom();
};

extern ATG atg;

#endif /* _Pid_H_ */
