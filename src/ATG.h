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

	byte eepromVer = 07;  // eeprom data tracking

	bool _menuActive = false;
public:

	bool isMenuActive(){return _menuActive;}
		void setMenuActive(bool val){
			_menuActive=val;
			Serial.print(F("Menu active: "));Serial.println(val);
		}
	Controller pidStates[2];
	MenuItem   *currentMenu=NULL;
	float lastUdpDataSent = 0;
	int expectedReqId = 1; //expected request id
	int  nOfControllers=2,selectedController=0, stateSelection = 0, currMenuStart=0;
	PidStateValue state = svMain;
	MainMenu   mainMenu;
	MenuItem* decodeCurrentMenu();
	void setCurrentMenu(MenuItem *m);
	MenuItem  *getCurrentMenu(){ return currentMenu; }
	EncoderMovement decodeEncoderMoveDirection(int encoderPos);
	ATG();

	Controller getSelectedController(){return pidStates[selectedController];}
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
	void sendStatus();
};

extern ATG atg;

#endif /* _Pid_H_ */
