/*
 * PidState.h
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#ifndef PIDSTATE_H_
#define PIDSTATE_H_

#include <WString.h>
#include "Menu.h"
#include "PID_v1.h"
#include <Servo.h>

class LCDHelper;
#include "LCDHelper.h"

enum PidStateValue { None=0, RunAuto=10, RunHeatPause=12, Config_ServoDirection=20, Config_ServoMin=30,Config_ServoMax=40};
enum ServoDirection {ServoDirectionCW=0,ServoDirectionCCW=1};

enum EncoderMovement {EncMoveNone,EncMoveCW,EncMoveCCW};
enum EncoderPushButtonState {EncoderPushButtonNone, EncoderPushButtonPressed, EncoderPushButtonKeepedPressed};

class PidState {
	float lastPressMillis=0;

	void updateLcd();
private:
	MenuItem         *currentMenu=NULL;
	EncoderPushButtonState decodeEncoderPushBtnState (boolean encoderPress);
	EncoderMovement decodeEncoderMoveDirection(int encoderPos);

	byte eepromVer = 01;  // eeprom data tracking
protected:

//	double Input;

	PID pid;
	Servo servo;

	void setServoPosition(int degree);
public:
	LCDHelper        *lcdHelper=NULL;

	MenuItem         *topMenu = NULL;
	int              stateSelection = 0, currMenuStart=0;
	int              currEncoderPos=0,prevEncoderPos=0;    // a counter for the rotary encoder dial
	double           temperature = 0;
	double Setpoint = 25;
	double Output;
	PidStateValue     state = None;

	ServoDirection servoDirection = ServoDirectionCW;
	int servoMin = 0; //degrees
	int servoMax = 180; //degrees

	PidState();

	void setCurrentMenu(MenuItem *m){
		currentMenu = m;
		stateSelection=0;
		currMenuStart = 0;
		if(lcdHelper!=NULL){
			lcdHelper->display(*this);
		}
	}

	double getTemperature(){
		return temperature;
	}

	float RoundTo025(float Num){
	  int tmp= (int)Num;
	  return tmp+int((Num-tmp)*1000/225)*0.25;
	}

	void setTemperature(double value){
		value = RoundTo025(value);
		if(temperature==value)return ;
		temperature = value;
	}

	MenuItem  *getCurrentMenu(){return currentMenu;}

	void update(double temp,int encoderPos, boolean encoderPress);
	void loadFromEEProm();
	void savetoEEprom();
};


#endif /* PIDSTATE_H_ */
