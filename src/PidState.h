/*
 * PidState.h
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#ifndef PIDSTATE_H_
#define PIDSTATE_H_

#include <WString.h>

enum EncoderMovement {EncMoveNone=-1,EncMoveCW=0,EncMoveCCW=1};
enum EncoderPushButtonState {EncoderPushButtonNone=0, EncoderPushButtonPressed=1};

#include "Menu.h"
#include "PID_v1.h"
#include <Servo.h>

class LCDHelper;
class MenuItem;

#include "LCDHelper.h"

enum PidStateValue {
	svUndefiend=-1,svMain=0,
	svRunAuto=10, svRunAutoSetpoint=13,svRunAutoRamp=17,
	svConfig=20,
	svPidConfig=22,
	svPidKpiConfig=23,svPidKpdConfig=24,
	svPidKiiConfig=25, svPidKidConfig=26,svPidKicConfig=27,
	svPidKdiConfig=28,svPidKddConfig=29,
	svPidSampleTimeConfig=30,
	svServo_Config=40, svConfig_ServoDirection=41, svConfig_ServoMin=42,svConfig_ServoMax=43,
	svConfig_Probe=50};
enum ServoDirection {ServoDirectionCW=0,ServoDirectionCCW=1};

enum FsmState {psIdle=0,psWaitDelay=5,psRampimg=10,psKeepTemp=30};
enum TemperatureTransitionState {TempStateUndefined=0,TempStateOff=10,TempStateOn=20,TempStateSwitchingOn=30,TempStateSwitchingOff=40};

class PidState {

	float lastPressMillis=0;
    float lastPressState = EncoderPushButtonNone;
	void updateLcd();
private:
	FsmState fsmState=psIdle;
	TemperatureTransitionState TempState = TempStateUndefined;
	EncoderPushButtonState decodeEncoderPushBtnState (boolean encoderPress);
	boolean IsEncoderPressed(boolean encoderPress);
	EncoderMovement decodeEncoderMoveDirection(int encoderPos);
	void _writeServo(int degree);

	bool isAutoState(int state);
	byte eepromVer = 06;  // eeprom data tracking
	PID pid;

public:
	Servo servo;
	MenuItem         *currentMenu=NULL;

	PidStateValue state = svMain;
	int autoModeOn = false; //true=> auto mode on, false auto mode paused
	int forcedOutput = 0; //0 means automatic, !=0 means forced to value
	LCDHelper        *lcdHelper=NULL;

	MenuItem         *topMenu = NULL;
	int              stateSelection = 0, currMenuStart=0;
	int              currEncoderPos=0,prevEncoderPos=0;    // a counter for the rotary encoder dial
	double           temperature = 0, lastTemperature=0/*,dTemperature*/;
	float pidSampleTimeSecs = 5;
	float lastManualLog = 0;
	float lastUdpDataSent = 0;
	double Setpoint = 25,DynamicSetpoint=25,Ramp=1;
	float approacingStartMillis,lastDynSetpointCalcMillis = 0;
	float approacingStartTemp = 0;
	double Output;
	float PrevSwitchOnOffMillis = 0;
	int servoPosition=0;

	void writeServoPosition(int degree, bool minValueSwitchOff,bool log=true);
	double kp=50,ki=0.5,kd=0;

	int expectedReqId = 1; //expected request id

	double myPTerm;
	double myITerm;
	double myDTerm;
	double myDTimeMillis;
	double myDInput;
	double myError;
	double myOutputSum;

	ServoDirection servoDirection = ServoDirectionCW;
	int servoMinValue = 0; //degrees
	int servoMaxValue = 180; //degrees
	int temperatureCorrection = 0;//0.1 deg steps: 5=+0.5, -8=-0.8

	PidState();

	PidStateValue getState(){ return state; }
	void SetState(PidStateValue value, boolean save=true){
		state = value;
		if(save)savetoEEprom();
		stateSelection=0;
		setCurrentMenu(decodeCurrentMenu());
	}

	int getOutPerc();
	void setOutPerc(double val);

	float getTemperature(){ return temperature; }
	void setTemperature(double value){ temperature = value; }

	MenuItem* decodeCurrentMenu();
	void setCurrentMenu(MenuItem *m);
	MenuItem  *getCurrentMenu(){ return currentMenu; }

	void update(double temp,int encoderPos, boolean encoderPress);
	void SetFsmState(FsmState value);
	void updatePidStatus();
	void startRamp();
	bool waitRampStart();
	void updateRamp();
	void sendStatus();

	void loadFromEEProm();
	void savetoEEprom();
	void saveSetPointTotoEEprom();
	void saveServoDirToEEprom();

};


#endif /* PIDSTATE_H_ */

