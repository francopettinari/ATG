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
#include "PID_AutoTune_v0.h"

class LCDHelper;
class MenuItem;

#include "LCDHelper.h"

enum PidStateValue {
	svUndefiend=-1,svMain=0,
	svRun=9, svRunAuto=10,svRunAutoTune=11,svRunAutoTuneResult=12,svRunAutoSetpoint=13,svRunAutoTimer=14,svRunAutoTimerMinutes=15,svRunManual=16,svRunAutoRamp=17,
	svConfig=20,
	svPidConfig=22,
	svPidKpiConfig=23,svPidKpdConfig=24,
	svPidKiiConfig=25, svPidKidConfig=26,svPidKicConfig=27,
	svPidKdiConfig=28,svPidKddConfig=29,
	svPidSampleTimeConfig=30,
	svServo_Config=40, svConfig_ServoDirection=41, svConfig_ServoMin=42,svConfig_ServoMax=43};
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

	byte eepromVer = 04;  // eeprom data tracking

protected:
	PID pid;

public:
	Servo servo;
	MenuItem         *currentMenu=NULL;
	PID_ATune aTune;

	PidStateValue state = svMain;
	LCDHelper        *lcdHelper=NULL;

	MenuItem         *topMenu = NULL;
	int              stateSelection = 0, currMenuStart=0;
	int              currEncoderPos=0,prevEncoderPos=0;    // a counter for the rotary encoder dial
	double           temperature = 0, lastTemperature=0/*,dTemperature*/;
	float pidSampleTimeSecs = 5;
	float lastManualLog=0;
	int timerValueMins=0; //when >=0 means that a timer is set
	int timerState = 0;//0:inactive, 1:active, 2:paused, 3:elapsed
	float timerStartMillis = 0;
	int timerElapsedSecs = -1; //when >=0 means a timer count down is active
	double Setpoint = 25,DynamicSetpoint=25,pDynamicSetpoint=0,Ramp=1;
	float approacingStartMillis,lastDynSetpointCalcMillis = 0;
	float approacingStartTemp = 0;
	double autotuneSetPoint = 0;
	double Output;
	float PrevSwitchOnOffMillis = 0;
	int servoPosition=0;
	void _writeServo(int degree);
	void writeServoPosition(int degree, bool minValueSwitchOff);
	void writeServoPositionCW(int degree, bool minValueSwitchOff);
	void writeServoPositionCCW(int degree, bool minValueSwitchOff);
	boolean autoTune = false;
	double kp=2,ki=0.5,kd=2;
	double akp=2,aki=0.5,akd=2;




	double myPTerm;
	double myITerm;
	double myDTerm;
	double myDTimeMillis;
	double myDInput;
	double myError;
	double myOutputSum;

	PID_ATune getATune(){return aTune;}

	void SetAutotuneResult(double arkp,double arki, double arkd){
		akp = arkp;
		aki = arki;
		akd = arkd;
	}

	void ConfirmAutoTuneResult(){
		kp = akp;
		ki = aki;
		kd = akd;
	    pid.SetTunings(kp, ki, kd);
	    savetoEEprom();
	}

	void SetState(PidStateValue value, boolean save=true){
		state = value;
		if(save)savetoEEprom();
//		currentMenu = decodeCurrentMenu();
		stateSelection=0;
		setCurrentMenu(decodeCurrentMenu());
	}

	ServoDirection servoDirection = ServoDirectionCW;
	int servoMinValue = 0; //degrees
	int servoMaxValue = 180; //degrees

	PidState();
	void changeAutoTune(int value);

	MenuItem* decodeCurrentMenu();

	PidStateValue getState(){
		return state;
	}

	void setCurrentMenu(MenuItem *m){
		if(currentMenu == m) return;
		currentMenu = m;
		stateSelection=0;
		currMenuStart = 0;
	}

	float getTemperature(){
		return temperature;
	}

	double getATuneSetPoint(){
		return autotuneSetPoint;
	}

//	float RoundTo025(float Num){
//	  int tmp= (int)Num;
//	  return tmp+int((Num-tmp)*1000/225)*0.25;
//	}

	void setTemperature(double value){
//		value = RoundTo025(value);
		float now = millis();
//		if(lastTemperatureMillis==0){
//			lastTemperatureMillis=now;
//			lastTemperature = temperature;
//			dTemperature=0;
//		}else{
//
//			if(now-lastTemperatureMillis>=pidSampleTimeSecs*1000){
//				dTemperature = (value-lastTemperature)*1000.0/(now-lastTemperatureMillis);
//				dTemperature = dTemperature * 60.0;//degrees per minute
//				lastTemperatureMillis=now;
//				lastTemperature = value;
//			}
//		}
		if(temperature==value)return;
		temperature = value;
	}

	MenuItem  *getCurrentMenu(){
		return currentMenu;
	}

	void StartTimer(){
		timerState = 1;
		timerStartMillis = millis();
		if(timerElapsedSecs<0){
			timerElapsedSecs = 0;
		}
		savetoEEprom();
	}

	void PauseTimer(){
		timerState = 2;
		savetoEEprom();
	}

	void StopTimer(){
		timerState = 0;
		timerElapsedSecs = 0;
		savetoEEprom();
	}

	void TimerDone(){
		timerState = 3;
		timerElapsedSecs = 0;
		savetoEEprom();
	}

	void update(double temp,int encoderPos, boolean encoderPress);
	void SetFsmState(FsmState value);
	void updatePidStatus();
	void startRamp();
	bool waitRampStart();
	void updateRamp();
	void loadFromEEProm();
	void savetoEEprom();

	void savetoTimerElapsedSecsEEprom();
	void saveSetPointTotoEEprom();
	void saveServoDirToEEprom();
};


#endif /* PIDSTATE_H_ */

