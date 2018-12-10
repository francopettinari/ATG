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
	svRun=9, svRunAuto=10,svRunAutoTune=11,svRunAutoTuneResult=12,svRunAutoSetpoint=13,svRunAutoTimer=14,
	svConfig=20, svPidConfig=22, svPidKpiConfig=23,svPidKpdConfig=24, svPidKiiConfig=25, svPidKidConfig=26,svPidKicConfig=27, svPidKdiConfig=28,svPidKddConfig=29,
	svServo_Config=40, svConfig_ServoDirection=41, svConfig_ServoMin=42,svConfig_ServoMax=43};
enum ServoDirection {ServoDirectionCW=0,ServoDirectionCCW=1};


class PidState {
	float lastPressMillis=0;
    float lastPressState = EncoderPushButtonNone;
	void updateLcd();
private:


	EncoderPushButtonState decodeEncoderPushBtnState (boolean encoderPress);
	boolean IsEncoderPressed(boolean encoderPress);
	EncoderMovement decodeEncoderMoveDirection(int encoderPos);

	byte eepromVer = 02;  // eeprom data tracking

protected:
	PID pid;
    float pidSampleTime =10000;
    bool servoOFF = false;
public:
	Servo servo;
	MenuItem         *currentMenu=NULL;
	PID_ATune aTune;

	PidStateValue state = svMain;
	LCDHelper        *lcdHelper=NULL;

	MenuItem         *topMenu = NULL;
	int              stateSelection = 0, currMenuStart=0;
	int              currEncoderPos=0,prevEncoderPos=0;    // a counter for the rotary encoder dial
	double           temperature = 0, lastTemperature=0,dTemperature;
	float lastTemperatureMillis=0;
	double Setpoint = 25;
	double autotuneSetPoint = 0;
	double Output;
	void SetServoOff(bool value);
	bool IsServoOff();
	bool IsServoUnderFireOff();
	boolean autoTune = false;
	double kp=2,ki=0.5,kd=2;
	double akp=2,aki=0.5,akd=2;

	PID_ATune getATune(){return aTune;}
	void setServoPosition(int degree);

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

	double getTemperature(){
		return temperature;
	}

	double getATuneSetPoint(){
		return autotuneSetPoint;
	}

	float RoundTo025(float Num){
	  int tmp= (int)Num;
	  return tmp+int((Num-tmp)*1000/225)*0.25;
	}

	void setTemperature(double value){
//		value = RoundTo025(value);

		if(lastTemperatureMillis==0){
			lastTemperatureMillis=millis();
			lastTemperature = temperature;
			dTemperature=0;
		}else{
			float now = millis();
			if(now-lastTemperatureMillis>=pidSampleTime){
				dTemperature = (value-lastTemperature)*1000.0/(now-lastTemperatureMillis);
				dTemperature = dTemperature * 60.0;//degrees per minute
				lastTemperatureMillis=now;
				lastTemperature = value;
			}
		}
		if(temperature==value)return;
		temperature = value;
	}

	MenuItem  *getCurrentMenu(){
		return currentMenu;
	}

	void update(double temp,int encoderPos, boolean encoderPress);
	void loadFromEEProm();
	void savetoEEprom();

	void saveSetPointTotoEEprom();
	void saveServoDirToEEprom();
};


#endif /* PIDSTATE_H_ */
