/*
 * PidState.h
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "Servo_ESP32.h"
#include <WString.h>




#include "Menu.h"
#include "pid/PID_v1.h"

class LCDHelper;
class MenuItem;

#include "LCDHelper.h"
#include "tempProbe.h"

enum PidStateValue { //FIXME: to be renamed to something common to two pids controllers
	svMain=0,
	svRunAuto=10, svRunAutoSetpoint=13,svRunAutoRamp=17,
	svConfig=20,svConfigController=21,
	svPidConfig=22,
	svPidKpiConfig=23,svPidKpdConfig=24,
	svPidKiiConfig=25, svPidKidConfig=26,svPidKicConfig=27,
	svPidKdiConfig=28,svPidKddConfig=29,
	svPidSampleTimeConfig=30,
	svServo_Config=40, svConfig_ServoDirection=41, svConfig_ServoMin=42,svConfig_ServoMax=43,
	svConfig_Probe=50,svConfig_ProbeCorrection=52,svConfig_ProbeAssign=54};
enum ServoDirection {ServoDirectionCW=0,ServoDirectionCCW=1};

enum FsmState {
	psIdle=0,      //initial state
	psWaitDelay=5, //wait initial response after head has been applied
	psRampimg=10,  //temperature ramping has started. dynSetpoint is calculated so that ramp is calculated
	               //ramp is activated if temperature is more thasn 3°C far from setpoint
	psSoak=30};//setpoint has been reached. now temperature must be kept stable
enum TemperatureTransitionState {TempStateUndefined=0,TempStateOff=10,TempStateOn=20,TempStateSwitchingOn=30,TempStateSwitchingOff=40};



class Controller {
private:

	TemperatureTransitionState TempState = TempStateUndefined;
//	EncoderPushButtonState decodeEncoderPushBtnState (boolean encoderPress);
	boolean IsEncoderPressed(boolean encoderPress);
	EncoderMovement decodeEncoderMoveDirection(int encoderPos);
	void _writeServo(int degree);

	bool isAutoState(int state);
//	byte eepromVer = 06;  // eeprom data tracking


	//last used KP=50, ki=0.15
	//good resolution(75, 0.4, 0)
	//last calculated 2020.06.09 KP=35, KI=KP/(deadTime*3.3)=25/(15*3.3)=0,7 (15 secs delay)
	PID pid;

//	Servo_ESP32 servo;


	uint8_t probeAddress;
	Servo_ESP32* pServo;
public:
	FsmState fsmState=psIdle;
	double _kp=50,_ki=0.3,_kd=0;
	double _setpoint = 25,_dynamicSetpoint=25,_ramp=1;
	TemperatureProbe probe;
	void initialize(int servoPin){
		pServo = new Servo_ESP32();
		pServo->attach(servoPin);
	}
	void setProbeAddress(uint8_t addr){probeAddress = addr;}
	uint8_t getProbeAddress(){return probeAddress;}

	float GetKp(){return _kp;}
	void SetKp(float val){
		_kp = val;
		pid.SetTunings(_kp, _ki, _kd);
	}
	float GetKi(){return _ki;}
	void SetKi(float val){
		_ki = val;
		pid.SetTunings(_kp, _ki, _kd);
	}
	float GetKd(){return _kd;}
	void SetKd(float val){
		_kd = val;
		pid.SetTunings(_kp, _ki, _kd);
	}
	double setpoint(){ return _setpoint; }
	void setSetpoint(double value){
		_setpoint=value;
		if(fsmState==psSoak){
			SetFsmState(psIdle);
		}
		updateRamp();
	}
	void incSetpoint(){
		_setpoint++; if(_setpoint>=120)_setpoint=120;
		if(fsmState==psRampimg){
			updateRamp();
		}
	}
	void decSetpoint(){
		_setpoint--; if(_setpoint<0)_setpoint=0;
		if(fsmState==psRampimg){
			updateRamp();
		}
	}

	double dynamicSetpoint(){ return _dynamicSetpoint; }
	void setDynamicSetpoint(double value){ _dynamicSetpoint=value; }

	double ramp(){ return _ramp; }
	void setRamp(double value){ _ramp=value; }
	void incRamp(){ _ramp++; }
	void decRamp(){ _ramp--; if(_ramp<0)_ramp=0; }


//	PidStateValue state = svMain;
	int autoModeOn = false; //true=> auto mode on, false auto mode paused
	int forcedOutput = 0; //0 means automatic, !=0 means forced to value
	LCDHelper        *lcdHelper=NULL;



	double           temperature = 0, lastTemperature=0/*,dTemperature*/;
	float pidSampleTimeSecs = 5;
	float lastManualLog = 0;
//	float lastUdpDataSent = 0;

	float approacingStartMillis,lastDynSetpointCalcMillis = 0;
	float approacingEnd1Millis=0,approacingEnd2Millis=0; //two intermediate points before end target
	float approacingStartTemp = 0;
	float approacingEnd1Temp = 0,approacingEnd2Temp = 0;
	double Output;
	float PrevSwitchOnOffMillis = 0;
	int servoPosition=0;

	void writeServoPosition(int degree, bool minValueSwitchOff,bool log=true);

//	int expectedReqId = 1; //expected request id

	ServoDirection servoDirection = ServoDirectionCW;
	int servoMinValue = 0; //degrees
	int servoMaxValue = 180; //degrees
	int temperatureCorrection = 0;//0.1 deg steps: 5=+0.5, -8=-0.8

	Controller();

//	PidStateValue getState(){ return state; }
//	void SetState(PidStateValue value, boolean save=true){
//		state = value;
//		if(save)savetoEEprom();
//		stateSelection=0;
//		setCurrentMenu(decodeCurrentMenu());
//	}

	int getOutPerc();
	void setOutPerc(double val);

	float getTemperature(){ return temperature; }
	void setTemperature(double value){ temperature = value; }



	void update();
	void SetFsmState(FsmState value);
//	void updatePidStatus();
	void startRamp();
	bool rampStarted();
	void updateRamp();
//	void sendStatus();

//	void loadFromEEProm();
//	void savetoEEprom();
//	void saveSetPointTotoEEprom();
//	void saveServoDirToEEprom();

};


#endif /* CONTROLLER_H_ */

