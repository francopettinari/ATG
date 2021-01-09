/*
 * PidState.cpp
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#include "Controller.h"

#include <EEPROM.h>
#include "ATG.h"

Controller::Controller() : pid(&temperature, &Output, &_dynamicSetpoint, _kp, _ki, _kd,P_ON_E, DIRECT){
	pid.SetSampleTime(pidSampleTimeSecs*1000);
	pid.SetMode(MANUAL);

	approacingStartMillis = 0;
	approacingEnd1Millis = 0;
	approacingEnd2Millis = 0;

	PrevSwitchOnOffMillis = 0;


}

long lastLog=0;


void Controller::_writeServo(int value){
	int degree = value;
	if(servoDirection==ServoDirectionCCW){
		degree = servoMaxValue-degree;
	}

	//Serial.print("Servo pos:");Serial.print(servoPosition);Serial.print("; degree:");Serial.println(+degree);
	if(value==0 || servoPosition!=degree){
		pServo->write(degree);
		delay(15);
		servoPosition = degree;
	}


}

/**
 * here only the temperature flow is handled and the state transition should not depend
 * on other statuses. a burner switch off/on should always avoid flame bumps!!
 *
 * minValueSwitchOff=false only during configuration. All other cases it is true
 */
void Controller::writeServoPosition(int degree, bool minValueSwitchOff,bool log){

	float now = millis();
//	if(log){
//		Serial.print(F("Current temp state: "));Serial.println(TempState);
//		Serial.print(F("TimeToLastSwitch:"));Serial.print(now-PrevSwitchOnOffMillis);
//		Serial.print(F(" Degree:"));Serial.print(degree);
//		Serial.print(F(" ServoPosition:"));Serial.print(servoPosition);
//		Serial.print(F(" ServoMinVal:"));Serial.println(servoMinValue);
//	}
	if(degree<servoMinValue || (minValueSwitchOff && degree==servoMinValue)) degree=0; //switch off on minVal
	switch(TempState){
		case TempStateUndefined:
			PrevSwitchOnOffMillis=0;//this will force next on/off transition to happen immediately without waits
			if(degree>=servoMinValue){
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			if(degree<servoMinValue){
				Serial.println(F("Forward to TempStateSwitchingOff"));
				TempState = TempStateSwitchingOff;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			break;
		case TempStateSwitchingOn:
			if (degree>=servoMinValue){
				//now switchin on
				if(now-PrevSwitchOnOffMillis>5000){
					_writeServo(servoMaxValue);
					delay(1000);
					_writeServo(degree);
					PrevSwitchOnOffMillis = now;
					TempState = TempStateOn;
					return;
				}else{
					//skip and wait
					println(F("Switch on: wait 5 seconds..."));
					return;
				}
			}
			Serial.println(F("Forward to TempStateSwitchingOff"));
			TempState = TempStateSwitchingOff;
			writeServoPosition(degree,minValueSwitchOff);
			break;
		case TempStateSwitchingOff:
			if (degree<servoMinValue){
				//now swtiching off
				if(now-PrevSwitchOnOffMillis>5000){
					_writeServo(servoMaxValue);
					delay(1000);
					_writeServo(0);
					PrevSwitchOnOffMillis = now;
					TempState = TempStateOff;
					return;
				}else{
					//skip and wait
					println(F("Switch off: wait 5 seconds..."));
					return;
				}
			}
			//here temperature is greater that min meaning that before switch off is complete the temp has raised over min again
			//so abort and change state to on
			Serial.println(F("Forward to TempStateSwitchingOn"));
			TempState = TempStateSwitchingOn;
			writeServoPosition(degree,minValueSwitchOff);
			break;
		case TempStateOn:
			if (degree<servoMinValue){
				//now swtiching off
				Serial.println(F("Forward to TempStateSwitchingOff"));
				TempState = TempStateSwitchingOff;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(degree);
			break;
		case TempStateOff:
			if (degree>=servoMinValue ){
				//now swtiching on
				Serial.println(F("Forward to TempStateSwitchingOn"));
				TempState = TempStateSwitchingOn;
				writeServoPosition(degree,minValueSwitchOff);
				return;
			}
			_writeServo(0);
			break;
	}
}

void Controller::SetFsmState(FsmState value){
	fsmState = value;
}

void Controller::startRamp(bool changeDynamicSetpoint){
	approacingStartMillis = millis();
	approacingEnd1Millis = 0;
	approacingEnd2Millis = 0;
	approacingStartTemp = temperature;
	approacingEnd1Temp=0;
	approacingEnd2Temp=0;
	if(changeDynamicSetpoint){
		_dynamicSetpoint=temperature; //ramp start from current temperature
//		if(_setpoint - temperature> 1){
//			_dynamicSetpoint+=1; //ramp start from current temperature
//		}

	}
	if(_dynamicSetpoint>_setpoint){
		_dynamicSetpoint=_setpoint;
	}
	lastDynSetpointCalcMillis=0;
}

//detect that heating has begun active if temperature has increased at least 0.1 deegres
void Controller::updateRamp(){
	//if(fsmState!=psRampimg) return;
	float now = millis();

	if (lastDynSetpointCalcMillis==0)lastDynSetpointCalcMillis=now;
	float deltaMillis = now-lastDynSetpointCalcMillis;
	if(deltaMillis>5000){

		float   currRamp = ramp;
		float   currStartMillis = approacingStartMillis;
		double  currStartTemp = approacingStartTemp;
		if(_setpoint-_dynamicSetpoint<2){
			if(approacingEnd1Millis==0){
				approacingEnd1Millis=now;
				approacingEnd1Temp = _dynamicSetpoint;
			}
			currStartMillis = approacingEnd1Millis;
			currStartTemp = approacingEnd1Temp;
			currRamp=currRamp*0.75;
		}
		if(_setpoint-_dynamicSetpoint<1){
			if(approacingEnd2Millis==0){
				approacingEnd2Millis=now;
				approacingEnd2Temp = _dynamicSetpoint;
			}
			currStartMillis = approacingEnd2Millis;
			currStartTemp = approacingEnd2Temp;
			currRamp=currRamp*0.75;
		}
		float elapsedSecs = (now-currStartMillis)/1000.0f;
		float elapsedDeltaTemp = currRamp/60.0f*elapsedSecs; //delta temp after elapsedSecs from start

		_dynamicSetpoint = currStartTemp + elapsedDeltaTemp; //calc new setpoint according to ramp
		if(_dynamicSetpoint>_setpoint){ //target is Setpoint: do not exeed it!
			_dynamicSetpoint = _setpoint;
		}

		lastDynSetpointCalcMillis = now;
		Serial.print(F("New dyn setpoint:"));Serial.println(_dynamicSetpoint,4);
		//UdpTracer->print(F("New dyn setpoint:"));UdpTracer->printFloat(DynamicSetpoint,4);UdpTracer->println();
	}
}

int Controller::getOutPerc(){
	int o = 0;
	double outRange = servoMaxValue-servoMinValue;
	if(Output<servoMinValue) o = 0;
	else o = 100.0*(Output-servoMinValue)/outRange;
	return o;
}

void Controller::setOutPerc(double val){
	Serial.print(F(">>>>>> OutputP: "));Serial.println(val);
	double outRange = servoMaxValue-servoMinValue;
	Output = servoMinValue + (val/100.0*outRange);
	Serial.print(F(">>>>>> Output: "));Serial.println(Output);
	writeServoPosition(Output,true,true);
}

void Controller::update(){

	double tempp = pProbe->readTemperature();

	tempp += tCorrection(tempp);

	setTemperature(tempp);
	if(!isAutoState(atg.state)){
		autoModeOn = false;
		Output=0;
		forcedOutput=0;
		writeServoPosition(Output,true,false);
	}

	//force to manual when:
	//- not in auto
	//- not configuring
	//- automode is disabled
	bool autoOrConfig = isAutoState(atg.state) || isConfigState(atg.state);
	if(autoModeOn==0 ||(pid.GetMode()!=AUTOMATIC && !autoOrConfig)){
		pid.SetMode(MANUAL);
	}

	if(!autoOrConfig) {
		Output=0;
		writeServoPosition(Output,true,false);
	}

	float now = millis();

	//fsm idle in case not automatic
	if(autoModeOn==0 || !isAutoState(atg.state)){
		fsmState = psIdle;
	}

	switch(atg.state){
		case svMain:
			writeServoPosition(0,true);
		break;
		case svRunAuto :
		case svRunAutoSetpoint0 :
		case svRunAutoRamp0 :
		case svRunAutoSetpoint1 :
		case svRunAutoRamp1 : {

			//if autoModeOn is disabled, then force the manual output and return
			if(autoModeOn==0){
				if(forcedOutput>0){
					pid.SetMode(MANUAL);
					setOutPerc(forcedOutput);
				}
				break;
			}
			
			//if here, pid should be in auto. if not, then let's force it!
			if(pid.GetMode()!=AUTOMATIC){
				Serial.println(F("PID switched to AUTOMATIC"));
				pid.SetTunings(_kp,_ki,_kd,P_ON_E);
				pid.SetControllerDirection(DIRECT);
				pid.SetOutputLimits(servoMinValue,servoMaxValue);
				pid.SetMode(AUTOMATIC);
			}

			//FIXME: move to startRamp/updateRamp ?
			if(ramp<=0){
				if(fsmState!=psSoak){
					SetFsmState(psSoak);
				}
				_dynamicSetpoint=_setpoint;
			}

			/*Handle fsm state, ramp and DynamicSetpoint*/
			switch(fsmState){
			case psIdle:
				SetFsmState(psRampimg);
				startRamp(true);
				break;
			case psRampimg:
				updateRamp(); //ramp until setpoint. it's the most graceful way to approach to setpoint

//				if(ramp<=0 && pid.GetKi()==_ki){
//					pid.SetTunings(_kp,_ki/3,_kd,P_ON_E);
//				} else if(ramp>0 && pid.GetKi()!=_ki){
//					pid.SetTunings(_kp,_ki,_kd,P_ON_E);
//				}
				if(temperature>_setpoint){
					SetFsmState(psSoak);
					_dynamicSetpoint=_setpoint;
				}

				break;
			case psSoak:
				//arrived to target temperature. now let's keep it stable
				//no more leave this status. no more ramp.
				//only setpoint change can provoke a ramp.


//				this status switch is going to provoke a jump in the output:
//				because status==psKeepTemp means that we arrived to setpoint and then themperature has
//				falled down. if now status==psSoakTemp and temperature<=_setpoint-1 the output can be something like 25%-50%
//				resetting state to psIdle will restart the ramping from 0%. this is wrong: now we only need much power, not less!
//				if(_ramp>0 && temperature<=_setpoint-1){
//					TcpComm->print(F("1 :"));TcpComm->print(temperature,2);TcpComm->println(fsmState);
//					SetFsmState(psIdle);//it's a way to restart
//					_dynamicSetpoint=_setpoint;
//				}
				break;
			}
			if(fsmState!=psIdle && temperature>_setpoint){
				pid.Reset();//avoid delay in switching off
			}
			pid.Compute();
			writeServoPosition(Output,true);

			break;
		}
		break;
		default:
		  break;
	}

}

//void Controller::saveSetPointTotoEEprom(){
//	EEPROM.begin(512);
//
//	int addr = 0;
//	addr+=1;
//	addr+=sizeof(PidStateValue);
//	addr+=sizeof(ServoDirection);
//	addr+=sizeof(int);
//	addr+=sizeof(int);
//	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(_setpoint);
//	EEPROM.put(addr, _setpoint);
//	addr+=sizeof(double);
//
//	EEPROM.commit();
////	EEPROM.end();
//
//}
//
//void Controller::loadFromEEProm(){
//	Serial.print(F("EEPROMReadSettings: Expected EEPROM_VER:"));Serial.println(eepromVer);
//
//	EEPROM.begin(512);
//	byte temp=0;
//	int addr = 0;
//	temp = EEPROM.get(addr, temp);
//	addr+=1;Serial.print(F("Size: "));Serial.println(addr);
//
////	if(temp!=eepromVer && temp!=eepromVer-1){
////		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
////		return;
////	}
//
//	state=EEPROM.get(addr, state);
//	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed state: "));Serial.println(state);
//	if(state<0 || state>100){
//		state=svMain;
//		Serial.println(F(">>>>>>>> SOMETHING WENT WRONG.... ABORT READING!"));
//		return;
//	}
//
//	servoDirection = EEPROM.get(addr, servoDirection);
//	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed servo dir: "));Serial.println(servoDirection);
//
//	servoMinValue = EEPROM.get(addr, servoMinValue);
//	addr+=sizeof(servoMinValue);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed servo min: "));Serial.println(servoMinValue);
//
//	servoMaxValue = EEPROM.get(addr, servoMaxValue);
//	addr+=sizeof(servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed servo max: "));Serial.println(servoMaxValue);
//
//	_setpoint = EEPROM.get(addr, _setpoint);
//	addr+=sizeof(_setpoint);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed setpoint: "));Serial.println(_setpoint);
//
//	_kp = EEPROM.get(addr, _kp);
//	addr+=sizeof(_kp);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed kp: "));Serial.println(_kp);
//
//	_ki = EEPROM.get(addr, _ki);
//	addr+=sizeof(_ki);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed ki: "));Serial.println(_ki);
//
//	_kd = EEPROM.get(addr, _kd);
//	addr+=sizeof(_kd);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed kd: "));Serial.println(_kd);
//
//	pidSampleTimeSecs = EEPROM.get(addr, pidSampleTimeSecs);
//	addr+=sizeof(pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
//	Serial.print(F("Readed Sample time secs: "));Serial.println(pidSampleTimeSecs);
//	if(pidSampleTimeSecs==NAN)pidSampleTimeSecs=5;
//
//	if(temp==4){
//		int dummy = 0;
//		dummy = EEPROM.get(addr, dummy);
//		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
//		Serial.print(F("Readed Timer secs: "));Serial.println(dummy);
//		if(dummy>60*24)dummy=0;
//		Serial.print(F("Calculated Timer mins: "));Serial.println(dummy);
//
//		dummy = EEPROM.get(addr, dummy);
//		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
//		Serial.print(F("Readed Timer state: "));Serial.println(dummy);
//		if(dummy<0||dummy>2)dummy=0;
//		Serial.print(F("Calculated Timer state: "));Serial.println(dummy);
//
//		dummy = EEPROM.get(addr, dummy);
//		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
//		Serial.print(F("Readed Timer elapsed secs: "));Serial.println(dummy);
//		if(dummy>60*60*24)dummy=0;
//		if(dummy<0)dummy=0;
//	}
//	if(temp>=5){
//		autoModeOn = EEPROM.get(addr, autoModeOn);
//		addr+=sizeof(autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
//		Serial.print(F("Readed autoModeOn: "));Serial.println(autoModeOn);
//
//		forcedOutput = EEPROM.get(addr, forcedOutput);
//		addr+=sizeof(forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
//		Serial.print(F("Readed forcedOutput: "));Serial.println(forcedOutput);
//	}
//	if(temp>=6){
//		temperatureCorrection = EEPROM.get(addr, temperatureCorrection);
//		addr+=sizeof(temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
//		Serial.print(F("Readed temperatureCorrection: "));Serial.println(temperatureCorrection*0.1);
//	}
//	EEPROM.commit();
//	EEPROM.end();
//}





//void Controller::savetoEEprom(){
////	ESP.wdtFeed();
//	EEPROM.begin(512);
//
//	int addr = 0;
//	EEPROM.put(addr, eepromVer);
//	addr+=1;Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. pid state: "));Serial.println(state);
//	EEPROM.put(addr, state);
//	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(servoDirection);
//	EEPROM.put(addr, servoDirection);
//	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(servoMinValue);
//	EEPROM.put(addr, servoMinValue);
//	addr+=sizeof(int);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(servoMaxValue);
//	EEPROM.put(addr, servoMaxValue);
//	addr+=sizeof(servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(_setpoint);
//	EEPROM.put(addr, _setpoint);
//	addr+=sizeof(_setpoint);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(_kp);
//	EEPROM.put(addr, _kp);
//	addr+=sizeof(_kp);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(_ki);
//	EEPROM.put(addr, _ki);
//	addr+=sizeof(_ki);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(_kd);
//	EEPROM.put(addr, _kd);
//	addr+=sizeof(_kd);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(pidSampleTimeSecs);
//	EEPROM.put(addr, pidSampleTimeSecs);
//	addr+=sizeof(pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. autoModeOn: "));Serial.println(autoModeOn);
//	EEPROM.put(addr, autoModeOn);
//	addr+=sizeof(autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. forcedOutput: "));Serial.println(forcedOutput);
//	EEPROM.put(addr, forcedOutput);
//	addr+=sizeof(forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
//
//	Serial.print(F("EEPROMWriteSettings. temperatureCorrection: "));Serial.println(temperatureCorrection*0.1);
//	EEPROM.put(addr, temperatureCorrection);
//	addr+=sizeof(temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
//
//	EEPROM.commit();
//	EEPROM.end();
//
////	ESP.wdtFeed();
//
//}
//
//void Controller::saveServoDirToEEprom(){
//	EEPROM.begin(512);
//
//	int addr = 0;
//	addr+=1;
//	addr+=sizeof(PidStateValue);
//	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(servoDirection);
//	EEPROM.put(addr, servoDirection);
//	addr+=sizeof(ServoDirection);
//	addr+=sizeof(int);
//	addr+=sizeof(int);
//	addr+=sizeof(double);
//
//	EEPROM.commit();
//	EEPROM.end();
//}
