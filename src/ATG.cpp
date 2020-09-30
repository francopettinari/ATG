#include "ATG.H"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WString.h>
#include <Arduino.h>
#include "LCDHelper.h"
#include <MD_REncoder.h>
#include <WiFiUdp.h>

#include "TCPComm.h"
#include "Controller.h"
#include "gdb.h"
#include "LiquidCrystal-I2C/LiquidCrystal_I2C.h"
#include <EEPROM.h>

LiquidCrystal_I2C lcdx(0x27, 20,4); // @suppress("Abstract class cannot be instantiated")

LCDHelper lcdHelper(lcdx);
ATG atg;


long int rotValue=0;
uint8_t state=0;

portMUX_TYPE gpioMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrAB() {
   uint8_t s = state & 3;

  portENTER_CRITICAL_ISR(&gpioMux);
    if (digitalRead(ROTARY_PINA)) s |= 4;
    if (digitalRead(ROTARY_PINB)) s |= 8;
    switch (s) {
      case 0: case 5: case 10: case 15:
        break;
      case 1: case 7: case 8: case 14:
        rotValue++; break;
      case 2: case 4: case 11: case 13:
        rotValue--; break;
      case 3: case 12:
        rotValue += 2; break;
      default:
        rotValue -= 2; break;
    }
    state = (s >> 2);
   portEXIT_CRITICAL_ISR(&gpioMux);

}

long int swValue=0;
unsigned long lastSwRead = 0;
void IRAM_ATTR isrRotaryPress() {
	portENTER_CRITICAL_ISR(&gpioMux);
	unsigned long now = millis();
	if(now-lastSwRead>250){
		swValue++;
	}
	portEXIT_CRITICAL_ISR(&gpioMux);
}



const char *ssid = "ATG";
const char *password = "log4fape@ATG";

#define MAX_SRV_CLIENTS 3
WiFiServer server(8266); // @suppress("Abstract class cannot be instantiated")
WiFiClient serverClients[MAX_SRV_CLIENTS]; // @suppress("Abstract class cannot be instantiated")



#define SP 1
#define RP 2
#define ST 3
#define OUT 4
#define KP 5
#define KI 6
#define KD 7

char parOpenChar = '(';
char separator = ':';
char parClosedChar = ')';
void parseString(String s){
	Serial.println(s);
	if(s.startsWith(F("SET("), 0)){
		int parOpenIdx = s.indexOf(parOpenChar);
		int semicolIdx1 = s.indexOf(separator);
		int semicolIdx2 = s.indexOf(separator,semicolIdx1+1);
		int parClosedIdx = s.indexOf(parClosedChar);
		String property = s.substring(parOpenIdx+1,semicolIdx1);
		int iProperty   = property.toInt();
		String cReqId   = s.substring(semicolIdx1+1,semicolIdx2);
		String value    = s.substring(semicolIdx2+1,parClosedIdx);
		int iReqId = cReqId.toInt();
		if(iReqId<atg.expectedReqId){
			Serial.print(F("Received reqId: "));Serial.print(cReqId);Serial.print(F(". Expected: "));Serial.print(atg.expectedReqId);
			return;
		}
		if(iReqId>atg.expectedReqId){
			Serial.print(F("Received reqId: "));Serial.print(cReqId);Serial.print(F(". Expected: "));Serial.print(atg.expectedReqId);
			atg.expectedReqId=iReqId;
		}
		switch (iProperty){
			case SP:

				atg.getController(0)->setSetpoint(value.toFloat());
				break;
			case RP:
				atg.getController(0)->ramp = value.toFloat();
				break;
			case ST: {
				atg.getController(0)->setAutoMode(value.toInt());
					if(atg.getController(0)->autoModeOn){
						atg.pMainMenu->runMenu->switchMenu0->Caption=F("Auto");
					}else{
						atg.pMainMenu->runMenu->switchMenu0->Caption=F("Manual");
					}
			    }
				break;
			case OUT:
				atg.getController(0)->setForcedOutput(value.toInt());
				break;
			case KP:
				atg.getController(0)->SetKp(value.toFloat());
				break;
			case KI:
				atg.getController(0)->SetKi(value.toFloat());
				break;
			case KD:
				atg.getController(0)->SetKd(value.toFloat());
				break;
		}
		atg.savetoEEprom();
		atg.sendStatus();
		TcpComm->Log(F("RESP:"));
		TcpComm->Log(cReqId);
		TcpComm->Log(F("\n"));
	}

}

void handleClients(){
	uint8_t i;
	if (server.hasClient()){
		for(i = 0; i < MAX_SRV_CLIENTS; i++){
		  if (!serverClients[i] || !serverClients[i].connected()){
			if(serverClients[i]) serverClients[i].stop();
			serverClients[i] = server.available();
			continue;
		  }
		}
		//no free spot
		WiFiClient serverClient = server.available(); // @suppress("Abstract class cannot be instantiated")
		serverClient.stop();
	}
	for(i = 0; i < MAX_SRV_CLIENTS; i++){
		if (serverClients[i] && serverClients[i].connected()){
		  if(serverClients[i].available()){
			while(serverClients[i].available()) {
				String msg = serverClients[i].readStringUntil('\0');
				Serial.println(serverClients[i].read());
				parseString(msg);
			}
		  }
		}
	}
}

void sendClients(String s){
	for(int i = 0; i < MAX_SRV_CLIENTS; i++){
		if (serverClients[i] && serverClients[i].connected()){
		  serverClients[i].print(s);
//		  serverClients[i].flush();
		}
	}
}

MenuItem* ATG::decodeCurrentMenu(){
	MainMenu* pmm = pMainMenu;

	switch(state){
		case svMain :
			return pmm;
		case svRunAuto :
			return pmm->runMenu;
		case svRunAutoSetpoint0 :
			return pmm->runMenu->setpointMenu0;
		case svRunAutoSetpoint1 :
			return pmm->runMenu->setpointMenu1;
		case svRunAutoRamp0 :
			return pmm->runMenu->rampMenu0;
		case svRunAutoRamp1 :
			return pmm->runMenu->rampMenu1;
		case svConfig:
			return pmm->configMenu;
		case svConfigController:
			return pmm->configMenu->configControllerMenu;
		case svServo_Config:
			return pmm->configMenu->servoMenu;
		case svConfig_ServoDirection :
			return pmm->configMenu->servoMenu->dirMenu;
		case svConfig_ServoMin :
			return pmm->configMenu->servoMenu->minMenu;
		case svConfig_ServoMax:
			return pmm->configMenu->servoMenu->maxMenu;
		case svPidConfig:
			return pmm->configMenu->pidMenu;
		case svPidKpiConfig:
		case svPidKpdConfig:
			return pmm->configMenu->pidMenu->kpMenu;
		case svPidKiiConfig:
		case svPidKidConfig:
		case svPidKicConfig:
			return pmm->configMenu->pidMenu->kiMenu;
		case svPidKdiConfig:
		case svPidKddConfig:
			return pmm->configMenu->pidMenu->kdMenu;
		case svPidSampleTimeConfig:
			return pmm->configMenu->pidMenu->sampleTimeMenu;
		case svConfig_ProbeCorrection:
			return pmm->configMenu->correctionMenu;
	}
	return pmm;
}

ATG::ATG(){
	ctrl0.initialize(SERVO1_PIN,25);
	ctrl1.initialize(SERVO2_PIN,26);
	pMainMenu = new MainMenu();
	currentMenu = pMainMenu;
}

void ATG::setCurrentMenu(MenuItem *m){
	if(currentMenu == m) return;
	currentMenu = m;
	stateSelection=0;
	currMenuStart = 0;
}

void ATG::loadFromEEProm(){
	Serial.print(F("EEPROMReadSettings: Expected EEPROM_VER:"));Serial.println(eepromVer);

	EEPROM.begin(512);
	byte temp=0;
	int addr = 0;
	temp = EEPROM.get(addr, temp);
	addr+=1;Serial.print(F("Size: "));Serial.println(addr);

	if(temp!=eepromVer && temp!=eepromVer-1){
		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
		return;
	}

	state=EEPROM.get(addr, state);
	state=svMain; //I now need to alwys start from main
	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed state: "));Serial.println(state);
	if(state<0 || state>100){
		state=svMain;
		Serial.println(F(">>>>>>>> SOMETHING WENT WRONG.... ABORT READING!"));
		return;
	}

	ctrl0.servoDirection = EEPROM.get(addr, ctrl0.servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo dir: "));Serial.println(ctrl0.servoDirection);
	ctrl1.servoDirection = EEPROM.get(addr, ctrl1.servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo dir: "));Serial.println(ctrl1.servoDirection);

	ctrl0.servoMinValue = EEPROM.get(addr, ctrl0.servoMinValue);
	addr+=sizeof(ctrl0.servoMinValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo min: "));Serial.println(ctrl0.servoMinValue);
	ctrl1.servoMinValue = EEPROM.get(addr, ctrl1.servoMinValue);
	addr+=sizeof(ctrl1.servoMinValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo min: "));Serial.println(ctrl1.servoMinValue);

	ctrl0.servoMaxValue = EEPROM.get(addr, ctrl0.servoMaxValue);
	addr+=sizeof(ctrl0.servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo max: "));Serial.println(ctrl0.servoMaxValue);
	ctrl1.servoMaxValue = EEPROM.get(addr, ctrl1.servoMaxValue);
	addr+=sizeof(ctrl1.servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo max: "));Serial.println(ctrl1.servoMaxValue);

	ctrl0._setpoint = EEPROM.get(addr, ctrl0._setpoint);
	addr+=sizeof(ctrl0._setpoint);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed setpoint: "));Serial.println(ctrl0._setpoint);
	ctrl1._setpoint = EEPROM.get(addr, ctrl1._setpoint);
	addr+=sizeof(ctrl1._setpoint);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed setpoint: "));Serial.println(ctrl1._setpoint);

	ctrl0._kp = EEPROM.get(addr, ctrl0._kp);
	addr+=sizeof(ctrl0._kp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kp: "));Serial.println(ctrl0._kp);
	ctrl1._kp = EEPROM.get(addr, ctrl1._kp);
	addr+=sizeof(ctrl1._kp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kp: "));Serial.println(ctrl1._kp);

	ctrl0._ki = EEPROM.get(addr, ctrl0._ki);
	addr+=sizeof(ctrl0._ki);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed ki: "));Serial.println(ctrl0._ki);
	ctrl1._ki = EEPROM.get(addr, ctrl1._ki);
	addr+=sizeof(ctrl1._ki);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed ki: "));Serial.println(ctrl1._ki);

	ctrl0._kd = EEPROM.get(addr, ctrl0._kd);
	addr+=sizeof(ctrl0._kd);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kd: "));Serial.println(ctrl0._kd);
	ctrl1._kd = EEPROM.get(addr, ctrl1._kd);
	addr+=sizeof(ctrl1._kd);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kd: "));Serial.println(ctrl1._kd);

	ctrl0.pidSampleTimeSecs = EEPROM.get(addr, ctrl0.pidSampleTimeSecs);
	addr+=sizeof(ctrl0.pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed Sample time secs: "));Serial.println(ctrl0.pidSampleTimeSecs);
	if(ctrl0.pidSampleTimeSecs==NAN)ctrl0.pidSampleTimeSecs=5;
	ctrl1.pidSampleTimeSecs = EEPROM.get(addr, ctrl1.pidSampleTimeSecs);
	addr+=sizeof(ctrl1.pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed Sample time secs: "));Serial.println(ctrl1.pidSampleTimeSecs);
	if(ctrl1.pidSampleTimeSecs==NAN)ctrl1.pidSampleTimeSecs=5;

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
	ctrl0.autoModeOn = EEPROM.get(addr, ctrl0.autoModeOn);
	addr+=sizeof(ctrl0.autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed autoModeOn: "));Serial.println(ctrl0.autoModeOn);
	ctrl1.autoModeOn = EEPROM.get(addr, ctrl1.autoModeOn);
	addr+=sizeof(ctrl1.autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed autoModeOn: "));Serial.println(ctrl1.autoModeOn);

	ctrl0.forcedOutput = EEPROM.get(addr, ctrl0.forcedOutput);
	addr+=sizeof(ctrl0.forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed forcedOutput: "));Serial.println(ctrl0.forcedOutput);
	ctrl1.forcedOutput = EEPROM.get(addr, ctrl1.forcedOutput);
	addr+=sizeof(ctrl1.forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed forcedOutput: "));Serial.println(ctrl1.forcedOutput);

	ctrl0.temperatureCorrection = EEPROM.get(addr, ctrl0.temperatureCorrection);
	addr+=sizeof(ctrl0.temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed temperatureCorrection: "));Serial.println(ctrl0.temperatureCorrection*0.1);
	ctrl1.temperatureCorrection = EEPROM.get(addr, ctrl1.temperatureCorrection);
	addr+=sizeof(ctrl1.temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed temperatureCorrection: "));Serial.println(ctrl1.temperatureCorrection*0.1);

	ctrl0.ramp = EEPROM.get(addr, ctrl0.ramp);
	addr+=sizeof(ctrl0.ramp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed ramp0: "));Serial.println(ctrl0.ramp);
	ctrl1.ramp = EEPROM.get(addr, ctrl1.ramp);
	addr+=sizeof(ctrl1.ramp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed ramp1: "));Serial.println(ctrl1.ramp);

	EEPROM.commit();
	EEPROM.end();
}

void ATG::savetoEEprom(){
//	ESP.wdtFeed();
	EEPROM.begin(512);

	int addr = 0;
	EEPROM.put(addr, eepromVer);
	addr+=1;Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. pid state: "));Serial.println(state);
	EEPROM.put(addr, state);
	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);


	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(ctrl0.servoDirection);
	EEPROM.put(addr, ctrl0.servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(ctrl1.servoDirection);
	EEPROM.put(addr, ctrl1.servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(ctrl0.servoMinValue);
	EEPROM.put(addr, ctrl0.servoMinValue);
	addr+=sizeof(int);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(ctrl1.servoMinValue);
	EEPROM.put(addr, ctrl1.servoMinValue);
	addr+=sizeof(int);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(ctrl0.servoMaxValue);
	EEPROM.put(addr, ctrl0.servoMaxValue);
	addr+=sizeof(ctrl0.servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(ctrl1.servoMaxValue);
	EEPROM.put(addr, ctrl1.servoMaxValue);
	addr+=sizeof(ctrl1.servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(ctrl0._setpoint);
	EEPROM.put(addr, ctrl0._setpoint);
	addr+=sizeof(ctrl0._setpoint);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(ctrl1._setpoint);
	EEPROM.put(addr, ctrl1._setpoint);
	addr+=sizeof(ctrl1._setpoint);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(ctrl0._kp);
	EEPROM.put(addr, ctrl0._kp);
	addr+=sizeof(ctrl0._kp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(ctrl1._kp);
	EEPROM.put(addr, ctrl1._kp);
	addr+=sizeof(ctrl1._kp);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(ctrl0._ki);
	EEPROM.put(addr, ctrl0._ki);
	addr+=sizeof(ctrl0._ki);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(ctrl1._ki);
	EEPROM.put(addr, ctrl1._ki);
	addr+=sizeof(ctrl1._ki);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(ctrl0._kd);
	EEPROM.put(addr, ctrl0._kd);
	addr+=sizeof(ctrl0._kd);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(ctrl1._kd);
	EEPROM.put(addr, ctrl1._kd);
	addr+=sizeof(ctrl1._kd);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(ctrl0.pidSampleTimeSecs);
	EEPROM.put(addr, ctrl0.pidSampleTimeSecs);
	addr+=sizeof(ctrl0.pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(ctrl1.pidSampleTimeSecs);
	EEPROM.put(addr, ctrl1.pidSampleTimeSecs);
	addr+=sizeof(ctrl1.pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. autoModeOn: "));Serial.println(ctrl0.autoModeOn);
	EEPROM.put(addr, ctrl0.autoModeOn);
	addr+=sizeof(ctrl0.autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. autoModeOn: "));Serial.println(ctrl1.autoModeOn);
	EEPROM.put(addr, ctrl1.autoModeOn);
	addr+=sizeof(ctrl1.autoModeOn);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. forcedOutput: "));Serial.println(ctrl0.forcedOutput);
	EEPROM.put(addr, ctrl0.forcedOutput);
	addr+=sizeof(ctrl0.forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. forcedOutput: "));Serial.println(ctrl1.forcedOutput);
	EEPROM.put(addr, ctrl1.forcedOutput);
	addr+=sizeof(ctrl1.forcedOutput);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. temperatureCorrection: "));Serial.println(ctrl0.temperatureCorrection*0.1);
	EEPROM.put(addr, ctrl0.temperatureCorrection);
	addr+=sizeof(ctrl0.temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. temperatureCorrection: "));Serial.println(ctrl1.temperatureCorrection*0.1);
	EEPROM.put(addr, ctrl1.temperatureCorrection);
	addr+=sizeof(ctrl1.temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);

	Serial.print(F("EEPROMWriteSettings. ramp0: "));Serial.println(ctrl0.ramp);
	EEPROM.put(addr, ctrl0.ramp);
	addr+=sizeof(ctrl0.ramp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. ramp1: "));Serial.println(ctrl1.ramp);
	EEPROM.put(addr, ctrl1.ramp);
	addr+=sizeof(ctrl1.ramp);Serial.print(F("Size: "));Serial.println(addr);

	EEPROM.commit();
	EEPROM.end();

//	ESP.wdtFeed();

}

void ATG::sendStatus(){
//	Serial.print(DynamicSetpoint,4);Serial.print(F(" "));
//	Serial.print(Setpoint,4);Serial.print(F(" "));
//	Serial.println(temperature,4);Serial.print(F(" "));
//	Serial.println(Output);
	float now = millis();
	TcpComm->print(F("LOG:")      );TcpComm->print(now,4);
	TcpComm->print(F(";EXPRQID:") );TcpComm->print((float) expectedReqId,0);
	TcpComm->print(F(";KP:")   );TcpComm->print((float)getController(0)->GetKp(),4);
	TcpComm->print(F(";KI:")   );TcpComm->print((float)getController(0)->GetKi(),4);
	TcpComm->print(F(";KD:")   );TcpComm->print((float)getController(0)->GetKd(),4);
	TcpComm->print(F(";STATE:")   );TcpComm->print((float)getController(0)->autoModeOn,0);
	TcpComm->print(F(";FSM_STATE:")   );TcpComm->print(getController(0)->fsmState);
	TcpComm->print(F(";SETP:")    );TcpComm->print(getController(0)->_setpoint,4);
	TcpComm->print(F(";RAMP:")    );TcpComm->print(getController(0)->ramp,4);
	TcpComm->print(F(";DSETP:")   );TcpComm->print(getController(0)->_dynamicSetpoint,4);
	TcpComm->print(F(";TEMP:")    );TcpComm->print(getController(0)->temperature,4);
	TcpComm->print(F(";OUT:")     );TcpComm->print(getController(0)->Output,4);
	TcpComm->print(F(";OUTPERC:") );TcpComm->print((float)getController(0)->getOutPerc(),0);
	TcpComm->print(F(";SERVOPOS:"));TcpComm->println((float)getController(0)->servoPosition,0);

	lastUdpDataSent = now;
}





EncoderMovement ATG::decodeEncoderMoveDirection(int encoderPos){
	prevEncoderPos = currEncoderPos;
	currEncoderPos = encoderPos/4;
	if(currEncoderPos>prevEncoderPos){
		return EncMoveCCW;
	}else if(currEncoderPos<prevEncoderPos){
		return EncMoveCW;
	}
	return EncMoveNone;
}

void ATG::update(int encoderPos, EncoderSwStates encoderPress){
	if(!isMenuActive()) return;
	//encoder position
	EncoderMovement encMovement = decodeEncoderMoveDirection(encoderPos);

	//encoder push button
//	EncoderPushButtonState encoderPushButtonState = decodeEncoderPushBtnState(encoderPress);

	//Serial.print(F("State selection: "));Serial.println(stateSelection);
	//Serial.print(F("Encoder push: "));Serial.println(encoderPushButtonState);
//	ESP.wdtFeed();
	setCurrentMenu(decodeCurrentMenu());
	if(encMovement!=EncMoveNone || encoderPress!=EncoderPressNone){
		Serial.print(F("oooo "));Serial.print(EncoderSwStatesNames[encoderPress]);Serial.print(F(" oooo  "));Serial.println(stateSelection);
		Serial.print(F("Menu     :"));Serial.println(getCurrentMenu()->Caption);
		Serial.print(F("Menu  len:"));Serial.println(getCurrentMenu()->subMenuItemsLen());

		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
	}
	if(encMovement!=EncMoveNone){
		Serial.print(F(">>>>>> Enc mov <<<<<<  "));
		if(encMovement==EncMoveCW){
			Serial.println(F(" CW "));
		}else{
			Serial.println(F(" CCW "));
		}
		Serial.print(F("Selected idx:"));Serial.println(stateSelection);
		getCurrentMenu()->HandleEncoderMovement(encMovement);
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("New Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		Serial.print(F("New Selected idx:"));Serial.println(stateSelection);
	}
	if(encoderPress!=EncoderPressNone) {
		getCurrentMenu()->HandleEncoderPush(encoderPress);
	}
}

void loop() {

}

long int prevSwVal = 0;
unsigned long lastPress = 0,lastRotated=0;
unsigned long lastdblPress = 0;
long int prevRotValue=0;
EncoderSwStates LastSwState = EncoderPressNone;
void TaskInputLoop( void * pvParameters ){
	Serial.print("TaskInputLoop running on core "); Serial.println(xPortGetCoreID());

	lcdHelper.createCustomChars();
		//encoder setup
	pinMode(ROTARY_PINA, INPUT_PULLUP);
	pinMode(ROTARY_PINB, INPUT_PULLUP);
	pinMode(ROTARY_PINSW, INPUT_PULLUP);

	attachInterrupt(ROTARY_PINA, isrAB, CHANGE);
	attachInterrupt(ROTARY_PINB, isrAB, CHANGE);
	attachInterrupt(ROTARY_PINSW, isrRotaryPress, FALLING);

	for(;;){
	//Serial.print("Task1 running on core "); Serial.println(xPortGetCoreID());
		unsigned long now = millis();

		EncoderSwStates SwState = EncoderPressNone;

		//Serial.print(F("PRESS: "));Serial.print(!digitalRead(ROTARY_PINSW));Serial.print(F(" SW: "));Serial.print(swValue);Serial.print(F(" ROT: "));Serial.println(rotValue);

		if(swValue-prevSwVal>0){
			SwState = EncoderPressPressed;
			lastPress=now;
		} else if(!digitalRead(ROTARY_PINSW)){ //press
			if(now-lastPress>2000) {
				SwState = EncoderPressLongPressed;
				lastPress=now; //at next loop avoid reapeat longPress
			}
		}
		prevSwVal = swValue;

		bool rotated = rotValue!=prevRotValue;
		if(rotated) lastRotated = now;
		prevRotValue = rotValue;

		// menuActive = now-lastRotated<10000 ||  now-lastPress<10000;//10 secs

		atg.update(rotValue,SwState);
		lcdHelper.display();

		handleClients();

		//atg.setMenuActive(menuActive);


//		delay(1);
	}
}

void TaskControllerLoop( void * pvParameters ){
	Serial.print("TaskControllerLoop running on core "); Serial.println(xPortGetCoreID());

	atg.loadFromEEProm();
	Serial.println(F("Initialized from EEProm"));

	for(;;){
		atg.getController(0)->update();
		atg.getController(1)->update();
		delay(100);
	}
}

void setup() {


	Serial.begin(115200);

	WiFi.disconnect(true);
	WiFi.mode(WIFI_AP);
	WiFi.softAP(ssid, password);

	Serial.print(F("IP ADDRESS: "));Serial.println(WiFi.localIP());
	Serial.println(F("Initialized"));

	server.begin();
    server.setNoDelay(true);

	Serial.println(F("Initialized from EEProm"));

	enableCore0WDT();
//	enableCore1WDT();

	xTaskCreatePinnedToCore(
		TaskInputLoop,   /* Task function. */
		"TaskInputLoop",     /* name of task. */
		10000,       /* Stack size of task */
		NULL,        /* parameter of the task */
		1,           /* priority of the task */
		NULL,      /* Task handle to keep track of created task */
		1);          /* pin task to core 1 */

	delay(500);

	xTaskCreatePinnedToCore(
		TaskControllerLoop,   /* Task function. */
		"TaskControllerLoop",     /* name of task. */
		10000,       /* Stack size of task */
		NULL,        /* parameter of the task */
		1,           /* priority of the task */
		NULL,      /* Task handle to keep track of created task */
		0);          /* pin task to core 1 */

	delay(500);

}

