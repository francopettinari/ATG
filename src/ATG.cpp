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
bool encoderBtnPressed = false;

void IRAM_ATTR isrRotaryPressed() {
	portENTER_CRITICAL_ISR(&gpioMux);

	swValue++;
	encoderBtnPressed = true;
//	lastPressX = micros();
//	if(lastPressX-lastPress<500000){ //500ms
//		dblPress = true;
//		lastdblPress = lastPressX;
//	}
//	lastPress = lastPressX;
	portEXIT_CRITICAL_ISR(&gpioMux);
}

void IRAM_ATTR isrRotaryReleased() {
	portENTER_CRITICAL_ISR(&gpioMux);
	swValue++;
	encoderBtnPressed = false;
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

				atg.pidStates[0].setSetpoint(value.toFloat());
				break;
			case RP:
				atg.pidStates[0].setRamp(value.toFloat());
				break;
			case ST: {
				atg.pidStates[0].autoModeOn = value.toInt();
					if(atg.pidStates[0].autoModeOn){
						atg.mainMenu.runMenu->switchMenu->Caption=F("Auto");
					}else{
						atg.mainMenu.runMenu->switchMenu->Caption=F("Manual");
					}
			    }
				break;
			case OUT:
				atg.pidStates[0].forcedOutput = value.toInt();
				break;
			case KP:
				atg.pidStates[0].SetKp(value.toFloat());
				break;
			case KI:
				atg.pidStates[0].SetKi(value.toFloat());
				break;
			case KD:
				atg.pidStates[0].SetKd(value.toFloat());
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
	MainMenu* pmm = (MainMenu*) &mainMenu;

	switch(state){
		case svMain :
			return pmm;
		case svRunAuto :
			return pmm->runMenu;
		case svRunAutoSetpoint :
			return pmm->runMenu->setpointMenu;
		case svRunAutoRamp :
			return pmm->runMenu->rampMenu;
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
		case svConfig_Probe:
			return pmm->configMenu->probeMenu;
		case svConfig_ProbeCorrection:
			return pmm->configMenu->probeMenu->correctionMenu;
		case svConfig_ProbeAssign:
			return pmm->configMenu->probeMenu->assignmentMenu;
	}
	return pmm;
}

ATG::ATG(){
	currentMenu = &mainMenu;
	pidStates[0].initialize(SERVO1_PIN);
	pidStates[1].initialize(SERVO2_PIN);
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

//	if(temp!=eepromVer && temp!=eepromVer-1){
//		Serial.print(F(">>>>>>>> WRONG EPROM VERSION expected "));Serial.print(eepromVer);Serial.print(F("FOUND "));Serial.println(temp);
//		return;
//	}

	state=EEPROM.get(addr, state);
	addr+=sizeof(PidStateValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed state: "));Serial.println(state);
	if(state<0 || state>100){
		state=svMain;
		Serial.println(F(">>>>>>>> SOMETHING WENT WRONG.... ABORT READING!"));
		return;
	}

	atg.pidStates[0].servoDirection = EEPROM.get(addr, atg.pidStates[0].servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo dir: "));Serial.println(atg.pidStates[0].servoDirection);
	if(temp>=07){
		atg.pidStates[1].servoDirection = EEPROM.get(addr, atg.pidStates[1].servoDirection);
		addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed servo dir: "));Serial.println(atg.pidStates[1].servoDirection);
	}

	atg.pidStates[0].servoMinValue = EEPROM.get(addr, atg.pidStates[0].servoMinValue);
	addr+=sizeof(atg.pidStates[0].servoMinValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo min: "));Serial.println(atg.pidStates[0].servoMinValue);
	if(temp>=07){
		atg.pidStates[1].servoMinValue = EEPROM.get(addr, atg.pidStates[1].servoMinValue);
		addr+=sizeof(atg.pidStates[1].servoMinValue);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed servo min: "));Serial.println(atg.pidStates[1].servoMinValue);
	}

	atg.pidStates[0].servoMaxValue = EEPROM.get(addr, atg.pidStates[0].servoMaxValue);
	addr+=sizeof(atg.pidStates[0].servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed servo max: "));Serial.println(atg.pidStates[0].servoMaxValue);
	if(temp>=07){
		atg.pidStates[1].servoMaxValue = EEPROM.get(addr, atg.pidStates[1].servoMaxValue);
		addr+=sizeof(atg.pidStates[1].servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed servo max: "));Serial.println(atg.pidStates[1].servoMaxValue);
	}

	atg.pidStates[0]._setpoint = EEPROM.get(addr, atg.pidStates[0]._setpoint);
	addr+=sizeof(atg.pidStates[0]._setpoint);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed setpoint: "));Serial.println(atg.pidStates[0]._setpoint);
	if(temp>=07){
		atg.pidStates[1]._setpoint = EEPROM.get(addr, atg.pidStates[1]._setpoint);
		addr+=sizeof(atg.pidStates[1]._setpoint);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed setpoint: "));Serial.println(atg.pidStates[1]._setpoint);
	}

	atg.pidStates[0]._kp = EEPROM.get(addr, atg.pidStates[0]._kp);
	addr+=sizeof(atg.pidStates[0]._kp);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kp: "));Serial.println(atg.pidStates[0]._kp);
	if(temp>=07){
		atg.pidStates[1]._kp = EEPROM.get(addr, atg.pidStates[1]._kp);
		addr+=sizeof(atg.pidStates[1]._kp);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed kp: "));Serial.println(atg.pidStates[1]._kp);
	}

	atg.pidStates[0]._ki = EEPROM.get(addr, atg.pidStates[0]._ki);
	addr+=sizeof(atg.pidStates[0]._ki);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed ki: "));Serial.println(atg.pidStates[0]._ki);
	if(temp>=07){
		atg.pidStates[1]._ki = EEPROM.get(addr, atg.pidStates[1]._ki);
		addr+=sizeof(atg.pidStates[1]._ki);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed ki: "));Serial.println(atg.pidStates[1]._ki);
	}

	atg.pidStates[0]._kd = EEPROM.get(addr, atg.pidStates[0]._kd);
	addr+=sizeof(atg.pidStates[0]._kd);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed kd: "));Serial.println(atg.pidStates[0]._kd);
	if(temp>=07){
		atg.pidStates[1]._kd = EEPROM.get(addr, atg.pidStates[1]._kd);
		addr+=sizeof(atg.pidStates[1]._kd);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed kd: "));Serial.println(atg.pidStates[1]._kd);
	}

	atg.pidStates[0].pidSampleTimeSecs = EEPROM.get(addr, atg.pidStates[0].pidSampleTimeSecs);
	addr+=sizeof(atg.pidStates[0].pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed Sample time secs: "));Serial.println(atg.pidStates[0].pidSampleTimeSecs);
	if(atg.pidStates[0].pidSampleTimeSecs==NAN)atg.pidStates[0].pidSampleTimeSecs=5;
	if(temp>=07){
		atg.pidStates[1].pidSampleTimeSecs = EEPROM.get(addr, atg.pidStates[1].pidSampleTimeSecs);
		addr+=sizeof(atg.pidStates[1].pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Sample time secs: "));Serial.println(atg.pidStates[1].pidSampleTimeSecs);
		if(atg.pidStates[1].pidSampleTimeSecs==NAN)atg.pidStates[1].pidSampleTimeSecs=5;
	}

	if(temp==4){
		int dummy = 0;
		dummy = EEPROM.get(addr, dummy);
		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Timer secs: "));Serial.println(dummy);
		if(dummy>60*24)dummy=0;
		Serial.print(F("Calculated Timer mins: "));Serial.println(dummy);

		dummy = EEPROM.get(addr, dummy);
		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Timer state: "));Serial.println(dummy);
		if(dummy<0||dummy>2)dummy=0;
		Serial.print(F("Calculated Timer state: "));Serial.println(dummy);

		dummy = EEPROM.get(addr, dummy);
		addr+=sizeof(dummy);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed Timer elapsed secs: "));Serial.println(dummy);
		if(dummy>60*60*24)dummy=0;
		if(dummy<0)dummy=0;
	}
	if(temp>=5){
		atg.pidStates[0].autoModeOn = EEPROM.get(addr, atg.pidStates[0].autoModeOn);
		addr+=sizeof(atg.pidStates[0].autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed autoModeOn: "));Serial.println(atg.pidStates[0].autoModeOn);
		if(temp>=07){
			atg.pidStates[1].autoModeOn = EEPROM.get(addr, atg.pidStates[1].autoModeOn);
			addr+=sizeof(atg.pidStates[1].autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
			Serial.print(F("Readed autoModeOn: "));Serial.println(atg.pidStates[1].autoModeOn);
		}

		atg.pidStates[0].forcedOutput = EEPROM.get(addr, atg.pidStates[0].forcedOutput);
		addr+=sizeof(atg.pidStates[0].forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed forcedOutput: "));Serial.println(atg.pidStates[0].forcedOutput);
		if(temp>=07){
			atg.pidStates[1].forcedOutput = EEPROM.get(addr, atg.pidStates[1].forcedOutput);
			addr+=sizeof(atg.pidStates[1].forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
			Serial.print(F("Readed forcedOutput: "));Serial.println(atg.pidStates[1].forcedOutput);
		}

	}
	if(temp>=6){
		atg.pidStates[0].temperatureCorrection = EEPROM.get(addr, atg.pidStates[0].temperatureCorrection);
		addr+=sizeof(atg.pidStates[0].temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
		Serial.print(F("Readed temperatureCorrection: "));Serial.println(atg.pidStates[0].temperatureCorrection*0.1);
		if(temp>=07){
			atg.pidStates[1].temperatureCorrection = EEPROM.get(addr, atg.pidStates[1].temperatureCorrection);
			addr+=sizeof(atg.pidStates[1].temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
			Serial.print(F("Readed temperatureCorrection: "));Serial.println(atg.pidStates[1].temperatureCorrection*0.1);
		}
	}
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

	Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(atg.pidStates[0].servoDirection);
	EEPROM.put(addr, atg.pidStates[0].servoDirection);
	addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. servo dir: "));Serial.println(atg.pidStates[1].servoDirection);
		EEPROM.put(addr, atg.pidStates[1].servoDirection);
		addr+=sizeof(ServoDirection);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(atg.pidStates[0].servoMinValue);
	EEPROM.put(addr, atg.pidStates[0].servoMinValue);
	addr+=sizeof(int);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. servo min: "));Serial.println(atg.pidStates[1].servoMinValue);
		EEPROM.put(addr, atg.pidStates[1].servoMinValue);
		addr+=sizeof(int);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(atg.pidStates[0].servoMaxValue);
	EEPROM.put(addr, atg.pidStates[0].servoMaxValue);
	addr+=sizeof(atg.pidStates[0].servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. servo max: "));Serial.println(atg.pidStates[1].servoMaxValue);
		EEPROM.put(addr, atg.pidStates[1].servoMaxValue);
		addr+=sizeof(atg.pidStates[1].servoMaxValue);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(atg.pidStates[0]._setpoint);
	EEPROM.put(addr, atg.pidStates[0]._setpoint);
	addr+=sizeof(atg.pidStates[0]._setpoint);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. Setpoint: "));Serial.println(atg.pidStates[1]._setpoint);
		EEPROM.put(addr, atg.pidStates[1]._setpoint);
		addr+=sizeof(atg.pidStates[1]._setpoint);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(atg.pidStates[0]._kp);
	EEPROM.put(addr, atg.pidStates[0]._kp);
	addr+=sizeof(atg.pidStates[0]._kp);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. Kp: "));Serial.println(atg.pidStates[1]._kp);
		EEPROM.put(addr, atg.pidStates[1]._kp);
		addr+=sizeof(atg.pidStates[1]._kp);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(atg.pidStates[0]._ki);
	EEPROM.put(addr, atg.pidStates[0]._ki);
	addr+=sizeof(atg.pidStates[0]._ki);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. Ki: "));Serial.println(atg.pidStates[1]._ki);
		EEPROM.put(addr, atg.pidStates[1]._ki);
		addr+=sizeof(atg.pidStates[1]._ki);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(atg.pidStates[0]._kd);
	EEPROM.put(addr, atg.pidStates[0]._kd);
	addr+=sizeof(atg.pidStates[0]._kd);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. Kd: "));Serial.println(atg.pidStates[1]._kd);
		EEPROM.put(addr, pidStates[1]._kd);
		addr+=sizeof(pidStates[1]._kd);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(pidStates[0].pidSampleTimeSecs);
	EEPROM.put(addr, pidStates[0].pidSampleTimeSecs);
	addr+=sizeof(pidStates[0].pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. Sample time secs: "));Serial.println(pidStates[1].pidSampleTimeSecs);
		EEPROM.put(addr, pidStates[1].pidSampleTimeSecs);
		addr+=sizeof(pidStates[1].pidSampleTimeSecs);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. autoModeOn: "));Serial.println(pidStates[0].autoModeOn);
	EEPROM.put(addr, pidStates[0].autoModeOn);
	addr+=sizeof(pidStates[0].autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. autoModeOn: "));Serial.println(pidStates[1].autoModeOn);
		EEPROM.put(addr, pidStates[1].autoModeOn);
		addr+=sizeof(pidStates[1].autoModeOn);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. forcedOutput: "));Serial.println(pidStates[0].forcedOutput);
	EEPROM.put(addr, pidStates[0].forcedOutput);
	addr+=sizeof(pidStates[0].forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. forcedOutput: "));Serial.println(pidStates[1].forcedOutput);
		EEPROM.put(addr, pidStates[1].forcedOutput);
		addr+=sizeof(pidStates[1].forcedOutput);Serial.print(F("Size: "));Serial.println(addr);
	}

	Serial.print(F("EEPROMWriteSettings. temperatureCorrection: "));Serial.println(pidStates[0].temperatureCorrection*0.1);
	EEPROM.put(addr, pidStates[0].temperatureCorrection);
	addr+=sizeof(pidStates[0].temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	if(eepromVer>=07){
		Serial.print(F("EEPROMWriteSettings. temperatureCorrection: "));Serial.println(pidStates[1].temperatureCorrection*0.1);
		EEPROM.put(addr, pidStates[1].temperatureCorrection);
		addr+=sizeof(pidStates[1].temperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	}

	EEPROM.commit();
	EEPROM.end();

//	ESP.wdtFeed();

}

void ATG::sendStatus(){
	//Serial.print(DynamicSetpoint,4);Serial.print(F(" "));
	//Serial.print(Setpoint,4);Serial.print(F(" "));
	//Serial.println(temperature,4);Serial.print(F(" "));
	//Serial.println(Output);
	float now = millis();
	TcpComm->print(F("LOG:")      );TcpComm->print(now,4);
	TcpComm->print(F(";EXPRQID:") );TcpComm->print((float) expectedReqId,0);
	TcpComm->print(F(";KP:")   );TcpComm->print((float)pidStates[0].GetKp(),4);
	TcpComm->print(F(";KI:")   );TcpComm->print((float)pidStates[0].GetKi(),4);
	TcpComm->print(F(";KD:")   );TcpComm->print((float)pidStates[0].GetKd(),4);
	TcpComm->print(F(";STATE:")   );TcpComm->print((float)pidStates[0].autoModeOn,0);
	TcpComm->print(F(";FSM_STATE:")   );TcpComm->print(pidStates[0].fsmState);
	TcpComm->print(F(";SETP:")    );TcpComm->print(pidStates[0]._setpoint,4);
	TcpComm->print(F(";RAMP:")    );TcpComm->print(pidStates[0]._ramp,4);
	TcpComm->print(F(";DSETP:")   );TcpComm->print(pidStates[0]._dynamicSetpoint,4);
	TcpComm->print(F(";TEMP:")    );TcpComm->print(pidStates[0].temperature,4);
	TcpComm->print(F(";OUT:")     );TcpComm->print(pidStates[0].Output,4);
	TcpComm->print(F(";OUTPERC:") );TcpComm->print((float)pidStates[0].getOutPerc(),0);
	TcpComm->print(F(";SERVOPOS:"));TcpComm->println((float)pidStates[0].servoPosition,0);

	lastUdpDataSent = now;
}

int angle =0;
int angleStep = 5;

int angleMin =0;
int angleMax = 180;
int dir = 1;

void setup() {
	lcdHelper.createCustomChars();
	//encoder setup
	pinMode(ROTARY_PINA, INPUT_PULLUP);
	pinMode(ROTARY_PINB, INPUT_PULLUP);
	pinMode(ROTARY_PINSW, INPUT_PULLUP);

	attachInterrupt(ROTARY_PINA, isrAB, CHANGE);
	attachInterrupt(ROTARY_PINB, isrAB, CHANGE);
	attachInterrupt(ROTARY_PINSW, isrRotaryPressed, FALLING);
	attachInterrupt(ROTARY_PINSW, isrRotaryReleased, RISING);

	Serial.begin(115200);

	WiFi.disconnect(true);
	WiFi.mode(WIFI_AP);
	WiFi.softAP(ssid, password);

	Serial.print(F("IP ADDRESS: "));Serial.println(WiFi.localIP());
	Serial.println(F("Initialized"));

	server.begin();
    server.setNoDelay(true);

	//pinMode(pushButtonPin, INPUT_PULLUP);
	//attachInterrupt(pushButtonPin, handleEncPush, CHANGE);
	atg.loadFromEEProm();

//	ESP.wdtDisable();
//	ESP.wdtEnable(WDTO_8S);


	Serial.println(F("Initialized from EEProm"));

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

	if(encMovement!=EncMoveNone){
		Serial.print(F(">>>>>> Enc mov <<<<<<  "));
		if(encMovement==EncMoveCW){
			Serial.println(F(" CW "));
		}else{
			Serial.println(F(" CCW "));
		}
		Serial.print(F("Menu    :"));Serial.println(getCurrentMenu()->Caption);
		Serial.print(F("Menu len:"));Serial.println(getCurrentMenu()->subMenuItemsLen());
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		Serial.print(F("Selected :"));Serial.println(stateSelection);
		getCurrentMenu()->HandleEncoderMovement(encMovement);
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("New Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		Serial.print(F("New Selected :"));Serial.println(stateSelection);
	}
	if(encoderPress!=EncoderPressNone) {
		Serial.print(F(">>>>>> "));Serial.print(EncoderSwStatesNames[encoderPress]);Serial.print(F(" <<<<<<  "));Serial.println(stateSelection);
		Serial.print(F("Menu     :"));Serial.println(getCurrentMenu()->Caption);
		Serial.print(F("Menu  len:"));Serial.println(getCurrentMenu()->subMenuItemsLen());
		if(getCurrentMenu()->subMenuItemsLen()>0){
			Serial.print(F("Sel menu:"));Serial.println(getCurrentMenu()->subMenuItems[stateSelection]->Caption);
		}
		getCurrentMenu()->HandleEncoderPush(encoderPress);
	}
}

long int prevSwVal = 0;

unsigned long lastPress = 0, lastRotated=0;
unsigned long lastdblPress = 0;
long int prevRotValue=0;
void loop() {
	EncoderSwStates SwState = EncoderPressNone;
//	Serial.print(F("swValue: "));Serial.println(swValue);
	unsigned long now = millis();

	if(swValue-prevSwVal>0){
		if(now-lastPress<500){
			SwState = EncoderPressDblPressed;
			lastdblPress = now;
		}else{
			SwState = EncoderPressPressed;
			Serial.println(F("EncoderPressPressed"));
		}
		lastPress=now;
	}
	prevSwVal = swValue;

//	Serial.print(F("XX :"));Serial.print(now);Serial.print(F(" - "));Serial.println(lastPress);

	bool rotated = rotValue!=prevRotValue;
	if(rotated) lastRotated = now;
	prevRotValue = rotValue;

	bool menuActive = now-lastRotated<10000 ||  now-lastPress<10000;//10 secs


	bool longPressed = encoderBtnPressed && (now-lastPress)>1000;
	if(longPressed){
		SwState = EncoderPressLongPressed;
		Serial.println(F("EncoderPressLongPressed"));
	}

	if(SwState == EncoderPressDblPressed){
		Serial.println(F("EncoderPressDblPressed"));
	}
	atg.update(rotValue,SwState);
	atg.pidStates[0].update();
	//atg.pidStates[1].update();
//	ESP.wdtFeed();
	lcdHelper.display();

	handleClients();

	atg.setMenuActive(menuActive);
}

