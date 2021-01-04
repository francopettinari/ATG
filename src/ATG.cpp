#include "ATG.H"
#include <WString.h>
#include <Arduino.h>
#include "LCDHelper.h"

#include "Controller.h"
#include "LiquidCrystal-I2C/LiquidCrystal_I2C.h"
#include <EEPROM.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoOTA.h>

//#include "BluetoothSerial.h"

//BluetoothSerial SerialBT;

LiquidCrystal_I2C lcdx(0x27, 20,4); // @suppress("Abstract class cannot be instantiated")

ATG atg;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

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

long int swValue=-1;
unsigned long lastSwRead = 0;
void IRAM_ATTR isrRotaryPress() {
	portENTER_CRITICAL_ISR(&gpioMux);
	unsigned long now = millis();
	if(now-lastSwRead>250){
		swValue++;
	}
	portEXIT_CRITICAL_ISR(&gpioMux);
}

#define SP "SETPOINT"
#define RP "RAMP"
#define ST "STATE"
#define OUT "OUT"
#define KP "KP"
#define KI "KI"
#define KD "KD"
#define READ "READ"

//format: SET(<CMD>:<ctrl>:<value>)
//SET(SETPOINT:0:55)
//SET(STATE:0:1)
//SET(OUT:0:50)
char parOpenChar = '(';
char separator = ':';
char parClosedChar = ')';
void parseCommand(String s){
	Serial.println(s);
	if(s.startsWith(F("SET("), 0)){
		int parOpenIdx = s.indexOf(parOpenChar);
		int semicolIdx1 = s.indexOf(separator);
		int semicolIdx2 = s.indexOf(separator,semicolIdx1+1);
		int parClosedIdx = s.indexOf(parClosedChar);
		String command = s.substring(parOpenIdx+1,semicolIdx1);
		String ctrl   = s.substring(semicolIdx1+1,semicolIdx2);
		int iCtrl = ctrl.toInt();
		String value    = s.substring(semicolIdx2+1,parClosedIdx);
		Serial.println(command);
		Serial.println(ctrl);
		Serial.println(value);
		if(command==SP){
			atg.getController(iCtrl)->setSetpoint(value.toFloat());
		} else if (command==RP){
			atg.getController(iCtrl)->ramp = value.toFloat();
		} else if (command==ST){
			atg.getController(iCtrl)->setAutoMode(value.toInt());
			if(atg.getController(iCtrl)->autoModeOn){
				atg.pMainMenu->runMenu->switchMenu0->Caption=F("Auto");
			}else{
				atg.pMainMenu->runMenu->switchMenu0->Caption=F("Manual");
			}
		} else if (command==OUT){
			atg.getController(iCtrl)->setForcedOutput(value.toInt());
		} else if (command==KP){
			atg.getController(iCtrl)->SetKp(value.toFloat());
		} else if (command==KI){
			atg.getController(iCtrl)->SetKi(value.toFloat());
		} else if (command==KD){
			atg.getController(iCtrl)->SetKd(value.toFloat());
		}
		atg.savetoEEprom();
	}
	if(s.startsWith(F("READ("), 0)){
		String str = F("[{");
		str += String(atg.getController(0)->setpoint(),1);
		str += F(":");
		str += (atg.getController(0)->autoModeOn?F("1"):F("0"));
		str += F(":");
		str += String(atg.getController(0)->Output);
		str += F(":");
		str += String(atg.getController(0)->ramp,1);
		str += F("},{");
		str += String(atg.getController(1)->setpoint(),1);
		str += F(":");
		str += (atg.getController(1)->autoModeOn?F("1"):F("0"));
		str += F(":");
		str += String(atg.getController(1)->Output);
		str += F(":");
		str += String(atg.getController(1)->ramp,1);
		str += F("}]");
//		SerialBT.print(str);
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
		case svConfig_ProbeZeroCorrection:
			return pmm->configMenu->correctionMenu->zeroCorrectionMenu;
		case svConfig_ProbeBoilCorrection:
			return pmm->configMenu->correctionMenu->boilCorrectionMenu;
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


	ctrl0.zeroTemperatureCorrection = EEPROM.get(addr, ctrl0.zeroTemperatureCorrection);
	addr+=sizeof(ctrl0.zeroTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed zero temperatureCorrection: "));Serial.println(ctrl0.zeroTemperatureCorrection);
	ctrl1.zeroTemperatureCorrection = EEPROM.get(addr, ctrl1.zeroTemperatureCorrection);
	addr+=sizeof(ctrl1.zeroTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed zero temperatureCorrection: "));Serial.println(ctrl1.zeroTemperatureCorrection);

	ctrl0.boilTemperatureCorrection = EEPROM.get(addr, ctrl0.boilTemperatureCorrection);
	addr+=sizeof(ctrl0.boilTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed boil temperatureCorrection: "));Serial.println(ctrl0.boilTemperatureCorrection);
	ctrl1.boilTemperatureCorrection = EEPROM.get(addr, ctrl1.boilTemperatureCorrection);
	addr+=sizeof(ctrl1.boilTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("Readed boil temperatureCorrection: "));Serial.println(ctrl1.boilTemperatureCorrection);

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

	Serial.print(F("EEPROMWriteSettings. zero temperatureCorrection: "));Serial.println(ctrl0.zeroTemperatureCorrection);
	EEPROM.put(addr, ctrl0.zeroTemperatureCorrection);
	addr+=sizeof(ctrl0.zeroTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. zero temperatureCorrection: "));Serial.println(ctrl1.zeroTemperatureCorrection);
	EEPROM.put(addr, ctrl1.zeroTemperatureCorrection);
	addr+=sizeof(ctrl1.zeroTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);


	Serial.print(F("EEPROMWriteSettings. boil temperatureCorrection: "));Serial.println(ctrl0.boilTemperatureCorrection);
	EEPROM.put(addr, ctrl0.boilTemperatureCorrection);
	addr+=sizeof(ctrl0.boilTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);
	Serial.print(F("EEPROMWriteSettings. boil temperatureCorrection: "));Serial.println(ctrl1.boilTemperatureCorrection);
	EEPROM.put(addr, ctrl1.boilTemperatureCorrection);
	addr+=sizeof(ctrl1.boilTemperatureCorrection);Serial.print(F("Size: "));Serial.println(addr);


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
	//encoder position
	EncoderMovement encMovement = decodeEncoderMoveDirection(encoderPos);

	//Serial.print(F("State selection: "));Serial.println(stateSelection);
	//Serial.print(F("Encoder push: "));Serial.println(encoderPushButtonState);
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

LCDHelper lcdHelper(lcdx);
long int prevSwVal = 0;
unsigned long lastPress = 0,lastOTAPress=0,lastRotated=0, lastCtrlUpdate;
long int prevRotValue=0;
int lastCtrlUpdated = -1;
void loop() {

	unsigned long now = millis();

	if(now-lastCtrlUpdate>TemperatureProbe::readIntervalMsDiv2){
		lastCtrlUpdated++;
		if(lastCtrlUpdated>1)lastCtrlUpdated=0;
		if(lastCtrlUpdated==0){
			atg.getController(0)->update();
		}else if(lastCtrlUpdated==1){
			atg.getController(1)->update();
		}
		lastCtrlUpdate = millis();
	}

	lcdHelper.display();

	ArduinoOTA.handle();
}

const char* ssid = "ATG";
const char* password = "giuliaepippo";
const char* host = "ATG";

void startWiFiOTA(){
	WiFi.mode(WIFI_OFF);

	WiFi.mode(WIFI_AP);
	if(!WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0))){
		  Serial.println("AP Config Failed");
	  }
	WiFi.softAP(ssid);

	Serial.println(F(""));
	Serial.print(F("WiFI AP "));
	Serial.println(ssid);
	Serial.print(F("IP address: "));
	Serial.println(WiFi.softAPIP());

	if (!MDNS.begin(host)) { //http://esp32.local
		Serial.println(F("Error setting up MDNS responder!"));
		while (1) {
		  delay(1000);
		}
	  }

	ArduinoOTA.setPort(8266);
	ArduinoOTA.setHostname(host);
	ArduinoOTA
		.onStart([]() {
		  String type;
		  if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		  else // U_SPIFFS
			type = "filesystem";

		  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		  Serial.println("Start updating " + type);
		})
		.onEnd([]() {
		  Serial.println("\nEnd");
		})
		.onProgress([](unsigned int progress, unsigned int total) {
		  Serial.printf("Progress: %u%%\n\r", (progress / (total / 100)));
		})
		.onError([](ota_error_t error) {
		  Serial.printf("Error[%u]: ", error);
		  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		  else if (error == OTA_END_ERROR) Serial.println("End Failed");
		});

	  ArduinoOTA.begin();
}

void TaskInputLoop( void * pvParameters ){
	Serial.print("TaskInputLoop running on core "); Serial.println(xPortGetCoreID());
	//encoder setup
	pinMode(ROTARY_PINA, INPUT_PULLUP);
	pinMode(ROTARY_PINB, INPUT_PULLUP);
	pinMode(ROTARY_PINSW, INPUT_PULLUP);

	attachInterrupt(ROTARY_PINA, isrAB, CHANGE);
	attachInterrupt(ROTARY_PINB, isrAB, CHANGE);
	attachInterrupt(ROTARY_PINSW, isrRotaryPress, FALLING);

	char endCmd='#';

	for(;;){
		unsigned long now = millis();

		bool updated = false;

		now = millis();
		EncoderSwStates SwState = EncoderPressNone;
		//Serial.print(F("PRESS: "));Serial.print(!digitalRead(ROTARY_PINSW));Serial.print(F(" SW: "));Serial.print(swValue);Serial.print(F(" ROT: "));Serial.println(rotValue);
		if(swValue-prevSwVal>0){
			if(now-lastPress>300){
				SwState = EncoderPressPressed;
				lastPress=now;
				lastOTAPress=now;
				updated = true;
			}
		} else if(!digitalRead(ROTARY_PINSW)){ //press
			if(now-lastPress>1500) {
				SwState = EncoderPressLongPressed;
				lastPress=now; //at next loop avoid reapeat longPress
				updated = true;
			}
			if(now-lastOTAPress>10000) {
				atg.OTAActive = true;
				startWiFiOTA();
			}
		}
		prevSwVal = swValue;

		bool rotated = rotValue!=prevRotValue;
		if(rotated) {
			lastRotated = now;
			updated = true;
		}
		prevRotValue = rotValue;


		if(updated){
			atg.update(rotValue,SwState);
		}

//		while (SerialBT.available()) {
//		  parseCommand(SerialBT.readStringUntil(endCmd));
//		}



		delay(100);
	}
}


void setup() {
	Serial.begin(115200);

//	SerialBT.begin("ATG"); //Bluetooth device name

	lcdHelper.createCustomChars();

	atg.loadFromEEProm();
	Serial.println(F("Initialized from EEProm"));



	xTaskCreatePinnedToCore(
			TaskInputLoop,   /* Task function. */
			"TaskInputLoop",     /* name of task. */
			10000,       /* Stack size of task */
			NULL,        /* parameter of the task */
			1,           /* priority of the task */
			NULL,      /* Task handle to keep track of created task */
			0);          /* pin task to core 1 */
}

