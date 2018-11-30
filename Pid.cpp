// Do not remove the include below
#include <WString.h>
#include "Arduino.h"
#include "PID_v1.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"
#include "OneWire.h"
#include "probe.h"
#ifdef DEBUG
#include "gdb.h"
#endif

#include <DallasTemperature.h>

OneWire onewire(D3);
//probe probe(&onewire);
// Declare a DS18B20 Instance and assing the OneWire reference to it.
DallasTemperature sensors(&onewire);

PidState pidState;
LCDHelper lcdHelper;


const byte encoderCk     = D5; // rotary encoder Clock
const byte pushButtonPin = D7; // rotary encoder pushbutton
const byte encoderDt     = D6; // rotary encoder Data
boolean isEncoderPressed;
Encoder enc(encoderCk, encoderDt);

void handleEncPush() {
	isEncoderPressed = digitalRead(pushButtonPin) == 0;
}
#ifdef DEBUG
void RAMFUNC setup() {
	uart_div_modify(0, UART_CLK_FREQ / 115200);
	Serial.begin(115200);
	gdbstub_init();
#else
void setup() {
	Serial.begin(115200);
	Serial.println(F("Initialized"));
#endif

	sensors.begin();

	pinMode(pushButtonPin, INPUT_PULLUP);
	attachInterrupt(pushButtonPin, handleEncPush, CHANGE);
	InitializeMenus();

	ESP.wdtDisable();
	ESP.wdtEnable(WDTO_8S);

	pidState.loadFromEEProm();
	Serial.println(F("Initialized from EEProm"));
}

#ifdef DEBUG
void RAMFUNC loop() {
#else
	void loop() {
#endif
//	Serial.println(F("loop begin"));
	ESP.wdtFeed();
	sensors.requestTemperatures(); // Tell the DS18B20 to get make a measurement
//	probe::startConv();// start conversion for all sensors
//	if (probe::isReady()) {// update sensors when conversion complete
//		ESP.wdtFeed();
//		probe.update();
//		ESP.wdtFeed();
//	}
	ESP.wdtFeed();
	//pidState.update(probe.getTemp(),enc.read(),isEncoderPressed);
	pidState.update(sensors.getTempCByIndex(0),enc.read(),isEncoderPressed);

	ESP.wdtFeed();
	lcdHelper.display(pidState);
	ESP.wdtFeed();
//	Serial.println(F("loop end"));
}
