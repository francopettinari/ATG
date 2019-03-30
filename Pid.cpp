// Do not remove the include below
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <WString.h>
#include "Arduino.h"
#include "PID_v1.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"
#include "OneWire.h"
#include "UDPTacer.h"
#include <WiFiUdp.h>

#include <DallasTemperature.h>

OneWire onewire(D3);
//probe probe(&onewire);
// Declare a DS18B20 Instance and assing the OneWire reference to it.
DallasTemperature sensors(&onewire);
int sensorsDelms = 50000; //fake default val

PidState pidState;
LCDHelper lcdHelper;


const byte encoderCk     = D5; // rotary encoder Clock
const byte pushButtonPin = D7; // rotary encoder pushbutton
const byte encoderDt     = D6; // rotary encoder Data
boolean isEncoderPressed;
Encoder enc(encoderCk, encoderDt);

const char *ssid = "atuttogas";
const char *password = "log4fape";
WiFiUDP Udp;

void handleEncPush() {
	isEncoderPressed = digitalRead(pushButtonPin) == 0;
}

void readGasAlarm() {
	float sensorValue = analogRead(A0);         // Read the Sensor Values from Analog Pin A0
	float sensorVoltage = sensorValue/1024*5.0; // Calculate the Sensor Voltage
	Serial.print("sensor voltage = ");          // Print the Message
	Serial.print(sensorVoltage);                // Print the Values
	Serial.println(" V");                       // Print the Message
}


void setup() {
	Serial.begin(115200);
	//Serial.begin(9600);  //due to serial XY graph

	WiFi.softAP(ssid, password);
	Serial.print(F("IP ADDRESS: "));Serial.println(WiFi.localIP());
	Serial.println(F("Initialized"));


	sensors.setWaitForConversion(false);
	sensors.begin();
	sensors.setResolution(12);
	sensorsDelms = sensors.millisToWaitForConversion(12);

	pinMode(pushButtonPin, INPUT_PULLUP);
	attachInterrupt(pushButtonPin, handleEncPush, CHANGE);
	pidState.loadFromEEProm();

	InitializeMenus();

	ESP.wdtDisable();
	ESP.wdtEnable(WDTO_8S);


	Serial.println(F("Initialized from EEProm"));


}

bool tempReadRequested = false;
float lastTempReadMillis = 0;

void loop() {
	unsigned long now = millis();
	if(!tempReadRequested || now-lastTempReadMillis>5000){
		sensors.requestTemperatures(); // Tell the DS18B20 to get make a measurement
		tempReadRequested = true;
		lastTempReadMillis = now;
		pidState.update(-200,enc.read(),isEncoderPressed);
	}else{
		if(sensors.isConversionComplete() && now-lastTempReadMillis>sensorsDelms){
			float temp = sensors.getTempCByIndex(0);
			pidState.update(temp,enc.read(),isEncoderPressed);
			tempReadRequested = false;
		}else{
			pidState.update(-200,enc.read(),isEncoderPressed);
		}
	}
	ESP.wdtFeed();
	lcdHelper.display(pidState);

}
