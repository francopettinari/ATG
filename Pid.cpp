// Do not remove the include below
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WString.h>
#include "Arduino.h"
#include "PID_v1.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"
#include "OneWire.h"
#include "UDPTacer.h"

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

void handleEncPush() {
	isEncoderPressed = digitalRead(pushButtonPin) == 0;
}

bool ConnectGiuliaWifi(){
	WiFi.disconnect(false);
	WiFi.mode(WIFI_STA);
	WiFi.begin("Giulia 2.4GHz", "giuliaepippo");
	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		//Serial.println("Connection Failed! Rebooting...");
		return false;
		//delay(5000);
		//ESP.restart();
	}
	return true;
}

bool ConnectLOWifi(){
	WiFi.disconnect(false);
	WiFi.mode(WIFI_STA);
	WiFi.begin("LOPonsaccoWifi", "log4wifiatlo");
	  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
		  return false;
		  //delay(5000);
		  //ESP.restart();
	  }
	  return true;
}

void OTASetup() {
  Serial.println("Booting");


  if(!ConnectGiuliaWifi() && !ConnectLOWifi()){
	  Serial.println("WiFi Connection Failed! OTA is disabled.");
	  return;
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("fapeOTA");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    Serial.println();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  UdpTracer->println("OTA Ready");
  UdpTracer->print("OTA IP address: ");
  Serial.println(WiFi.localIP());
}
void setup() {
	Serial.begin(115200);
	OTASetup();
	//Serial.begin(9600);  //due to serial XY graph
	Serial.println(F("Initialized"));

	sensors.setWaitForConversion(false);
	sensors.begin();
	sensors.setResolution(12);
	sensorsDelms = sensors.millisToWaitForConversion(12);

	pinMode(pushButtonPin, INPUT_PULLUP);
	attachInterrupt(pushButtonPin, handleEncPush, CHANGE);
	InitializeMenus();

	ESP.wdtDisable();
	ESP.wdtEnable(WDTO_8S);

	pidState.loadFromEEProm();
	Serial.println(F("Initialized from EEProm"));


}

bool tempReadRequested = false;
float lastTempReadMillis = 0;

void loop() {
	if(WiFi.status() == WL_CONNECTED){
		ArduinoOTA.handle();
	}

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
//	Serial.println(F("loop end"));
}
