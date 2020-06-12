#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <WString.h>
#include "Arduino.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include <WiFiUdp.h>

#include "TCPComm.h"
#include "Controller.h"
#include "gdb.h"



Controller pidState;
LCDHelper lcdHelper;


const byte encoderCk     = D5; // rotary encoder Clock      // ESP32 GPIO4 pin 26
const byte pushButtonPin = D7; // rotary encoder pushbutton // ESP32 GPIO0 pin 25
const byte encoderDt     = D6; // rotary encoder Data       // ESP32 GPIO2 pin 24
boolean isEncoderPressed;
Encoder enc(encoderCk, encoderDt);

const char *ssid = "ATG";
const char *password = "log4fape@ATG";

void ICACHE_RAM_ATTR  handleEncPush() {
	isEncoderPressed = digitalRead(pushButtonPin) == 0;
}

//void readGasAlarm() {
//	float sensorValue = analogRead(A0);         // Read the Sensor Values from Analog Pin A0
//	float sensorVoltage = sensorValue/1024*5.0; // Calculate the Sensor Voltage
//	Serial.print("sensor voltage = ");          // Print the Message
//	Serial.print(sensorVoltage);                // Print the Values
//	Serial.println(" V");                       // Print the Message
//}

#define MAX_SRV_CLIENTS 3
WiFiServer server(8266);
WiFiClient serverClients[MAX_SRV_CLIENTS];



void setup() {
#ifdef DEBUG
	uart_div_modify(0,UART_CLK_FREQ / 115200);
	Serial.begin(115200);
	gdbstub_init();
#else
	Serial.begin(115200);
#endif
	WiFi.disconnect(true);
	WiFi.mode(WIFI_AP);
	WiFi.softAP(ssid, password);

	Serial.print(F("IP ADDRESS: "));Serial.println(WiFi.localIP());
	Serial.println(F("Initialized"));

	server.begin();
    server.setNoDelay(true);

	pinMode(pushButtonPin, INPUT_PULLUP);
	attachInterrupt(pushButtonPin, handleEncPush, CHANGE);
	pidState.loadFromEEProm();

	InitializeMenus();

	ESP.wdtDisable();
	ESP.wdtEnable(WDTO_8S);


	Serial.println(F("Initialized from EEProm"));

}

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
		if(iReqId<pidState.expectedReqId){
			Serial.print(F("Received reqId: "));Serial.print(cReqId);Serial.print(F(". Expected: "));Serial.print(pidState.expectedReqId);
			return;
		}
		if(iReqId>pidState.expectedReqId){
			Serial.print(F("Received reqId: "));Serial.print(cReqId);Serial.print(F(". Expected: "));Serial.print(pidState.expectedReqId);
			pidState.expectedReqId=iReqId;
		}
		switch (iProperty){
			case SP:

				pidState.setSetpoint(value.toFloat());
				break;
			case RP:
				pidState.setRamp(value.toFloat());
				break;
			case ST: {
					pidState.autoModeOn = value.toInt();
					MainMenu* pmm = (MainMenu*) pidState.topMenu;
					if(pidState.autoModeOn){
						pmm->runMenu->switchMenu->Caption=F("Auto");
					}else{
						pmm->runMenu->switchMenu->Caption=F("Manual");
					}
			    }
				break;
			case OUT:
				pidState.forcedOutput = value.toInt();
				break;
			case KP:
				pidState.SetKp(value.toFloat());
				break;
			case KI:
				pidState.SetKi(value.toFloat());
				break;
			case KD:
				pidState.SetKd(value.toFloat());
				break;
		}
		pidState.savetoEEprom();
		pidState.sendStatus();
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
		WiFiClient serverClient = server.available();
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

void RAMFUNC loop() {
	pidState.update(enc.read(),isEncoderPressed);

	ESP.wdtFeed();
	lcdHelper.display(pidState);

	handleClients();
}
