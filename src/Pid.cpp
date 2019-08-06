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
#include <WiFiUdp.h>

#include <DallasTemperature.h>
#include "TCPComm.h"

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

const char *ssid = "ATG";
const char *password = "log4fape@ATG";
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

#define MAX_SRV_CLIENTS 3
WiFiServer server(8266);
WiFiClient serverClients[MAX_SRV_CLIENTS];

void setup() {
	Serial.begin(115200);
	//Serial.begin(9600);  //due to serial XY graph

	WiFi.disconnect(true);
	WiFi.mode(WIFI_AP);
	WiFi.softAP(ssid, password);
	Serial.print(F("IP ADDRESS: "));Serial.println(WiFi.localIP());
	Serial.println(F("Initialized"));

	server.begin();
    server.setNoDelay(true);

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

//	Udp = UdpTracer->Udp;
//	Udp.begin(8266);
}

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
		if(property==F("SP")){
			pidState.Setpoint = value.toFloat();
		} else if(property==F("RP")){
			pidState.Ramp = value.toFloat();
		} else if(property==F("ST")){
			pidState.autoModeOn = value.toInt();
			MainMenu* pmm = (MainMenu*) pidState.topMenu;
			if(pidState.autoModeOn){
				pmm->runMenu->switchMenu->Caption=F("Auto");
			}else{
				pmm->runMenu->switchMenu->Caption=F("Manual");
			}
		} else if(property==F("OUT")){
			pidState.forcedOutput = value.toInt();

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
		  serverClients[i].flush();
		}
	}
}

//expected format: SET(<property>:<reqId>:<val>)
void parseIncomingUdp(){
	char incomingPacket[255];  // buffer for incoming packets
	int packetSize = Udp.parsePacket();
	if (packetSize) {
		// receive incoming UDP packets
		//Serial.printf(F(">>>>>>>>>>>>> Received %d bytes from %s, port %d\n"), packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
		int len = Udp.read(incomingPacket, 255);
		if (len > 0) {
		  incomingPacket[len] = 0;
		}
		String s = String(incomingPacket);
		parseString(s);
	}
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

//	parseIncomingUdp();
	handleClients();
}