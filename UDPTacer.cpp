/*
 * UDPTacer.cpp
 *
 *  Created on: 13 dic 2018
 *      Author: franc
 */

#include "UDPTacer.h"
#include <ESP8266WiFi.h>

UDPTracer* UdpTracer = new UDPTracer();
//IPAddress broadcastIp;
IPAddress broadcastIp (192,168,1,109);
//IPAddress broadcastIp(10,11,2,55);
unsigned int broadcastPort = 8267;      // local port to listen on

UDPTracer::UDPTracer() {
//	broadcastIp = ~WiFi.subnetMask() | WiFi.localIP();
	//broadcastIp = WiFi.localIP();
	//broadcastIp[3] = 255;
}

UDPTracer::~UDPTracer() {
}

void UDPTracer::LogChar(char s) {
	Udp.beginPacketMulticast(broadcastIp, broadcastPort, WiFi.localIP());
	Udp.write(s);
	if(s=='\n'){
		Udp.endPacket();
	}
}

void UDPTracer::Log(__FlashStringHelper *ifsh){
	Serial.print(ifsh);

	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	while (1) {
		uint8_t c = pgm_read_byte(p++);
		if (c == 0) break;
		LogChar((char)c);
	}
}

void UDPTracer::Log(String s){
	Udp.beginPacketMulticast(broadcastIp, broadcastPort, WiFi.localIP());
	Udp.write(s.c_str());
	if(s.endsWith(F("\n"))){
		Udp.endPacket();
	}
}

void UDPTracer::print(String s){
	Log(s);

}

void UDPTracer::println(String s){
	print(s);
	LogChar('\n');
}

void UDPTracer::printNumber(unsigned long n, uint8_t base) {
    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    // prevent crash if called with base == 1
    if(base < 2)
        base = 10;

    do {
        unsigned long m = n;
        n /= base;
        char c = m - base * n;
        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(n);

    return Log(str);
}

void UDPTracer::write(uint8_t c) {
	Udp.beginPacketMulticast(broadcastIp, broadcastPort, WiFi.localIP());
	Udp.write(c);
	if(c=='\n'){
		Udp.endPacket();
	}
}

void UDPTracer::print(unsigned long n, int base) {
    if(base == 0)
        return write(n);
    else
        return printNumber(n, base);
}

void formatFloat(float f,char *buff, int digits){
	if(digits<=0){
		dtostrf(f, 3, 0, buff);
	}else{
		dtostrf(f, 4+digits, digits, buff);
	}
}

void UDPTracer::printFloat(double number, uint8_t digits) {
	char line[16+1];
	formatFloat(number,line,digits);
	print(line);
}

void UDPTracer::print(long n, int base) {
    if(base == 0) {
        return write(n);
    } else if(base == 10) {
        if(n < 0) {
            LogChar('-');
            n = -n;
            printNumber(n, 10);
            return;
        }
        return printNumber(n, 10);
    } else {
        return printNumber(n, base);
    }
}

void UDPTracer::print(int n, int base) {
    print((long) n, base);
}

void UDPTracer::print(double n, int digits) {
    return printFloat(n, digits);
}

void UDPTracer::println(long l, int d){
	print(l,d);
	println();
}

void UDPTracer::println(double n, int digits){
	print(n,digits);
	println();
}

void UDPTracer::println(void) {
    return print(F("\r\n"));
}
