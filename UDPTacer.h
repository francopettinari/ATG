/*
 * UDPTacer.h
 *
 *  Created on: 13 dic 2018
 *      Author: franc
 */

#ifndef UDPTACER_H_
#define UDPTACER_H_

#include <WString.h>
#include "Arduino.h"
#include <WiFiUdp.h>



class UDPTracer {
public:
	WiFiUDP Udp;
	UDPTracer();
	virtual ~UDPTracer();

	void LogChar(char s) ;
	void Log(__FlashStringHelper *ifsh);
	void Log(String s);
	void write(uint8_t c);
	void print(String s);
	void println(String s);
	void printNumber(unsigned long n, uint8_t base);
    void print(int, int = DEC);
	void print(unsigned long n, int base = DEC);
    void print(long, int = DEC);
    void println(long l, int d = DEC);
	void printFloat(double number, uint8_t digits);
	void print(double n, int digits);
	void println(double n, int digits);
	void println(void);
};

extern UDPTracer*  UdpTracer;



#endif /* UDPTACER_H_ */