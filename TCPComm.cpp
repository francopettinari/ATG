/*
 * TCPComm.cpp
 *
 *  Created on: 13 dic 2018
 *      Author: franc
 */

#include "TCPComm.h"

TCPComm* TcpComm = new TCPComm();

extern void sendClients(String s);

TCPComm::TCPComm() {
}

TCPComm::~TCPComm() {
}

String buff = "";
void TCPComm::LogChar(char s) {
    buff.concat(s);
    if(s=='\n'){
    	sendClients(buff);
    	buff = "";
	}
}

void TCPComm::Log(__FlashStringHelper *ifsh){
	Serial.print(ifsh);

	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	while (1) {
		uint8_t c = pgm_read_byte(p++);
		if (c == 0) break;
		LogChar((char)c);
	}
}

void TCPComm::Log(String s){
	buff.concat(s);
	if(s.endsWith(F("\n"))){
		sendClients(buff);
		buff = "";
	}
}

void TCPComm::print(String s){
	Log(s);

}

void TCPComm::println(String s){
	print(s);
	LogChar('\n');
}

void TCPComm::printNumber(unsigned long n, uint8_t base) {
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

void TCPComm::write(uint8_t c) {
	buff.concat((char)c);
	if(c == 0){
		sendClients(buff);
		buff = "";
	}
}

void TCPComm::print(unsigned long n, int base) {
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

void TCPComm::printFloat(double number, uint8_t digits) {
	char line[16+1];
	formatFloat(number,line,digits);
	print(line);
}

void TCPComm::print(long n, int base) {
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

void TCPComm::print(int n, int base) {
    print((long) n, base);
}

void TCPComm::print(double n, int digits) {
    return printFloat(n, digits);
}

void TCPComm::println(long l, int d){
	print(l,d);
	println();
}

void TCPComm::println(double n, int digits){
	print(n,digits);
	println();
}

void TCPComm::println(void) {
    return print(F("\r\n"));
}
