/*
 * LCDDoubleBuffer.cpp
 *
 *  Created on: 14 nov 2018
 *      Author: franc
 */

#include "LCDBuffer.h"
#include <Arduino.h>
//#include <Wire.h>
//#define LCD_DEBUG = 1

LCDBuffer::LCDBuffer(LiquidCrystal_I2C& lcd,const int width, const int height):_lcd(lcd) {
	this->_width = width;
	this->_height = height;
	this->_size = width * height;
	this->_buffer = new char[this->_size];
}

LiquidCrystal_I2C* LCDBuffer::getLcd(){
	return &_lcd;
}

void LCDBuffer::clear() {
	memset(this->_buffer, 32, sizeof(char) * this->_size);
}

unsigned short crc16(char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}


void LCDBuffer::render() {
#ifdef LCD_DEBUG
	Serial.write(27);       // ESC command
	Serial.print("[2J");    // clear screen command
	Serial.write(27);
	Serial.print("[H");     // cursor to home command
	Serial.println(F("--------------------"));
	for(byte y=0;y<_height;y++){
		for(byte x=0;x<_width;x++){
			char c = _buffer[(y*_width)+x];
			Serial.print(c);
		}
		Serial.println();
	}
	Serial.println(F("--------------------"));
#endif
//	_lcd.setCursor(0,1);
//	_lcd.print("2 Hello, world!");
//	return;
	char line[_width+1];
	for (int y = 0; y < _height; y++) {
		memcpy(line,&_buffer[y*_width],_width);
		line[_width]=0;//NULL terminated
		this->_lcd.setCursor(0,y);
		this->_lcd.print(line);
	}
}

void LCDBuffer::PrintPChar(const int x, const int y,const char *s) {
	char *o = this->_buffer + (x + (y * this->_width));
	memcpy(o, s, strlen(s));
}

void LCDBuffer::PrintChar(const int x, const int y,char s) {
	char *o = this->_buffer + (x + (y * this->_width));
	memcpy(o, &s, 1);
}

void LCDBuffer::PrintString(const int x, const int y, String s){
	//Serial.print(F("LCDBuffer::Print("));Serial.print(x);Serial.print(F(","));Serial.print(y);Serial.println(F(",String)"));
	PrintPChar(x,y,s.c_str());
}

void LCDBuffer::PrintF(const int x, const int y,const __FlashStringHelper *ifsh) {
	//Serial.print(F("LCDBuffer::Print("));Serial.print(x);Serial.print(F(","));Serial.print(y);Serial.println(F(",ifsh)"));
    PGM_P p = reinterpret_cast<PGM_P>(ifsh);
    int ix=x;
    while (1) {
        uint8_t c = pgm_read_byte(p++);
        if (c == 0) break;
        PrintChar(ix,y,(char)c);
        ix++;
    }
}


extern void formatFloat(float f,char *buff, int digits);
//void formatFloat(float f,char *buff, int digits){
//	if(digits<=0){
//		dtostrf(f, 3, 0, buff);
//	}else{
//		dtostrf(f, 4+digits, digits, buff);
//	}
//}

float roundToDp( float f, int digits ) {
	float multiplier = powf( 10.0f, digits );
	f = roundf( f * multiplier ) / multiplier;
	return f;
}

void formatDouble(double d,char *buff, int decimals){
	d = roundToDp(d,decimals);
	if(decimals<=0){
		dtostrf(d, 3, 0, buff);
	}else{
		dtostrf(d, 4+decimals, decimals, buff);
	}
}

void formatDouble(double d,char *buff,int numbers, int decimals){
	d = roundToDp(d,decimals);
	if(decimals<=0){
		dtostrf(d, numbers, 0, buff);
		if(d<10)buff[0]='0';
	}else{
		dtostrf(d, numbers+1+decimals, decimals, buff);
	}
}

void LCDBuffer::pprintf(const int x, const int y, char* format,...){
	char line[_width+1];
	va_list argptr;
	va_start(argptr, format);
	sprintf(line,format,argptr);
	va_end(argptr);

	PrintPChar(x,y,line);
}

void LCDBuffer::PrintDoubleFD(const int x, const int y,double d,int fixed, int decimals){
	char line[_width+1];
	formatDouble(d,line,fixed,decimals);
	PrintPChar(x,y,line);
}

void LCDBuffer::PrintDoubleD(const int x, const int y,double d, int decimals){
	char line[_width+1];
	formatDouble(d,line,decimals);
	PrintPChar(x,y,line);
}

void LCDBuffer::PrintFloat(const int x, const int y,float f, int digits){
	char line[_width+1];
	f = roundToDp(f,digits);
	formatFloat(f,line,digits);
	PrintPChar(x,y,line);
}
