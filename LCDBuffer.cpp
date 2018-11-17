/*
 * LCDDoubleBuffer.cpp
 *
 *  Created on: 14 nov 2018
 *      Author: franc
 */

#include "LCDBuffer.h"
#include <Wire.h>

LCDBuffer::LCDBuffer(const int width, const int height) {
	this->_width = width;
	this->_height = height;
	this->_size = width * height;

	_lcd = new LiquidCrystal_I2C(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

	this->_buffer = new char[this->_size];

	Wire.begin(D2,D1);
	_lcd->begin(20,4);
	_lcd->clear();
}

void LCDBuffer::clear() {
	memset(this->_buffer, ' ', sizeof(char) * this->_size);
}

void LCDBuffer::render() {
#ifdef LCD_DEBUG
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
//	this->_lcd->print(_buffer);
	//Serial.println(F("LCD render 1"));
	char line[_width+1];
	for (int y = 0; y < _height; y++) {
		memcpy(line,&_buffer[y*_width],_width);
		line[_width]=0;//NULL terminated
		this->_lcd->setCursor(0,y);
		this->_lcd->print(line);
	}
	//Serial.println(F("LCD render 10"));
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
    while (1) {
        uint8_t c = pgm_read_byte(p++);
        if (c == 0) break;
        PrintChar(x,y,(char)c);
    }
}

void formatFloat(float f,char *buff){
  if(f>=0 && f<10){
    buff[0]=' ';
    dtostrf(f, 2, 1, &buff[1]);
  }else{
    dtostrf(f, 2, 1, buff);
  }
}

void formatDouble(double d,char *buff, int digits){
  if(d>=0 && d<10){
    buff[0]=' ';
    dtostrf(d, digits, 1, &buff[1]);
  }else{
    dtostrf(d, digits, 1, buff);
  }
}

void LCDBuffer::PrintDouble(const int x, const int y,double d, int digits){
	char line[_width+1];
	formatDouble(d,line,digits);
	PrintPChar(x,y,line);
}
