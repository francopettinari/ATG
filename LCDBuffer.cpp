/*
 * LCDDoubleBuffer.cpp
 *
 *  Created on: 14 nov 2018
 *      Author: franc
 */

#include "LCDBuffer.h"
#include <Wire.h>
//#define LCD_DEBUG = 1

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

LiquidCrystal_I2C* LCDBuffer::getLcd(){
	return _lcd;
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
//	unsigned short currentCRC = crc16(_buffer, 16);
//	if(currentCRC==previousCRC) return;
//	previousCRC = currentCRC;
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
		this->_lcd->write(line,_width);
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
    int ix=x;
    while (1) {
        uint8_t c = pgm_read_byte(p++);
        if (c == 0) break;
        PrintChar(ix,y,(char)c);
        ix++;
    }
}

void formatFloat(float f,char *buff){
	dtostrf(f, 6, 2, buff);
//  if(f>=0 && f<10){
////    buff[0]=' ';
//    dtostrf(f, 6, 2, buff);
//  }else{
//    dtostrf(f, 6, 2, buff);
//  }
}

void formatDouble(double d,char *buff, int digits){
	dtostrf(d, 4+digits, digits, buff);
//  if(d>=0 && d<10){
////    buff[0]=' ';
//    dtostrf(d, 4+digits, digits, buff);
//  }else{
//    dtostrf(d, 4+digits, digits, buff);
//  }
}

void LCDBuffer::PrintDouble(const int x, const int y,double d, int digits){
	char line[_width+1];
	formatDouble(d,line,digits);
	PrintPChar(x,y,line);
}

void LCDBuffer::PrintFloat(const int x, const int y,float f){
	char line[_width+1];
	formatFloat(f,line);
	PrintPChar(x,y,line);
}
