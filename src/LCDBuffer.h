/*
 * LCDDoubleBuffer.h
 *
 *  Created on: 14 nov 2018
 *      Author: franc
 */

#ifndef LCDBUFFER_H_
#define LCDBUFFER_H_
#include "LiquidCrystalI2C/LiquidCrystal_I2C.h"

class LCDBuffer {
public:
  LCDBuffer(const int width, const int height);

  int width() { return this->_width; }
  int height() { return this->_height; }

  char* buffer() { return _buffer; }

  void clear();

  void render();

  void PrintChar(const int x, const int y,char s);
  void PrintPChar(const int x, const int y,const char *s);
  void PrintString(const int x, const int y, String s);
  void PrintF(const int x, const int y, const __FlashStringHelper *ifsh);
  void pprintf(const int x, const int y, char* format,...);
  void PrintDoubleFD(const int x, const int y,double d,int fixed, int digits);
  void PrintDoubleD(const int x, const int y,double d, int digits);
  void PrintFloat(const int x, const int y,float f,int digits);
  LiquidCrystal_I2C* getLcd();
private:
  int _width, _height, _size;
  char *_buffer;
  LiquidCrystal_I2C *_lcd;
  unsigned short previousCRC;
};

#endif /* LCDBUFFER_H_ */
