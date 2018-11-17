/*
 * LCDDoubleBuffer.h
 *
 *  Created on: 14 nov 2018
 *      Author: franc
 */

#ifndef LCDBUFFER_H_
#define LCDBUFFER_H_

#include "LiquidCrystal_I2C.h"

class LCDBuffer {
public:
  LCDBuffer(const int width, const int height);

  const int width() const { return this->_width; }
  const int height() const { return this->_height; }

  char* buffer() { return _buffer; }

  void clear();

  void render();

  void PrintChar(const int x, const int y,char s);
  void PrintPChar(const int x, const int y,const char *s);
  void PrintString(const int x, const int y, String s);
  void PrintF(const int x, const int y, const __FlashStringHelper *ifsh);
  void PrintDouble(const int x, const int y,double d, int digits=2);
private:
  int _width, _height, _size;
  char *_buffer;
  LiquidCrystal_I2C *_lcd;
};

#endif /* LCDBUFFER_H_ */
