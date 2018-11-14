/*
 * LCDHelper.h
 *
 *  Created on: 29 ott 2018
 *      Author: franc
 */

#ifndef LCDHELPER_H_
#define LCDHELPER_H_

#include <WString.h>
#include "LiquidCrystal_I2C.h"
#include "Arduino.h"


class PidState;

//

class LCDHelper {
	LiquidCrystal_I2C lcd;

	void Space (byte num);
	void Clear(byte row);
	void displayRun(PidState pstate);
	void displayConfigServo(PidState pstate);
public:
	void display(PidState state);
	void print(byte col, byte row, int val);
	void print(byte col, byte row,  __FlashStringHelper *ifsh);

	LCDHelper();
	virtual ~LCDHelper(){};
};

#endif /* LCDHELPER_H_ */
