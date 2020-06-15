/*
 * LCDHelper.h
 *
 *  Created on: 29 ott 2018
 *      Author: franc
 */

#ifndef LCDHELPER_H_
#define LCDHELPER_H_

#include <WString.h>
#include "Arduino.h"
#include "LCDBuffer.h"

class Controller;

class LCDHelper {
    LCDBuffer lcd;
	void displayRun(Controller pstate);
	void displayManual(Controller pstate);
	void displayTimerValue(Controller pstate);
	void displayConfigServo(Controller pstate);
	void displayConfigPid(Controller pstate);
	void displayConfigProbe(Controller pstate);
	void displayDefault(Controller pstate);

//	int lastDisplayCount = 0;
//	double lastDisplayMillis = 0;
public:
	void display(Controller state);
//	void print(byte col, byte row, int val);
	void print(byte col, byte row,  __FlashStringHelper *ifsh);

	LCDHelper(LiquidCrystal_I2C& lcd);
};

#endif /* LCDHELPER_H_ */
