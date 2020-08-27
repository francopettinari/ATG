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
	void displayRun();
	void displayRun(int idx,Controller ctrl, bool showCtrlLabel);
	void displayAutoCtrlSel();
	void displayManual();
	void displayTimerValue();
	void displayConfig();
	void displayConfigServo();
	void displayConfigPid();
	void displayConfigProbe();
	void displayConfigProbeCorrection();
	void displayDefault();

//	int lastDisplayCount = 0;
//	double lastDisplayMillis = 0;
public:
	void createCustomChars();
	void display();
//	void print(byte col, byte row, int val);
	void print(byte col, byte row,  __FlashStringHelper *ifsh);

	LCDHelper(LiquidCrystal_I2C& lcd);
};

#endif /* LCDHELPER_H_ */
