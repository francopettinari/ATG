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

class PidState;

class LCDHelper {
    LCDBuffer lcd;
	void displayRun(PidState pstate);
	void displayAutoTune(PidState pstate);
	void displayAutoTuneResult(PidState pstate);
	void displayConfigServo(PidState pstate);
	void displayConfigPid(PidState pstate);

//	int lastDisplayCount = 0;
//	double lastDisplayMillis = 0;
public:
	void display(PidState state);
//	void print(byte col, byte row, int val);
	void print(byte col, byte row,  __FlashStringHelper *ifsh);

	LCDHelper();
};

#endif /* LCDHELPER_H_ */
