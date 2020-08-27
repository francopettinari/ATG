/*
 * fir.h
 *
 *  Created on: 15 feb 2020
 *      Author: franc
 */

#ifndef TEMPPROBE_H_
#define TEMPPROBE_H_

#include <Arduino.h>
#include <DallasTemperature.h>

class TemperatureProbe {
private:
	static const int firNOfSamples = 5;
	static const int windowSecs = 3;
	static const int firIdxMax = firNOfSamples*10;//max idx after which firIdx will be reset to start
	int readIntervalMs=1000*windowSecs/firNOfSamples;

	int sensorsDelms = 50000; //fake default val

	OneWire* onewire;
	DallasTemperature* sensors;


public:

	float lastTempReadMillis = 0;
	float firArray[firNOfSamples];
	float filteredValue;
	int firIdx; //current index of fir array. next value will be stored in fir[firIdx]

	float readTemperature();
	bool isReady(){ return firIdx>=firNOfSamples;}

	TemperatureProbe(int pin);

	virtual ~TemperatureProbe();
};

#endif /* TEMPPROBE_H_ */
