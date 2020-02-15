/*
 * fir.h
 *
 *  Created on: 15 feb 2020
 *      Author: franc
 */

#ifndef TEMPPROBE_H_
#define TEMPPROBE_H_

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class TemperatureProbe {
	int windowSecs = 5;
	int firNOfSamples;
	int firIdxMax;//max idx after which firIdx will be reset to start
	int readIntervalMs;

	int sensorsDelms = 50000; //fake default val

	bool tempReadRequested = false;
	float lastTempReadMillis = 0;
	float* firArray=NULL;
	float filteredValue = 0;
	int firIdx = 0; //current index of fir array. next value will be stored in fir[firIdx]

	OneWire* onewire;
	DallasTemperature* sensors;
public:
	TemperatureProbe(int secs, int nOfSamples);
	float readTemperature();
	virtual ~TemperatureProbe();
};

#endif /* TEMPPROBE_H_ */
