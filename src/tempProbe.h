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
private:
	static const int firNOfSamples = 5;
	static const int windowSecs = 5;
	static const int firIdxMax = firNOfSamples*10;//max idx after which firIdx will be reset to start
	static const int readIntervalMs=1000*windowSecs/firNOfSamples;


	OneWire* onewire;
	DallasTemperature* sensors;
public:
	int sensorsDelms = 50000; //fake default val
		float lastTempReadMillis = 0;
		float firArray[firNOfSamples];
		float filteredValue;
		int firIdx; //current index of fir array. next value will be stored in fir[firIdx]


	TemperatureProbe();
	float readTemperature();
	virtual ~TemperatureProbe();
};

#endif /* TEMPPROBE_H_ */
