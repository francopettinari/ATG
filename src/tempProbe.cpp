/*
 * fir.cpp
 *
 *  Created on: 15 feb 2020
 *      Author: franc
 */

#include "tempProbe.h"

#include <Arduino.h>
#include <DallasTemperature.h>

TemperatureProbe::TemperatureProbe(int secs, int nOfSamples) {
	windowSecs = secs;
	firNOfSamples = nOfSamples;

	firIdxMax = (firNOfSamples * 10);
	readIntervalMs = 1000*windowSecs/firNOfSamples;

	firArray = new float[firNOfSamples];

	onewire = new OneWire(D3);
	sensors = new DallasTemperature(onewire);

	sensors->setWaitForConversion(false);
	sensors->begin();
	sensors->setResolution(12);
	sensorsDelms = sensors->millisToWaitForConversion(12);
}

float TemperatureProbe::readTemperature(){
	unsigned long now = millis();
	if( (!tempReadRequested && now-lastTempReadMillis<readIntervalMs)) return filteredValue;

	if(!tempReadRequested){ //request temperature reading
		sensors->requestTemperatures(); // Tell the DS18B20 to get make a measurement
		tempReadRequested = true;
		lastTempReadMillis = now;
	}else if(sensors->isConversionComplete()){
		float temp = sensors->getTempCByIndex(0);
		tempReadRequested = false;
		firArray[firIdx%firNOfSamples] = temp;
		firIdx++;
		if(firIdx>firIdxMax)
			firIdx=firNOfSamples;
		float sum = 0;
		for(int i=0;i<firNOfSamples;i++)sum = sum+firArray[i];
		filteredValue = sum / firNOfSamples;
	}

	return filteredValue;
}

TemperatureProbe::~TemperatureProbe() {
	// TODO Auto-generated destructor stub
}

