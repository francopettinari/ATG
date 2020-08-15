/*
 * fir.cpp
 *
 *  Created on: 15 feb 2020
 *      Author: franc
 */

#include "tempProbe.h"

#include <Arduino.h>
#include <DallasTemperature.h>
#include <gdb.h>

RAMFUNC TemperatureProbe::TemperatureProbe() {
	onewire = new OneWire(25);
	sensors = new DallasTemperature(onewire);

	sensors->setWaitForConversion(false);
	sensors->begin();
	sensors->setResolution(12);
	sensorsDelms = sensors->millisToWaitForConversion(12);

	firIdx = 0;
	filteredValue = 0;


}

float TemperatureProbe::readTemperatureByIndex(int index){
	return readTemperature(index,0);
}

float TemperatureProbe::readTemperatureByAddress(const uint8_t* deviceAddress){
	return readTemperature(-1,deviceAddress);
}

bool tempReadRequested = false;
float TemperatureProbe::readTemperature(int index, const uint8_t* deviceAddress){
	unsigned long now = millis();

	//too early
	if( (!tempReadRequested && now-lastTempReadMillis<readIntervalMs)) return filteredValue;

	//too late...
	if( (tempReadRequested && now-lastTempReadMillis>5000)) {
		tempReadRequested = false;
	}

	if(!tempReadRequested){ //request temperature reading
		sensors->requestTemperatures(); // Tell the DS18B20 to get make a measurement
		tempReadRequested = true;
		lastTempReadMillis = now;
	}else if(sensors->isConversionComplete()){
		float temp = index>=0?sensors->getTempCByIndex(index):sensors->getTempC(deviceAddress);
		tempReadRequested = false;
		firArray[firIdx%firNOfSamples] = temp;
		firIdx=firIdx+1;
		if(firIdx>firIdxMax)
			firIdx=firNOfSamples;
		if(firIdx<firNOfSamples) return 0;
		float sum = 0;
		for(int i=0;i<firNOfSamples;i++)sum = sum+firArray[i];
		filteredValue = sum / firNOfSamples;
	}

	return filteredValue;
}

TemperatureProbe::~TemperatureProbe() {
	// TODO Auto-generated destructor stub
}

