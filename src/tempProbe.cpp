/*
 * fir.cpp
 *
 *  Created on: 15 feb 2020
 *      Author: franc
 */

#include "tempProbe.h"

#include <Arduino.h>
#include <DallasTemperature.h>
#include "ATG.h"

TemperatureProbe::TemperatureProbe(int pin) {
	onewire = new OneWire(pin);
	sensors = new DallasTemperature(onewire);

	sensors->setWaitForConversion(false);
	sensors->begin();
	sensors->setResolution(10); // (9, 10, 11, or 12 bits) => f 0.5�C, 0.25�C, 0.125�C, or 0.0625�C

	firIdx = 0;
	filteredValue = 0;
}

float TemperatureProbe::readTemperature(){
	unsigned long now = millis();

	if(now-lastTempReadMillis<readIntervalMs) return filteredValue; //too early

	sensors->requestTemperatures(); // Tell the DS18B20 to get make a measurement
	lastTempReadMillis = now;
	float temp = sensors->getTempCByIndex(0);

	if(temp<=0) return filteredValue; //filter issue related to multicore and OneWire

	firArray[firIdx%firNOfSamples] = temp;
	firIdx=firIdx+1;
	if(firIdx>firIdxMax)
		firIdx=firNOfSamples;
	if(firIdx<firNOfSamples) return 0;
	float sum = 0;
	for(int i=0;i<firNOfSamples;i++)sum = sum+firArray[i];
	filteredValue = sum / firNOfSamples;

	return filteredValue;
}

//float TemperatureProbe::readTemperature(){
//	unsigned long now = millis();
//
//	//too early
//	if( (!tempReadRequested && now-lastTempReadMillis<readIntervalMs)) return filteredValue;
//
//	//too late...
//	if( (tempReadRequested && now-lastTempReadMillis>5000)) {
//		tempReadRequested = false;
//	}
//
//	if(!tempReadRequested){ //request temperature reading
//		sensors->requestTemperatures(); // Tell the DS18B20 to get make a measurement
//		tempReadRequested = true;
//		lastTempReadMillis = now;
//	}else if(sensors->isConversionComplete()){
//		float temp = sensors->getTempCByIndex(0);
//		tempReadRequested = false;
//		if(temp<=0) return filteredValue;
//
//		firArray[firIdx%firNOfSamples] = temp;
//		firIdx=firIdx+1;
//		if(firIdx>firIdxMax)
//			firIdx=firNOfSamples;
//		if(firIdx<firNOfSamples) return 0;
//		float sum = 0;
//		for(int i=0;i<firNOfSamples;i++)sum = sum+firArray[i];
//		filteredValue = sum / firNOfSamples;
//	}
//
//	return filteredValue;
//}

TemperatureProbe::~TemperatureProbe() {
}

