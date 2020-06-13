/*
 * TemperatureStep.h
 *
 *  Created on: 13 giu 2020
 *      Author: franc
 */

#ifndef TIMERFSM_TEMPERATURESTEP_H_
#define TIMERFSM_TEMPERATURESTEP_H_

namespace filter {

class TemperatureStep {
public:
	TemperatureStep();
	virtual ~TemperatureStep();

	int status;
};

} /* namespace filter */

#endif /* TIMERFSM_TEMPERATURESTEP_H_ */
