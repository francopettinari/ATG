/*
 * PidState.h
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#ifndef PIDSTATE_H_
#define PIDSTATE_H_

#include <WString.h>
#include "Menu.h"

class LCDHelper;
#include "LCDHelper.h"

class PidState {
	float lastPressMillis=0;

	void updateLcd();
private:
	MenuItem         *currentMenu=NULL;
public:
	LCDHelper        *lcdHelper=NULL;

	MenuItem         *topMenu = NULL;
	int              state = 0;
	int              stateSelection = 0;
	int              currEncoderPos=0,prevEncoderPos=0;    // a counter for the rotary encoder dial
	PidState(){}

	void setCurrentMenu(MenuItem *m){
		currentMenu = m;
		stateSelection=0;
		if(lcdHelper!=NULL){
			lcdHelper->display(*this);
		}
	}

	MenuItem  *getCurrentMenu(){return currentMenu;}

	void update(int encoderPos, boolean encoderPress){
		prevEncoderPos = currEncoderPos;
		currEncoderPos = encoderPos/4;
		if(currEncoderPos<prevEncoderPos){
			stateSelection--;
			if(stateSelection<0) stateSelection=0;
		}else if(currEncoderPos>prevEncoderPos){
			stateSelection++;
			if(stateSelection>currentMenu->subMenuItemsLen-1) stateSelection=currentMenu->subMenuItemsLen-1;
		}
		if(encoderPress){
			if(lastPressMillis>0 ){
				if ((millis()-lastPressMillis)<500){
					return;
				}
			}
		    lastPressMillis=millis();
		    MenuItem* selMI = currentMenu->subMenuItems[stateSelection];
		    if(selMI->callBack!=NULL){
		    	currentMenu->subMenuItems[stateSelection]->callBack(selMI->parent);
		    }
		}else{
			lastPressMillis = -1;
		}
	}
};


#endif /* PIDSTATE_H_ */
