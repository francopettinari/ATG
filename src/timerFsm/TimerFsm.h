/*
 * Fsm.h
 *
 *  Created on: 13 giu 2020
 *      Author: franc
 */

#ifndef TIMERFSM_TIMERFSM_H_
#define TIMERFSM_TIMERFSM_H_

#include <vector>
#include <map>
#include <WString.h>
#include "TemperatureStep.h"

namespace filter {

class TimerFsmEvent;
class TimerFsmState;
class TimerFsmAction;
class TimerFsmTransition;
class TimerFsmContext;

typedef void (*onTransitionDoneCallback)(TimerFsmTransition*);

class TimerFsm {
	std::vector<TimerFsmTransition*> _transitions;
	std::map<String, TimerFsmState*> states;
	//private static object fsmTransitionLock = new object();
protected:

	bool doActions(TimerFsmContext& context, TimerFsmTransition& transition, TimerFsmEvent& evt, TemperatureStep& s);
public:
	TimerFsm();
	virtual ~TimerFsm();

	TimerFsmState* getState(String name);
	TimerFsmState* getState(int code);
	void addState(TimerFsmState& state);
	TimerFsmTransition* addTransition(String transitionName, String triggerEvent, String srcStateName, String dstStateName);
	String setCorrespondingStateName(int code);
	String getCorrespondingStateName(int code);
	void dispatchFsmEvent(TimerFsmEvent& e, TemperatureStep& s, TimerFsmContext* context);
	void dispatchFsmEvent(TimerFsmEvent& e, TemperatureStep& s, onTransitionDoneCallback onTransitionDone);
	bool containsTransition(int sourceStatus, int destinationStatus);
	bool containsTransition(int sourceStatus, int destinationStatus, String triggerEvent);
	bool containsValidTransition(TemperatureStep t, int sourceStatus, int destinationStatus, String triggerEvent);
	bool canHandleEventFromStatus(int status, String eventName);
	//FsmTransitionState ValidateTransition(T t, int sourceStatus, int destinationStatus, string triggerEvent);
};

class TimerFsmTransition {
	String _name;
public:
	TimerFsmTransition(String name);
	virtual ~TimerFsmTransition();

	std::vector<TimerFsmAction *> fsmActions;
	//private List<TaskFsmTransitionValidation<T>> validations = new List<TaskFsmTransitionValidation<T>>(){TaskFsmTransitionValidation<T>.Default};
	String getName(){ return _name;}
	String triggerEvent;
	TimerFsmState* nextState;
	TimerFsmState* state;
};

class TimerFsmContext {
	TimerFsm& _FSM;
	TemperatureStep* _adaptee;
public:
	TimerFsmContext(TimerFsm& fsm,TemperatureStep* pTs);
	virtual ~TimerFsmContext();



	TimerFsmEvent* currentEvent;
	TimerFsmTransition* currentTransition;
	onTransitionDoneCallback onTransitionDone;
	bool forwarded;
	std::map<String,void*> attributes;

	TimerFsmTransition* setCurrentTransition(TimerFsmTransition* t);
	TimerFsm& getFSM(){return _FSM;}
	TimerFsmState* getState(int code);
	TemperatureStep* getAdaptee(){ return _adaptee; }
	TemperatureStep* setFsmObject(TemperatureStep* obj);
	void reset(TimerFsmTransition* t,TimerFsmEvent* e,TemperatureStep* s);
};

class TimerFsmEvent {
public:
	TimerFsmEvent();
	virtual ~TimerFsmEvent();

	bool userConfirmed,aborted;
	float millis;
	String name;

};

class TimerFsmState {
	std::map<String,TimerFsmTransition*> transitions;
	String _name;
public:
	TimerFsmState();
	virtual ~TimerFsmState();

	TimerFsm* fsm;
	int code;

	String getName(){return _name;}
	TimerFsmTransition* getTransitionByTriggerEvent(String triggerEvent);
	void addTransition(TimerFsmTransition& transition);
	bool containsTransitionToState(int code);
	bool containsTransitionToState(int code, String triggerEvent);
	TimerFsmTransition* getTransitionToState(int code, String triggerEvent);
	bool hasTransitionByTriggerEvent(String triggerEvent);
};

class TimerFsmAction {
public:
	TimerFsmAction();
	virtual ~TimerFsmAction();
};

} /* namespace filter */

#endif /* TIMERFSM_TIMERFSM_H_ */
