/*
 * Fsm.cpp
 *
 *  Created on: 13 giu 2020
 *      Author: franc
 */

#include "TimerFsm.h"
#include "Arduino.h"
namespace filter {

TimerFsm::TimerFsm() {
	// TODO Auto-generated constructor stub

}

TimerFsm::~TimerFsm() {
	// TODO Auto-generated destructor stub
}

TimerFsmState* TimerFsm::getState(String name){
	auto it = states.find(name);
	if(it==states.end()){
		return NULL;
	}

	return it->second;
}

TimerFsmState* TimerFsm::getState(int code) {
	auto it = states.begin();
	while(it!=states.end()){
		if (it->second->code == code) {
			return it->second;
		}
		it++;
	}
	return NULL;
}

void TimerFsm::addState(TimerFsmState& state) {
	state.fsm = this;
	this->states[state.getName()] = &state;
}

TimerFsmTransition* TimerFsm::addTransition(String transitionName, String triggerEvent, String srcStateName, String dstStateName){
	Serial.println("FSM Transition: " + transitionName + "; Source: " + srcStateName + "; Destination: " + dstStateName + "; Event: " + triggerEvent);
	TimerFsmState* src = getState(srcStateName);
	if (src==NULL) {
		return NULL;
	}
	TimerFsmState* dst = getState(dstStateName);
	if (dst==NULL) {
		return NULL;
	}

	TimerFsmTransition* transition = src->getTransitionByTriggerEvent(triggerEvent);
	if (transition != NULL){
		Serial.println("Transition for trigger event already present with name "+transition->getName());
		return transition;
	}

	transition = new TimerFsmTransition(transitionName);
	transition->nextState = dst;
	transition->triggerEvent = triggerEvent;
	src->addTransition(*transition);
	Serial.println("Transition "+transitionName+" added");

	return transition;
}

String TimerFsm::getCorrespondingStateName(int code) {
	TimerFsmState* state = getState(code);
	if(state==NULL) return F("");
	return state->getName();
}
//
//        public int GetCorrespondingInternalStateName(string fsmName, string stateName) {
//            return GetState(fsmName, stateName).Code;
//        }
//
void TimerFsm::dispatchFsmEvent(TimerFsmEvent& e, TemperatureStep& s, TimerFsmContext* context) {
	if (context == NULL)
	{
		context = new TimerFsmContext(*this, &s);
	}
	TemperatureStep* last = context->setFsmObject(&s);
//	string fsmName = context.FSM.Name;

	String currentStateName = context->getFSM().getCorrespondingStateName(context->getAdaptee()->status);
	if(currentStateName==F("")){
		Serial.println("No state found for status "+context->getAdaptee()->status);
		return;
	}
	TimerFsmState* currentState = getState(currentStateName);
	if(currentState==NULL){
		Serial.println("No state found for status "+context->getAdaptee()->status);
		return;
	}

	TimerFsmTransition* transition = currentState->getTransitionByTriggerEvent(e.name);
	if (transition == NULL){
		Serial.println("No transition found for "+e.name);
		return;
	}

	TimerFsmEvent* lastEvent = context->currentEvent;
	context->currentEvent = &e;
	TimerFsmTransition* lastTransition = context->setCurrentTransition(transition);

	if(!doActions(*context, *transition, e, s)){
		e.aborted = true;
		context->reset(lastTransition,lastEvent,last);
		return;
	}
	// don't complete the transition on any errors
	if (context->onTransitionDone != NULL) context->onTransitionDone(transition);

	context->reset(lastTransition,lastEvent,last);
}

//
//        protected virtual void DoActions(FsmContext<T> context, FsmTransition<T> transition, FsmEvent evt, T task) {
//            if (transition == null) throw new ArgumentException("transition was null");
//            if (task == null) {
//                throw new FSMException("No target specified while activating transition " +
//                                       transition.Name + "\n");
//            }
//
//            //Update the status to the new one
//            Logger.info(new StringBuilder()
//                        .Append("Doing transition <").Append(transition.Name)
//                        .Append("> from <").Append(context.FSM.GetCorrespondingStateName(task.Status))
//                        .Append("> to <").Append(transition.NextState.Name)
//                        .Append("> by event <").Append(evt.Name)
//                        .Append("> on: ").Append(task.Id)
//                        .ToString());
//
//            var valRes = transition.Validate(task);
//            if (valRes.StopTransactionIfFailed
//                     && (!valRes.Enabled || !valRes.Active)) {
//                Logger.error("Task Validation failed! Not allowed to proceed the transaction. Active: '{0}' - Enable: '{1}' !", valRes.Active, valRes.Enabled);
//
//                if (!valRes.ErrorMessage.IsNullOrEmpty()) {
//                    MApplication.Instance.ShowMessage(
//                         MApplication.Translate("Error", "Error"), valRes.ErrorMessage, MessageType.Error);
//                }
//
//                return;
//            }
//
//
//            //Apply the actions
//            foreach (FsmAction<T> action in transition.Actions) {
//                if (!action.Enabled) continue;
//                //if (ctx.getRollbackOnly()) return; // no point we continue a failed transaction
//
//                if (action.SupportsEvent(evt)) {
//                    int? tId = context.Adaptee == null ? (int?)null : context.Adaptee.Id;
//                    Logger.info(new StringBuilder()
//                        .Append("Executing action " + action.GetType().FullName + " from <").Append(context.FSM.GetCorrespondingStateName(task.Status))
//                        .Append("> to <").Append(transition.NextState.Name)
//                        .Append("> by event <").Append(evt.Name)
//                        .Append("> on: ").Append(task.Id)
//                        .ToString());
//                    action.Context = context;
//                    action.Execute(context);
//                } else {
//                    ATGApplication.ATGInstance.DB.RollbackTransaction();
//                    throw new EventNotAllowedForAction(action.GetType().FullName, evt.Name);
//                }
//            }
//
//            //target.SetCurrentStateName(transition.NextState.Name, evt);
//        }

void TimerFsm::dispatchFsmEvent(TimerFsmEvent& e, TemperatureStep& s, onTransitionDoneCallback onTransitionDone){
	TimerFsmContext* context = new TimerFsmContext(*this, &s);
	context->onTransitionDone = onTransitionDone;
	dispatchFsmEvent(e, s, context);
}

//        public bool ContainsTransition(int sourceStatus, int destinationStatus)
//        {
//            try {
//                FsmState<T> srcState = getState(sourceStatus);
//                if (srcState == null) return false;
//                return srcState.ContainsTransitionToState(destinationStatus);
//            }
//            catch (DoesNotExistException ex) {
//                return false;
//            }
//        }
//
//
//
//        public bool ContainsTransition(int sourceStatus, int destinationStatus, string triggerEvent) {
//            try {
//                FsmState<T> srcState = getState(sourceStatus);
//                if (srcState == null) return false;
//                return srcState.ContainsTransitionToState(destinationStatus, triggerEvent);
//            }
//            catch (DoesNotExistException ex) {
//                return false;
//            }
//        }
//
//        public bool ContainsValidTransition(T t, int sourceStatus, int destinationStatus, string triggerEvent) {
//            FsmTransitionState valSt = ValidateTransition(t, sourceStatus, destinationStatus, triggerEvent);
//            return valSt.Active && valSt.Enabled;
//        }
//
//        public bool CanHandleEventFromStatus(int status, string eventName) {
//            try {
//                FsmState<T> srcState = getState(status);
//                if (srcState == null) return false;
//                return srcState.HasTransitionByTriggerEvent(eventName);
//            }
//            catch (DoesNotExistException ex) {
//                return false;
//            }
//        }
//
//        public FsmTransitionState ValidateTransition(T t, int sourceStatus, int destinationStatus, string triggerEvent) {
//            try {
//                FsmState<T> srcState = getState(sourceStatus);
//                if (srcState == null) return FsmTransitionState.Null;
//                FsmTransition<T> transition = srcState.GetTransitionToState(destinationStatus, triggerEvent);
//                if (transition == null) return FsmTransitionState.Null;
//                return transition.Validate(t);
//            }
//            catch (DoesNotExistException ex) {
//                return FsmTransitionState.Null;
//            }
//        }

TimerFsmAction::TimerFsmAction() {
	// TODO Auto-generated constructor stub

}

TimerFsmAction::~TimerFsmAction() {
	// TODO Auto-generated destructor stub
}

TimerFsmContext::TimerFsmContext(TimerFsm& fsm,TemperatureStep* pTs):_FSM(fsm) {
	_adaptee = pTs;
}

TimerFsmContext::~TimerFsmContext() {
	// TODO Auto-generated destructor stub
}

void TimerFsmContext::reset(TimerFsmTransition* t,TimerFsmEvent* e,TemperatureStep* s) {
	currentEvent = e;
	currentTransition = t;
	setFsmObject(s);
}

//public FsmContext(Fsm<T> fsm, T fsmObject) {
//            FSM = fsm;
//            Adaptee = fsmObject;
//        }
//
//        public FsmContext(FsmContext<T> ctx) {
//            fsmObjectUpdatedInThisContext = false;
//            FSM = ctx.FSM;
//            Adaptee = ctx.Adaptee;
//            CurrentEvent = ctx.CurrentEvent;
//            CurrentTransition = ctx.CurrentTransition;
//            OnTransitionDone = ctx.OnTransitionDone;
//            Forwarded = ctx.Forwarded;
//            attributes = ctx.attributes;
//        }
//
//        public FsmContext<T> Clone() {
//            return new FsmContext<T>(this);
//        }
//
//        public void UpdateTask() {
//            if (fsmObjectUpdatedInThisContext) return;//already updated;
//            if (Adaptee == null) return;
//            fsmObjectUpdatedInThisContext = true;
//        }
//
//        public FsmEvent SetCurrentEvent(FsmEvent currentEvent) {
//            FsmEvent last = this.CurrentEvent;
//            this.CurrentEvent = currentEvent;
//            return last;
//        }
//
TimerFsmTransition* TimerFsmContext::setCurrentTransition(TimerFsmTransition* t) {
	TimerFsmTransition* last = currentTransition;
	currentTransition = t;
	return last;
}

TemperatureStep* TimerFsmContext::setFsmObject(TemperatureStep* obj) {
	TemperatureStep* last = getAdaptee();
	_adaptee = obj;
	return last;
}
//
//        public void SetAttribute(String name, Object obj) {
//            //if (obj == null) throw new ArgumentException("object was null");
//            if (attributes == null) {
//                attributes = new Dictionary<Object, Object>();
//            }
//            if (attributes.Remove(name)) {
//                Logger.warn("Replace attribute with key {0}", name);
//            }
//            attributes.Add(name, obj);
//        }
//
//        public Object GetAttribute(String name) {
//            if (attributes == null) return null;
//            object result = null;
//            if (attributes.TryGetValue(name, out result)) {
//                return result;
//            }
//            return null;
//        }
//
//        public void SetAttribute<T>(T obj) {
//            if (obj == null) throw new ArgumentException("object was null");
//            if (attributes == null) {
//                attributes = new Dictionary<Object, Object>();
//            }
//            attributes.Add(obj.GetType().FullName, obj);
//        }
//
//        public T GetAttribute<T>(Type typ) where T : class {
//            if (attributes == null) return null;
//            object result = null;
//            if (attributes.TryGetValue(typ.FullName, out result)) {
//                return (T)result;
//            }
//            return null;
//        }

TimerFsmEvent::TimerFsmEvent() {
	// TODO Auto-generated constructor stub

}

TimerFsmEvent::~TimerFsmEvent() {
	// TODO Auto-generated destructor stub
}

TimerFsmState::TimerFsmState() {
	// TODO Auto-generated constructor stub

}

TimerFsmState::~TimerFsmState() {
	// TODO Auto-generated destructor stub
}

TimerFsmTransition* TimerFsmState::getTransitionByTriggerEvent(String triggerEvent) {
	auto result = transitions.find(triggerEvent);
	if (result==transitions.end()) {
		return NULL;
	}

	return result->second;
}

void TimerFsmState::addTransition(TimerFsmTransition& transition) {
	transition.state = this;
	transitions[transition.triggerEvent] = &transition;
}

bool TimerFsmState::containsTransitionToState(int code) {
	auto it = transitions.begin();
	while (it!= transitions.end()){
	  if(it->second->nextState!=NULL && it->second->nextState->code==code) return true;
	  it++;
	}
	return false;
}

//        public bool ContainsTransitionToState(int code, string triggerEvent) {
//            return transitions.Values.Any(trans => trans.NextState.Code == code && trans.TriggerEvent==triggerEvent);
//        }
//
//        public FsmTransition<T> GetTransitionToState(int code, string triggerEvent) {
//            FsmTransition<T> result = GetTransitionByTriggerEvent(triggerEvent);
//            if (result == null) return null;
//            if (result.NextState.Code == code) return result;
//            return null;
//        }
//
//        public bool HasTransitionByTriggerEvent(string triggerEvent) {
//            return GetTransitionByTriggerEvent(triggerEvent) != null;
//        }

TimerFsmTransition::TimerFsmTransition(String name):_name(name) {
	// TODO Auto-generated constructor stub

}

TimerFsmTransition::~TimerFsmTransition() {
	// TODO Auto-generated destructor stub
}



} /* namespace filter */
