/*
 * Menu.h
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#ifndef MENU_H_
#define MENU_H_

class ConfigMenu;
#include <WString.h>
#include <vector>
#include <Arduino.h>
#include "tempProbe.h"

enum EncoderMovement {EncMoveNone=-1,EncMoveCW=0,EncMoveCCW=1};

//enum EncoderPushButtonState {EncoderPushButtonNone=0, EncoderPushButtonPressed=1};
enum EncoderSwStates {EncoderPressNone=0,EncoderPressPressed=1,EncoderPressLongPressed=2,EncoderPressDblPressed=3};
static const char* EncoderSwStatesNames[] = { "None", "Pressed", "Long Pressed","Double Pressed" };

typedef  void (*SelectedInMenuCallback)();
typedef  void (*EncoderMovementCallback)(EncoderMovement mvmnt);
typedef  void (*EncoderPushCallback)(EncoderSwStates pst);

class MenuItem;
class MainMenu;
class ConfigMenu;
class RunMenu;

typedef MenuItem* MenuItemPtr;

class MenuItem {
protected:
	MenuItem* parent;
public:
	bool Visible=true;
	bool Selected=false;
    int mappedState = -1; //mapped to an int because unable to resolve compile errors
	String Caption;
	std::vector<MenuItem *> subMenuItems;
    int subMenuItemsLen(){
    	return subMenuItems.size();
    }

//    MenuItem(MenuItem* parent,int state);
    MenuItem(MenuItem* parent,int state, String s);
    virtual ~MenuItem(){}

	virtual void OnSelectedInMenu();
	virtual void OnLongPress();
	virtual void OnDoublePress();
	virtual void HandleEncoderMovement(EncoderMovement mvmnt);
	virtual void HandleEncoderPush(EncoderSwStates pst);
};

class CallbackMenuItem : public MenuItem {
protected:
	SelectedInMenuCallback callBack = NULL;
	EncoderMovementCallback encoderMovementCallBack = NULL;
	EncoderPushCallback encoderPushCallback = NULL;
public:
	CallbackMenuItem(MenuItem* parent,int state, String s,SelectedInMenuCallback cb);
    virtual ~CallbackMenuItem(){}

	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};


class UpMenu : public MenuItem {
public:
	int upState=-1;
	UpMenu(MenuItem* parent,int upstate);
	void OnSelectedInMenu();
};

class ServoConfigDirMenu : public MenuItem {
public:
	ServoConfigDirMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class ServoConfigMinMenu : public MenuItem {
public:
	ServoConfigMinMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class ServoConfigMaxMenu : public MenuItem {
public:
	ServoConfigMaxMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class ServoConfigMenu : public MenuItem {
public:
	ServoConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();

	ServoConfigDirMenu* dirMenu;
	ServoConfigMinMenu* minMenu;
	ServoConfigMaxMenu* maxMenu;
};

class KpPidConfigMenu : public MenuItem {
public:
	KpPidConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class KiPidConfigMenu : public MenuItem {
public:
	KiPidConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class KdPidConfigMenu : public MenuItem {
public:
	KdPidConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class SampleTimePidConfigMenu : public MenuItem {
public:
	SampleTimePidConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class PidConfigMenu : public MenuItem {
public:
	PidConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();

	KpPidConfigMenu* kpMenu;
	KiPidConfigMenu* kiMenu;
	KdPidConfigMenu* kdMenu;
	SampleTimePidConfigMenu* sampleTimeMenu;
};

class ProbeCorrectionMenu : public MenuItem {
public:
	ProbeCorrectionMenu(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderSwStates pst);
};

class ControllerSelection : public MenuItem{
public:
	ControllerSelection(MenuItem* parent);
	void OnSelectedInMenu();
	void HandleEncoderPush(EncoderSwStates pst);
	void HandleEncoderMovement(EncoderMovement mvmnt);
};

class ConfigMenu : public MenuItem{
public:
	ConfigMenu(MenuItem* parent);
	void OnSelectedInMenu();

	MenuItem* upMenu;
	ControllerSelection* configControllerMenu;
	ServoConfigMenu* servoMenu;
	PidConfigMenu* pidMenu;
	ProbeCorrectionMenu* correctionMenu;
};

class RunAutoBaseMenu : public MenuItem {

public:
	int CtrlIdx = 0;
	RunAutoBaseMenu(MenuItem* parent,int state, String s,int ctrlIdx):MenuItem(parent,state,s){
		CtrlIdx=ctrlIdx;
	}
};


class RunAutoSetpointMenu : public RunAutoBaseMenu {
public:
	RunAutoSetpointMenu(MenuItem* parent,int state, int ctrlIdx);
	void OnSelectedInMenu();
};

class RunAutoRampMenu : public RunAutoBaseMenu {
public:

	RunAutoRampMenu(MenuItem* parent,int state, int ctrlIdx);
	void OnSelectedInMenu();
};

class RunAutoSwitch : public RunAutoBaseMenu {
public:

	RunAutoSwitch(MenuItem* parent, int ctrlIdx);
	void OnDoublePress();
	void OnSelectedInMenu();
};

class RunAutoMenu : public MenuItem {
public:
	RunAutoMenu(MenuItem* parent);
	void OnSelectedInMenu();

	RunAutoSetpointMenu* setpointMenu0;
	RunAutoRampMenu* rampMenu0;
	RunAutoSwitch* switchMenu0;

	RunAutoSetpointMenu* setpointMenu1;
	RunAutoRampMenu* rampMenu1;
	RunAutoSwitch* switchMenu1;

	void HandleEncoderPush(EncoderSwStates pst);
	void HandleEncoderMovement(EncoderMovement mvmnt);
};

class MainMenu : public MenuItem {
private:


public:
	MainMenu();
	void OnSelectedInMenu();



	ConfigMenu* configMenu;
	RunAutoMenu* runMenu;
};


#endif /* MENU_H_ */
