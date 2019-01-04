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
#include "PidState.h"

typedef  void (*SelectedInMenuCallback)();
typedef  void (*EncoderMovementCallback)(EncoderMovement mvmnt);
typedef  void (*EncoderPushCallback)(EncoderPushButtonState pst);

class MenuItem;
class MainMenu;
class ConfigMenu;
class RunMenu;

typedef MenuItem* MenuItemPtr;

class MenuItem {
protected:
public:
	bool Selected=false;
    int mappedState = -1; //mapped to an int because unable to resolve compile errors
	String Caption;
	std::vector<MenuItem *> subMenuItems;
    int subMenuItemsLen(){
    	return subMenuItems.size();
    }

    MenuItem(int state);
    MenuItem(int state, String s);
    virtual ~MenuItem(){}

	virtual void OnSelectedInMenu()=0;
	virtual void HandleEncoderMovement(EncoderMovement mvmnt);
	virtual void HandleEncoderPush(EncoderPushButtonState pst);
};

class CallbackMenuItem : public MenuItem {
protected:
	SelectedInMenuCallback callBack = NULL;
	EncoderMovementCallback encoderMovementCallBack = NULL;
	EncoderPushCallback encoderPushCallback = NULL;
public:
	CallbackMenuItem(int state, String s,SelectedInMenuCallback cb);
    virtual ~CallbackMenuItem(){}

	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};


class UpMenu : public MenuItem {
public:
	int upState=-1;
	UpMenu(int upstate);
	void OnSelectedInMenu();
};

class ServoConfigDirMenu : public MenuItem {
public:
	ServoConfigDirMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class ServoConfigMinMenu : public MenuItem {
public:
	ServoConfigMinMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class ServoConfigMaxMenu : public MenuItem {
public:
	ServoConfigMaxMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class ServoConfigMenu : public MenuItem {
public:
	ServoConfigMenu();
	void OnSelectedInMenu();

	ServoConfigDirMenu* dirMenu;
	ServoConfigMinMenu* minMenu;
	ServoConfigMaxMenu* maxMenu;
};

class KpPidConfigMenu : public MenuItem {
public:
	KpPidConfigMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class KiPidConfigMenu : public MenuItem {
public:
	KiPidConfigMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class KdPidConfigMenu : public MenuItem {
public:
	KdPidConfigMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class SampleTimePidConfigMenu : public MenuItem {
public:
	SampleTimePidConfigMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class PidConfigMenu : public MenuItem {
public:
	PidConfigMenu();
	void OnSelectedInMenu();

	KpPidConfigMenu* kpMenu;
	KiPidConfigMenu* kiMenu;
	KdPidConfigMenu* kdMenu;
	SampleTimePidConfigMenu* sampleTimeMenu;
};

class ConfigMenu : public MenuItem{
public:
	ConfigMenu();
	void OnSelectedInMenu();

	MenuItem* upMenu;
	ServoConfigMenu* servoMenu;
	PidConfigMenu* pidMenu;
};

class AutoTuneResultMenu : public MenuItem {
public:
	AutoTuneResultMenu();
	void OnSelectedInMenu();

	MenuItem* confirmNSaveMenu;
	MenuItem* discardMenu;
};

class RunAutoTuneMenu : public MenuItem {
public:
	RunAutoTuneMenu();
	AutoTuneResultMenu* autoTuneResultMenu;

	void OnSelectedInMenu();
	void HandleEncoderPush(EncoderPushButtonState pst);
};

class RunAutoSetpointMenu : public MenuItem {
public:
	RunAutoSetpointMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
};

class RunAutoRampMenu : public MenuItem {
public:
	RunAutoRampMenu();
	void OnSelectedInMenu();
	void HandleEncoderMovement(EncoderMovement mvmnt);
};

class RunAutoTimerMenu : public MenuItem {
public:
	RunAutoTimerMenu();
	void OnSelectedInMenu();
};


class RunAutoMenu : public MenuItem {
public:
	RunAutoMenu();
	void OnSelectedInMenu();

	RunAutoSetpointMenu* setpointMenu;
	RunAutoRampMenu* rampMenu;
	RunAutoTimerMenu* timerMenu;

	void HandleEncoderPush(EncoderPushButtonState pst);
	void HandleEncoderMovement(EncoderMovement mvmnt);
};

class RunManualMenu : public MenuItem {
public:
	RunManualMenu();
	void OnSelectedInMenu();

	void HandleEncoderPush(EncoderPushButtonState pst);
	void HandleEncoderMovement(EncoderMovement mvmnt);
};


class RunMenu : public MenuItem {
public:
	RunMenu();
	void OnSelectedInMenu();

	RunAutoMenu* runAutoMenu;
	RunManualMenu* runManualMenu;
	RunAutoTuneMenu* runAutoTuneMenu;
};

class MainMenu : public MenuItem {
public:
	MainMenu();
	void OnSelectedInMenu();

	ConfigMenu* configMenu;
	RunMenu* runMenu;
};



void InitializeMenus();

#endif /* MENU_H_ */
