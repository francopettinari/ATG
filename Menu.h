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

typedef  void (*MenuItemCallback)();

class MenuItem;
class MainMenu;
class ConfigMenu;
class RunMenu;

typedef MenuItem* MenuItemPtr;

class MenuItem {
protected:
public:
    int mappedState = -1; //mapped to an int because unable to resolve compile errors
	String Caption;
	MenuItem* parent = NULL;
	std::vector<MenuItem *> subMenuItems;
    int subMenuItemsLen(){
    	return subMenuItems.size();
    }

    MenuItem(MenuItem* parent, int state);
    MenuItem(MenuItem* parent, int state, String s);
    virtual ~MenuItem(){}

	virtual void OnPress()=0;
};

class CallbackMenuItem : public MenuItem {
protected:
	MenuItemCallback callBack;
public:
	CallbackMenuItem(MenuItem* parent, int state, String s,MenuItemCallback cb);
    virtual ~CallbackMenuItem(){}

	void OnPress();
};

class UpMenu : public MenuItem {
public:
	int upState=-1;
	UpMenu(MenuItem* parent,int upstate);
	void OnPress();
};

class ServoConfigMenu : public MenuItem {
public:
	ServoConfigMenu(ConfigMenu* parent);
	void OnPress();

	MenuItem* dirMenu;
	MenuItem* minMenu;
	MenuItem* maxMenu;
};

class PidConfigMenu : public MenuItem {
public:
	PidConfigMenu(ConfigMenu* parent);
	void OnPress();

	MenuItem* kpMenu;
	MenuItem* kiMenu;
	MenuItem* kdMenu;
};

class ConfigMenu : public MenuItem{
public:
	ConfigMenu(MainMenu* parent);
	void OnPress();

	MenuItem* upMenu;
	ServoConfigMenu* servoMenu;
	PidConfigMenu* pidMenu;
};

class AutoTuneResultMenu : public MenuItem {
public:
	AutoTuneResultMenu(RunMenu* parent);
	void OnPress();

	MenuItem* confirmNSaveMenu;
	MenuItem* discardMenu;
};

class RunAutoTuneMenu : public MenuItem {
public:
	RunAutoTuneMenu(RunMenu* parent);
	AutoTuneResultMenu* autoTuneResultMenu;

	void OnPress();
};

class RunMenu : public MenuItem {
public:
	RunMenu(MainMenu* parent);
	void OnPress();

	MenuItem* runAutoMenu;
	RunAutoTuneMenu* runAutoTuneMenu;
};

class MainMenu : public MenuItem {
public:
	MainMenu();
	void OnPress();

	ConfigMenu* configMenu;
	RunMenu* runMenu;
};



void InitializeMenus();

#endif /* MENU_H_ */
