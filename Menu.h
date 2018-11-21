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
#include "PidState.h"

typedef  void (*MenuItemCallback)();

class MainMenu;
class ConfigMenu;


class MenuItem {
protected:
	MenuItemCallback callBack;
public:
    int mappedState = -1; //mapped to an int because unable to resolve compile errors
	String Caption;
	MenuItem* parent = NULL;
	MenuItem** subMenuItems = NULL;
    int subMenuItemsLen = 0;

    MenuItem(MenuItem* parent, int state);
	MenuItem(MenuItem* parent, int state, String s,MenuItemCallback cb);
    virtual ~MenuItem(){}

	virtual void OnPress();
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

	MenuItem* dirMenu;
	MenuItem* minMenu;
	MenuItem* maxMenu;
};

class PidConfigMenu : public MenuItem {
public:
	PidConfigMenu(ConfigMenu* parent);

	MenuItem* kpMenu;
	MenuItem* kiMenu;
	MenuItem* kdMenu;
};

class ConfigMenu : public MenuItem{
public:
	ConfigMenu(MainMenu* parent);

	MenuItem* upMenu;
	MenuItem* tempCorrectionMenu;
	ServoConfigMenu* servoMenu;
	PidConfigMenu* pidMenu;
};

class RunMenu : public MenuItem {
public:
	RunMenu(MainMenu* parent);
	MenuItem* runAutoMenu;
	MenuItem* runAutoTuneMenu;
};

class MainMenu : public MenuItem {
public:
	MainMenu();

	ConfigMenu* configMenu;
	RunMenu* runMenu;
};

typedef MenuItem* MenuItemPtr;

void InitializeMenus();

#endif /* MENU_H_ */
