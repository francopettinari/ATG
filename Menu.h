/*
 * Menu.h
 *
 *  Created on: 06 nov 2018
 *      Author: franc
 */

#ifndef MENU_H_
#define MENU_H_

#include <WString.h>
class MenuItem;

typedef  void (*MenuItemCallback)(MenuItem* parent);

class MenuItem {
public:
	String Caption;
	MenuItemCallback callBack = NULL;
	MenuItem* parent = NULL;
	MenuItem** subMenuItems = NULL;
    int subMenuItemsLen = 0;

	MenuItem(MenuItem* parent, String s,MenuItemCallback cb){
		this->parent = parent;
		Caption = s;
		callBack = cb;
	}
};

typedef MenuItem* MenuItemPtr;

void InitializeMenus();

#endif /* MENU_H_ */
