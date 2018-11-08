#include "Menu.h"
#include "Pid.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"

void buildMainMenu(MenuItem* mainMenu);
void biuldConfigMenu(MenuItem* mainMenu);
void buildRunMenu(MenuItem* runMenu);
void buildConfigTCorrectionMenu(MenuItem* configMenu);

void InitializeMenus(){
	pidState.topMenu = new MenuItem(NULL,F("Main"),NULL);
	buildMainMenu(pidState.topMenu);
	pidState.setCurrentMenu(pidState.topMenu);
}

void buildMainMenu(MenuItem* mainMenu){
	MenuItem* configMenu = new MenuItem(mainMenu,F("Config"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->subMenuItems[0]);
	});

	MenuItem* runMenu = new MenuItem(mainMenu,F("Run"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->subMenuItems[1]);
	});

	pidState.topMenu->subMenuItems = new MenuItemPtr[2];
	pidState.topMenu->subMenuItemsLen = 2;
	pidState.topMenu->subMenuItems[0] = configMenu;
	pidState.topMenu->subMenuItems[1] = runMenu;

	biuldConfigMenu(configMenu);
	buildRunMenu(runMenu);
}

void biuldConfigMenu(MenuItem* configMenu){
	configMenu->subMenuItems = new MenuItemPtr[2];
	configMenu->subMenuItemsLen = 2;
	configMenu->subMenuItems[0] = new MenuItem(configMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);
	});
	MenuItem* tCorrMenu = new MenuItem(configMenu,F("Temp correction"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->subMenuItems[1]);
	});
	configMenu->subMenuItems[1] = tCorrMenu;
	buildConfigTCorrectionMenu(configMenu);
}

void buildRunMenu(MenuItem* runMenu){
	runMenu->subMenuItems = new MenuItemPtr[2];
	runMenu->subMenuItemsLen = 2;
	runMenu->subMenuItems[0] = new MenuItem(runMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);

	});
	MenuItem* heatMenu = new MenuItem(runMenu,F("Heat"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->subMenuItems[1]);
	});
	runMenu->subMenuItems[1] = heatMenu;
	heatMenu->subMenuItems = new MenuItemPtr[1];
	heatMenu->subMenuItemsLen = 1;
	heatMenu->subMenuItems[0] = new MenuItem(heatMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);
	});
}

void buildConfigTCorrectionMenu(MenuItem* tCorrMenu){
	tCorrMenu->subMenuItems = new MenuItemPtr[1];
	tCorrMenu->subMenuItemsLen = 1;
	tCorrMenu->subMenuItems[0] = new MenuItem(tCorrMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);
	});
}


