#include "Menu.h"
#include "Pid.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"

void buildMainMenu(MenuItem* mainMenu);
void biuldConfigMenu(MenuItem* mainMenu);
void buildRunMenu(MenuItem* runMenu);
void buildConfigTCorrectionMenu(MenuItem* configMenu);
void buildConfigServoMenu(MenuItem* servoCorrMenu);

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
	configMenu->subMenuItems = new MenuItemPtr[3];
	configMenu->subMenuItemsLen = 3;
	configMenu->subMenuItems[0] = new MenuItem(configMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);
	});
	MenuItem* tCorrMenu = new MenuItem(configMenu,F("Temp Corr"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->subMenuItems[1]);
	});
	configMenu->subMenuItems[1] = tCorrMenu;
	buildConfigTCorrectionMenu(configMenu);

	MenuItem* tServoMenu = new MenuItem(configMenu,F("Servo"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->subMenuItems[2]);
	});
	configMenu->subMenuItems[2] = tServoMenu;
	buildConfigServoMenu(tServoMenu);
}

void buildRunMenu(MenuItem* runMenu){
	runMenu->subMenuItems = new MenuItemPtr[2];
	runMenu->subMenuItemsLen = 2;
	runMenu->subMenuItems[0] = new MenuItem(runMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);

	});
	MenuItem* heatMenu = new MenuItem(runMenu,F("Auto"),[](MenuItem* parent){
		if(pidState.state == None){
			pidState.state = RunAuto;
			pidState.setCurrentMenu(parent->subMenuItems[1]);
		}else{
			pidState.state = None;
		}
	});
	runMenu->subMenuItems[1] = heatMenu;
	heatMenu->subMenuItems = new MenuItemPtr[1];
	heatMenu->subMenuItemsLen = 1;
	heatMenu->subMenuItems[0] = new MenuItem(heatMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);
	});
}

void buildConfigServoMenu(MenuItem* servoCorrMenu){
	servoCorrMenu->subMenuItems = new MenuItemPtr[4];
	servoCorrMenu->subMenuItemsLen = 4;
	servoCorrMenu->subMenuItems[0] = new MenuItem(servoCorrMenu,F("Up"),[](MenuItem* parent){
		pidState.setCurrentMenu(parent->parent);
	});
	servoCorrMenu->subMenuItems[1] = new MenuItem(servoCorrMenu,F("Dir"),[](MenuItem* parent){
		pidState.state = Config_ServoDirection;
	});
	servoCorrMenu->subMenuItems[2] = new MenuItem(servoCorrMenu,F("Min"),[](MenuItem* parent){
		if(pidState.state == None){
			pidState.state = Config_ServoMin;
		}else{
			pidState.state = None;
			pidState.setCurrentMenu(parent->parent);
		}

	});
	servoCorrMenu->subMenuItems[3] = new MenuItem(servoCorrMenu,F("Max"),[](MenuItem* parent){
		if(pidState.state == None){
			pidState.state = Config_ServoMax;
		}else{
			pidState.state = None;
			pidState.setCurrentMenu(parent->parent);
		}
	});
}

void buildConfigTCorrectionMenu(MenuItem* configMenu){

}


