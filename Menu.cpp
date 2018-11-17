#include "Menu.h"
#include "Pid.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"


//MenuItem* getCurrentMenu(){
//	MainMenu* pmm = (MainMenu*) pidState.topMenu;
//	switch(pidState.getState()){
//		case svMain :
//			return pmm;
//		case svRunAuto :
//			return pmm->runMenu->runAutoMenu;
//		case svConfig_ServoDirection :
//			return pmm->configMenu->servoMenu->dirMenu;
//		case svConfig_ServoMin :
//			return pmm->configMenu->servoMenu->minMenu;
//		case svConfig_ServoMax:
//			return pmm->configMenu->servoMenu->maxMenu;
//	}
//	return pmm;
//}

MenuItem::MenuItem(MenuItem* parent, int state, String s,MenuItemCallback cb){
	this->parent = parent;
	Caption = s;
	callBack = cb;
	mappedState = state;
}

MenuItem::MenuItem(MenuItem* parent, int state):parent(parent){
	callBack = NULL;
	mappedState = state;
}

 void MenuItem::OnPress(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	if(callBack!=NULL){
		Serial.println(F("Callback"));
		callBack();
	}
}

UpMenu::UpMenu(MenuItem* parent,int state):MenuItem(parent,-1){
	Caption = "Up";
	upState=state;
}

void UpMenu::OnPress(){
	Serial.println(F(">>>>>> Press <<<<<<  "));
	Serial.print(F("Up to "));Serial.println(upState);
	pidState.SetState((PidStateValue)upState);
}



ServoConfigMenu::ServoConfigMenu(ConfigMenu* parent):MenuItem(parent,svServo_Config){
	Caption = "Servo";
	subMenuItems = new MenuItemPtr[4];
	subMenuItemsLen = 4;
	subMenuItems[0] = new UpMenu(this,svConfig);
	dirMenu = new MenuItem(this,svConfig_ServoDirection,F("Dir"),[](){
		pidState.SetState(svConfig_ServoDirection);
	});
	minMenu =  new MenuItem(this,svConfig_ServoMin,F("Min"),[](){
		if(pidState.getState() == svServo_Config){
			pidState.SetState(svConfig_ServoMin);
		}else{
			pidState.SetState(svServo_Config);
		}
	});
	maxMenu = new MenuItem(this,svConfig_ServoMax,F("Max"),[](){
		if(pidState.getState() == svServo_Config){
			pidState.SetState(svConfig_ServoMax);
		}else{
			pidState.SetState(svServo_Config);
		}
	});
	subMenuItems[1] = dirMenu;
	subMenuItems[2] = minMenu;
	subMenuItems[3] = maxMenu;
	callBack = [](){
			pidState.SetState(svServo_Config);
		};
}

ConfigMenu::ConfigMenu(MainMenu* parent):MenuItem(parent,svConfig){
	Caption = "Config";
	upMenu = new UpMenu(this,svMain);
	tempCorrectionMenu = new MenuItem(this,svTempConfig, F("Temp Corr"),[=](){
		//pidState.setCurrentMenu(this->parent->subMenuItems[1]);
	});
	servoMenu = new ServoConfigMenu(this);

	subMenuItems = new MenuItemPtr[3];
	subMenuItemsLen = 3;
	subMenuItems[0] = upMenu;
	subMenuItems[1] = tempCorrectionMenu;
	subMenuItems[2] = servoMenu;

	callBack = [](){
		pidState.SetState(svConfig);
	};
}


RunMenu::RunMenu(MainMenu* parent):MenuItem(parent,svRun){
	Caption = "Run";
	subMenuItems = new MenuItemPtr[2];
	subMenuItemsLen = 2;
	subMenuItems[0] = new UpMenu(this,svMain);
	runAutoMenu = new MenuItem(this,svRun,F("Auto"),[](){
		if(pidState.getState() == svRun){
			pidState.SetState(svRunAuto);
		}else{
			pidState.SetState(svRun);
		}
	});
	subMenuItems[1] = runAutoMenu;

	runAutoMenu->subMenuItems = new MenuItemPtr[1];
	runAutoMenu->subMenuItemsLen = 1;
	runAutoMenu->subMenuItems[0] = new UpMenu(runAutoMenu,svRun);
	callBack = [](){
		pidState.SetState(svRun);
	};
}

MainMenu::MainMenu():MenuItem(NULL,-1){
	Caption = "Main";
	parent = NULL;
	callBack = NULL;
	subMenuItems = new MenuItemPtr[2];
	subMenuItemsLen = 2;
	subMenuItems[0] = configMenu = new ConfigMenu(this);
	subMenuItems[1] = runMenu = new RunMenu(this);
}

void InitializeMenus(){
	pidState.topMenu = new MainMenu();
	pidState.setCurrentMenu(pidState.topMenu);
}



