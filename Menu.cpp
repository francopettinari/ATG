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

MenuItem::MenuItem(MenuItem* parent, int state, String s){
	this->parent = parent;
	Caption = s;
	mappedState = state;
}

MenuItem::MenuItem(MenuItem* parent, int state){
	this->parent = parent;
	mappedState = state;
}

 void MenuItem::OnPress(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
}

 CallbackMenuItem::CallbackMenuItem(MenuItem* parent, int state, String s,MenuItemCallback cb):MenuItem(parent,state,s){
 	Caption = s;
 	callBack=cb;
 }

 void CallbackMenuItem::OnPress(){
 	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
 	if(callBack!=NULL){
 		Serial.println(F("Callback"));
 		callBack();
 	}
}

UpMenu::UpMenu(MenuItem* parent,int state):MenuItem(parent,-1){
	Caption = F("Up");
	upState=state;
}

void UpMenu::OnPress(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	Serial.print(F("Up to "));Serial.println(upState);
	pidState.SetState((PidStateValue)upState);
}



ServoConfigMenu::ServoConfigMenu(ConfigMenu* parent):MenuItem(parent,svServo_Config){
	Caption = F("Servo");
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(this,svConfig);
	dirMenu = new CallbackMenuItem(this,svConfig_ServoDirection,F("Dir"),[](){
		pidState.SetState(svConfig_ServoDirection);
	});
	minMenu =  new CallbackMenuItem(this,svConfig_ServoMin,F("Min"),[](){
		if(pidState.getState() == svServo_Config){
			pidState.SetState(svConfig_ServoMin);
		}else{
			pidState.SetState(svServo_Config);
		}
	});
	maxMenu = new CallbackMenuItem(this,svConfig_ServoMax,F("Max"),[](){
		if(pidState.getState() == svServo_Config){
			pidState.SetState(svConfig_ServoMax);
		}else{
			pidState.SetState(svServo_Config);
		}
	});
	subMenuItems[1] = dirMenu;
	subMenuItems[2] = minMenu;
	subMenuItems[3] = maxMenu;
}

void ServoConfigMenu::OnPress(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	pidState.SetState(svServo_Config);
}


PidConfigMenu::PidConfigMenu(ConfigMenu* parent):MenuItem(parent,svPidConfig){
	Caption = F("PID");
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(this,svConfig);
	kpMenu = new CallbackMenuItem(this,svPidKpiConfig,F("Kp"),[](){
		pidState.SetState(svPidKpiConfig);
	});
	kiMenu =  new CallbackMenuItem(this,svPidKiiConfig,F("Ki"),[](){
		pidState.SetState(svPidKiiConfig);
	});
	kdMenu = new CallbackMenuItem(this,svPidKdiConfig,F("Kd"),[](){
		pidState.SetState(svPidKdiConfig);
	});
	subMenuItems[1] = kpMenu;
	subMenuItems[2] = kiMenu;
	subMenuItems[3] = kdMenu;
}

void PidConfigMenu::OnPress(){
	pidState.SetState(svPidConfig);
}

ConfigMenu::ConfigMenu(MainMenu* parent):MenuItem(parent,svConfig){
	Caption = F("Config");
	upMenu = new UpMenu(this,svMain);
	servoMenu = new ServoConfigMenu(this);
    pidMenu = new PidConfigMenu(this);
	subMenuItems.resize(3);
	subMenuItems[0] = upMenu;
	subMenuItems[1] = servoMenu;
	subMenuItems[2] = pidMenu;
}

void ConfigMenu::OnPress(){
	pidState.SetState(svConfig);
}


AutoTuneResultMenu::AutoTuneResultMenu(RunMenu* parent):MenuItem(parent,svRunAutoTuneResult){
	Caption = F("Tune res.");
	subMenuItems.resize(2);

	confirmNSaveMenu = new CallbackMenuItem(this,svRun,F("Yes"),[](){
		pidState.ConfirmAutoTuneResult();
		pidState.SetState(svRunAuto);
	});
	discardMenu = new CallbackMenuItem(this,svRun,F("No"),[](){
		pidState.SetState(svRunAuto);
	});

	subMenuItems[0] = confirmNSaveMenu;
	subMenuItems[1] = discardMenu;
}

void AutoTuneResultMenu::OnPress(){
//at moment not needed because this menu is displayed programmatically after AutoTune is finished
}

RunAutoTuneMenu::RunAutoTuneMenu(RunMenu* runMenu):MenuItem(parent,svRunAutoTune){
	Caption=F("Auto Tune");
	autoTuneResultMenu = new AutoTuneResultMenu(runMenu);
}

void RunAutoTuneMenu::OnPress(){
	if(pidState.getState() == svRun){
		pidState.SetState(svRunAutoTune);
	}else{
		pidState.SetState(svRun);
	}
}

RunMenu::RunMenu(MainMenu* parent):MenuItem(parent,svRun){
	Caption = F("Run");
	subMenuItems.resize(3);

	runAutoMenu = new CallbackMenuItem(this,svRun,F("Auto"),[](){
		if(pidState.getState() == svRun){
			pidState.SetState(svRunAuto);
		}else{
			pidState.SetState(svRun);
		}
	});

	runAutoMenu->subMenuItems.resize(1);
	runAutoTuneMenu = new RunAutoTuneMenu(this);

	subMenuItems[0] = new UpMenu(this,svMain);
	subMenuItems[1] = runAutoMenu;
	subMenuItems[2] = runAutoTuneMenu;
}

void RunMenu::OnPress(){
	pidState.SetState(svRun);
}

MainMenu::MainMenu():MenuItem(NULL,-1){
	Caption = F("Main");
	parent = NULL;
	subMenuItems.resize(2);
	subMenuItems[0] = configMenu = new ConfigMenu(this);
	subMenuItems[1] = runMenu = new RunMenu(this);
}

void MainMenu::OnPress(){
}

void InitializeMenus(){
	pidState.topMenu = new MainMenu();
	pidState.setCurrentMenu(pidState.topMenu);
}



