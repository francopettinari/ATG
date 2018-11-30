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

MenuItem::MenuItem(int state, String s){
	Caption = s;
	mappedState = state;
}

MenuItem::MenuItem(int state){
	mappedState = state;
}

 void MenuItem::OnPress(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
}

 CallbackMenuItem::CallbackMenuItem(int state, String s,MenuItemCallback cb):MenuItem(state,s){
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

UpMenu::UpMenu(int state):MenuItem(-1){
	Caption = F("Up");
	upState=state;
}

void UpMenu::OnPress(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	Serial.print(F("Up to "));Serial.println(upState);
	pidState.SetState((PidStateValue)upState);
}



ServoConfigMenu::ServoConfigMenu():MenuItem(svServo_Config){
	Caption = F("Servo");
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(svConfig);
	dirMenu = new CallbackMenuItem(svConfig_ServoDirection,F("Dir"),[](){
		pidState.SetState(svConfig_ServoDirection);
	});
	minMenu =  new CallbackMenuItem(svConfig_ServoMin,F("Min"),[](){
		if(pidState.getState() == svServo_Config){
			pidState.SetState(svConfig_ServoMin);
		}else{
			pidState.SetState(svServo_Config);
		}
	});
	maxMenu = new CallbackMenuItem(svConfig_ServoMax,F("Max"),[](){
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


PidConfigMenu::PidConfigMenu():MenuItem(svPidConfig){
	Caption = F("PID");
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(svConfig);
	kpMenu = new CallbackMenuItem(svPidKpiConfig,F("Kp"),[](){
		pidState.SetState(svPidKpiConfig);
	});
	kiMenu =  new CallbackMenuItem(svPidKiiConfig,F("Ki"),[](){
		pidState.SetState(svPidKiiConfig);
	});
	kdMenu = new CallbackMenuItem(svPidKdiConfig,F("Kd"),[](){
		pidState.SetState(svPidKdiConfig);
	});
	subMenuItems[1] = kpMenu;
	subMenuItems[2] = kiMenu;
	subMenuItems[3] = kdMenu;
}

void PidConfigMenu::OnPress(){
	pidState.SetState(svPidConfig);
}

ConfigMenu::ConfigMenu():MenuItem(svConfig){
	Caption = F("Config");
	upMenu = new UpMenu(svMain);
	servoMenu = new ServoConfigMenu();
    pidMenu = new PidConfigMenu();
	subMenuItems.resize(3);
	subMenuItems[0] = upMenu;
	subMenuItems[1] = servoMenu;
	subMenuItems[2] = pidMenu;
}

void ConfigMenu::OnPress(){
	pidState.SetState(svConfig);
}


AutoTuneResultMenu::AutoTuneResultMenu():MenuItem(svRunAutoTuneResult){
	Caption = F("Tune res.");
	subMenuItems.resize(2);

	confirmNSaveMenu = new CallbackMenuItem(svRun,F("Yes"),[](){
		pidState.ConfirmAutoTuneResult();
		pidState.SetState(svRunAuto);
	});
	discardMenu = new CallbackMenuItem(svRun,F("No"),[](){
		pidState.SetState(svRunAuto);
	});

	subMenuItems[0] = confirmNSaveMenu;
	subMenuItems[1] = discardMenu;
}

void AutoTuneResultMenu::OnPress(){
//at moment not needed because this menu is displayed programmatically after AutoTune is finished
}

RunAutoTuneMenu::RunAutoTuneMenu():MenuItem(svRunAutoTune){
	Caption=F("Auto Tune");
	autoTuneResultMenu = new AutoTuneResultMenu();
}

void RunAutoTuneMenu::OnPress(){
	if(pidState.getState() == svRun){
		pidState.SetState(svRunAutoTune);
	}else{
		pidState.SetState(svRun);
	}
}

RunAutoSetpointMenu::RunAutoSetpointMenu():MenuItem(svRunAutoSetpoint){
	Caption=F("Setpoint");
	subMenuItems.resize(1);
	subMenuItems[0] = new UpMenu(svRunAuto);
}
void RunAutoSetpointMenu::OnPress(){
//	if(pidState.getState() == svRunAuto){
		pidState.SetState(svRunAutoSetpoint,false);
//	}else{
//		pidState.SetState(svRunAuto);
//	}
}

RunAutoTimerMenu::RunAutoTimerMenu():MenuItem(svRunAutoTimer){
	Caption=F("Timer");
	subMenuItems.resize(1);
	subMenuItems[0] = new UpMenu(svRunAuto);
}

void RunAutoTimerMenu::OnPress(){
//	if(pidState.getState() == svRunAuto){
		pidState.SetState(svRunAutoTimer,false);
//	}else{
//		pidState.SetState(svRunAuto);
//	}
}

RunAutoMenu::RunAutoMenu():MenuItem(svRunAuto){
	Caption=F("Auto");
	subMenuItems.resize(3);

	setpointMenu = new RunAutoSetpointMenu();
	timerMenu = new RunAutoTimerMenu();
	
	subMenuItems[0] = new UpMenu(svRun);
	subMenuItems[1] = setpointMenu;
	subMenuItems[2] = timerMenu;
}

void RunAutoMenu::OnPress(){
	//if(pidState.getState() == svRun){
		pidState.SetState(svRunAuto);
	//}else{
	//	pidState.SetState(svRun);
	//}
}

RunMenu::RunMenu():MenuItem(svRun){
	Caption = F("Run");
	subMenuItems.resize(3);

	runAutoMenu = new RunAutoMenu();
	runAutoTuneMenu = new RunAutoTuneMenu();

	subMenuItems[0] = new UpMenu(svMain);
	subMenuItems[1] = runAutoMenu;
	subMenuItems[2] = runAutoTuneMenu;
}

void RunMenu::OnPress(){
	pidState.SetState(svRun);
}

MainMenu::MainMenu():MenuItem(-1){
	Caption = F("Main");
	subMenuItems.resize(2);
	subMenuItems[0] = configMenu = new ConfigMenu();
	subMenuItems[1] = runMenu = new RunMenu();
}

void MainMenu::OnPress(){
}

void InitializeMenus(){
	pidState.topMenu = new MainMenu();
	pidState.setCurrentMenu(pidState.topMenu);
}



