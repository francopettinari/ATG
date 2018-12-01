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

void MenuItem::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
}

void MenuItem::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		pidState.stateSelection--;
		if(pidState.stateSelection<0) pidState.stateSelection=0;
		if(pidState.stateSelection<pidState.currMenuStart)pidState.currMenuStart--;
	}else if(mvmnt==EncMoveCW){
		pidState.stateSelection++;
		if(pidState.stateSelection>pidState.currentMenu->subMenuItemsLen()-1) pidState.stateSelection=pidState.currentMenu->subMenuItemsLen()-1;
		if(pidState.stateSelection-pidState.currMenuStart>=3)pidState.currMenuStart++;
	}
}

void MenuItem::HandleEncoderPush(EncoderPushButtonState pst){
	Serial.print(F(">>>>>> Push <<<<<<  "));Serial.println(pidState.stateSelection);
	Serial.print(F("Menu    :"));Serial.println(pidState.getCurrentMenu()->Caption);
	Serial.print(F("Menu len:"));Serial.println(pidState.getCurrentMenu()->subMenuItemsLen());
	MenuItem* selMI = pidState.currentMenu->subMenuItems[pidState.stateSelection];
	selMI->OnSelectedInMenu();
}

 CallbackMenuItem::CallbackMenuItem(int state, String s,SelectedInMenuCallback cb):MenuItem(state,s){
 	Caption = s;
 	callBack=cb;
 }

 void CallbackMenuItem::OnSelectedInMenu(){
 	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
 	if(callBack!=NULL){
 		Serial.println(F("Callback"));
 		callBack();
 	}
}

void CallbackMenuItem::HandleEncoderMovement(EncoderMovement mvmnt){
	if(encoderMovementCallBack!=NULL){
		Serial.println(F("EncoderMovement callBack"));
		encoderMovementCallBack(mvmnt);
	}
}
void CallbackMenuItem::HandleEncoderPush(EncoderPushButtonState pst){
	if(encoderPushCallback!=NULL){
		Serial.println(F("EncoderPush callBack"));
		encoderPushCallback(pst);
	}
}

UpMenu::UpMenu(int state):MenuItem(-1){
	Caption = F("Up");
	upState=state;
}

void UpMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	Serial.print(F("Up to "));Serial.println(upState);
	pidState.SetState((PidStateValue)upState);
}

ServoConfigDirMenu::ServoConfigDirMenu():MenuItem(svConfig_ServoDirection){
	Caption = F("Dir");
}
void ServoConfigDirMenu::OnSelectedInMenu(){
	pidState.SetState(svConfig_ServoDirection);
}
void ServoConfigDirMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		if(pidState.servoDirection==ServoDirectionCW)pidState.servoDirection=ServoDirectionCCW;
	}else if(mvmnt==EncMoveCW){
		if(pidState.servoDirection==ServoDirectionCCW)pidState.servoDirection=ServoDirectionCW;
	}
	pidState.setServoPosition(pidState.servoMinValue);
}
void ServoConfigDirMenu:: HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svServo_Config;
	pidState.saveServoDirToEEprom();
}

ServoConfigMinMenu::ServoConfigMinMenu():MenuItem(svConfig_ServoMin){
	Caption = F("Min");
}
void ServoConfigMinMenu::OnSelectedInMenu(){
	pidState.SetState(svConfig_ServoMin);
}
void ServoConfigMinMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		pidState.servoMinValue--;
		if(pidState.servoMinValue<0)pidState.servoMinValue=0;
	}else if(mvmnt==EncMoveCW){
		if(pidState.servoMinValue<pidState.servoMaxValue-1)pidState.servoMinValue++;
		if(pidState.servoMinValue>=180)pidState.servoMinValue=179;
	}
	pidState.setServoPosition(pidState.servoMinValue);
}
void ServoConfigMinMenu:: HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svServo_Config;
	pidState.savetoEEprom();
}

ServoConfigMaxMenu::ServoConfigMaxMenu():MenuItem(svConfig_ServoMax){
	Caption = F("Max");
}
void ServoConfigMaxMenu::OnSelectedInMenu(){
	pidState.SetState(svConfig_ServoMax);
}
void ServoConfigMaxMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		if(pidState.servoMaxValue>pidState.servoMinValue+1)pidState.servoMaxValue--;
		if(pidState.servoMaxValue<=0)pidState.servoMaxValue=1;
	}else if(mvmnt==EncMoveCW){
		pidState.servoMaxValue++;
		if(pidState.servoMaxValue>180)pidState.servoMaxValue=180;
	}
	pidState.setServoPosition(pidState.servoMaxValue);
}
void ServoConfigMaxMenu::HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svServo_Config;
	pidState.savetoEEprom();
}
ServoConfigMenu::ServoConfigMenu():MenuItem(svServo_Config){
	Caption = F("Servo");
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(svConfig);
	dirMenu = new ServoConfigDirMenu();
	minMenu =  new ServoConfigMinMenu();
	maxMenu = new ServoConfigMaxMenu();
	subMenuItems[1] = dirMenu;
	subMenuItems[2] = minMenu;
	subMenuItems[3] = maxMenu;
}

void ServoConfigMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	pidState.SetState(svServo_Config);
}

////KP
KpPidConfigMenu::KpPidConfigMenu():MenuItem(svPidKpiConfig){
	Caption = F("Kp");
}
void KpPidConfigMenu::OnSelectedInMenu(){
	pidState.SetState(svPidKpiConfig);
}
void KpPidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	Serial.print(F(">>>>>> Kp Enc Mov <<<<<<  "));Serial.println(pidState.state);

	int ikp = (int)pidState.kp;
	float decPart = (pidState.kp-ikp);

	if(pidState.state==svPidKpiConfig){
		if(mvmnt==EncMoveCCW){
			ikp--;
		}else if(mvmnt==EncMoveCW){
			ikp++;
		}
		pidState.kp=ikp+decPart;
	} else if(pidState.state==svPidKpdConfig){

		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			//Serial.println(F("CCW"));
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			//Serial.println(F("CW"));
			decPart+=1.0;
		}
		pidState.kp=ikp+(decPart/100.0);
	}
}
void KpPidConfigMenu::HandleEncoderPush(EncoderPushButtonState pst){
	Serial.print(F(">>>>>> Kp Push <<<<<<  "));Serial.println(pidState.state);
	if(pidState.state==svPidKpiConfig){
		pidState.SetState(svPidKpdConfig);
	} else if(pidState.state==svPidKpdConfig){
		pidState.state = svPidConfig;
		pidState.savetoEEprom();
	}
}

////KI
KiPidConfigMenu::KiPidConfigMenu():MenuItem(svPidKiiConfig){
	Caption = F("Ki");
}
void KiPidConfigMenu::OnSelectedInMenu(){
	pidState.SetState(svPidKiiConfig);
}
void KiPidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(pidState.state==svPidKiiConfig){
		int iki = (int)pidState.ki;
		float decPart = (pidState.ki-iki);
		if(mvmnt==EncMoveCCW){
			iki--;
		}else if(mvmnt==EncMoveCW){
			iki++;
		}
		pidState.ki=iki+decPart;
	} else if(pidState.state==svPidKidConfig) {
		int iki = (int)pidState.ki;
		float decPart = (pidState.ki-iki);
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			decPart+=1.0;
		}
		pidState.ki=iki+(decPart/100.0);
	}
}
void KiPidConfigMenu::HandleEncoderPush(EncoderPushButtonState pst){
	if(pidState.state==svPidKiiConfig){
		pidState.state = svPidKidConfig;
	} else if(pidState.state==svPidKidConfig) {
		pidState.state = svPidConfig;
		pidState.savetoEEprom();
	}
}

KdPidConfigMenu::KdPidConfigMenu():MenuItem(svPidKdiConfig){
	Caption = F("Kd");
}
void KdPidConfigMenu::OnSelectedInMenu(){
	pidState.SetState(svPidKdiConfig);
}
void KdPidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(pidState.state==svPidKdiConfig) {
		int ikd = (int)pidState.kd;
		float decPart = (pidState.kd-ikd);
		if(mvmnt==EncMoveCCW){
			ikd-=1.0;
		}else if(mvmnt==EncMoveCW){
			ikd+=1.0;
		}
		pidState.kd=ikd+decPart;
	} else if(pidState.state==svPidKddConfig) {
		int ikd = (int)pidState.kd;
		float decPart = (pidState.kd-ikd);
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			decPart+=1.0;
		}
		pidState.kd=ikd+(decPart/100.0);
	}
}
void KdPidConfigMenu::HandleEncoderPush(EncoderPushButtonState pst){
	if(pidState.state==svPidKdiConfig) {
		pidState.state = svPidKddConfig;
	} else if(pidState.state==svPidKddConfig) {
		pidState.state = svPidConfig;
		pidState.savetoEEprom();
	}
}

PidConfigMenu::PidConfigMenu():MenuItem(svPidConfig){
	Caption = F("PID");
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(svConfig);
	kpMenu = new KpPidConfigMenu();
	kiMenu = new KiPidConfigMenu();
	kdMenu = new KdPidConfigMenu();
	subMenuItems[1] = kpMenu;
	subMenuItems[2] = kiMenu;
	subMenuItems[3] = kdMenu;
}

void PidConfigMenu::OnSelectedInMenu(){
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

void ConfigMenu::OnSelectedInMenu(){
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

void AutoTuneResultMenu::OnSelectedInMenu(){
//at moment not needed because this menu is displayed programmatically after AutoTune is finished
}

RunAutoTuneMenu::RunAutoTuneMenu():MenuItem(svRunAutoTune){
	Caption=F("Auto Tune");
	autoTuneResultMenu = new AutoTuneResultMenu();
}

void RunAutoTuneMenu::OnSelectedInMenu(){
	if(pidState.getState() == svRun){
		pidState.SetState(svRunAutoTune);
	}else{
		pidState.SetState(svRun);
	}
}

void RunAutoTuneMenu::HandleEncoderPush(EncoderPushButtonState pst){
	pidState.aTune.Cancel();
	pidState.SetState(svRun);
}

RunAutoSetpointMenu::RunAutoSetpointMenu():MenuItem(svRunAutoSetpoint){
	Caption=F("Setpoint");
	subMenuItems.resize(1);
	subMenuItems[0] = new UpMenu(svRunAuto);
}
void RunAutoSetpointMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		pidState.Setpoint -= 1;
		if(pidState.Setpoint<0)pidState.Setpoint=0;
//				saveSetPointTotoEEprom();
	}else if(mvmnt==EncMoveCW){
		pidState.Setpoint+=1;
		if(pidState.Setpoint>=120)pidState.Setpoint=120;
//				saveSetPointTotoEEprom();
	}
}
void RunAutoSetpointMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Setpoint <<<<<<  "));Serial.println(pidState.state);
//	if(pidState.getState() == svRunAuto){
		pidState.SetState(svRunAutoSetpoint,true);
		Serial.println(pidState.state);
//	}else{
//		pidState.SetState(svRunAuto);
//	}
}

RunAutoTimerMenu::RunAutoTimerMenu():MenuItem(svRunAutoTimer){
	Caption=F("Timer");
	subMenuItems.resize(1);
	subMenuItems[0] = new UpMenu(svRunAuto);
}

void RunAutoTimerMenu::OnSelectedInMenu(){
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

void RunAutoMenu::OnSelectedInMenu(){
	pidState.SetState(svRunAuto);
}

void RunAutoMenu::HandleEncoderPush(EncoderPushButtonState pst){
	MenuItem::HandleEncoderPush(pst);
//	pidState.aTune.Cancel();
//	pidState.SetState(svRun);
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

void RunMenu::OnSelectedInMenu(){
	pidState.SetState(svRun);
}

MainMenu::MainMenu():MenuItem(-1){
	Caption = F("Main");
	subMenuItems.resize(2);
	subMenuItems[0] = configMenu = new ConfigMenu();
	subMenuItems[1] = runMenu = new RunMenu();
}

void MainMenu::OnSelectedInMenu(){
}

void InitializeMenus(){
	pidState.topMenu = new MainMenu();
	pidState.setCurrentMenu(pidState.topMenu);
}



