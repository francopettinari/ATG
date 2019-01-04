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

//////////////   ServoConfigDirMenu
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
	pidState.writeServoPosition(pidState.servoMinValue);
}
void ServoConfigDirMenu:: HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svServo_Config;
	pidState.saveServoDirToEEprom();
}
//////////////   ServoConfigDirMenu

//////////////   ServoConfigMinMenu
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
	pidState.writeServoPosition(pidState.servoMinValue);
}
void ServoConfigMinMenu:: HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svServo_Config;
	pidState.savetoEEprom();
}
//////////////   ServoConfigMinMenu

//////////////   ServoConfigMaxMenu
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
	pidState.writeServoPosition(pidState.servoMaxValue);
}
void ServoConfigMaxMenu::HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svServo_Config;
	pidState.savetoEEprom();
}
//////////////   ServoConfigMaxMenu

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
	} else if(pidState.state==svPidSampleTimeConfig){
		if(mvmnt==EncMoveCCW){
			pidState.pidSampleTimeSecs-=1.0;
		}else if(mvmnt==EncMoveCW){
			pidState.pidSampleTimeSecs+=1.0;
		}
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
	}else if(pidState.state==svPidKicConfig) {
		int iki = (int)pidState.ki;
		float millPart = (pidState.ki-iki);
		millPart = (int)(millPart*1000);
		if(mvmnt==EncMoveCCW){
			millPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			millPart+=1.0;
		}
		pidState.ki=iki+(millPart/1000.0);
	}
}
void KiPidConfigMenu::HandleEncoderPush(EncoderPushButtonState pst){
	if(pidState.state==svPidKiiConfig){
		pidState.state = svPidKidConfig;
	} else if(pidState.state==svPidKidConfig) {
		pidState.state = svPidKicConfig;
	}else if(pidState.state==svPidKicConfig) {
		pidState.state = svPidConfig;
		pidState.savetoEEprom();
	}
}

//Kd
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

//Sample time
SampleTimePidConfigMenu::SampleTimePidConfigMenu():MenuItem(svPidSampleTimeConfig){
	Caption = F("ST");
}
void SampleTimePidConfigMenu::OnSelectedInMenu(){
	pidState.SetState(svPidSampleTimeConfig);
}
void SampleTimePidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		pidState.pidSampleTimeSecs--;
	}else if(mvmnt==EncMoveCW){
		pidState.pidSampleTimeSecs++;
	}
}
void SampleTimePidConfigMenu::HandleEncoderPush(EncoderPushButtonState pst){
	pidState.state = svPidConfig;
	pidState.savetoEEprom();
}

//PID Config
PidConfigMenu::PidConfigMenu():MenuItem(svPidConfig){
	Caption = F("PID");
	subMenuItems.resize(5);
	subMenuItems[0] = new UpMenu(svConfig);
	subMenuItems[1] = kpMenu = new KpPidConfigMenu();
	subMenuItems[2] = kiMenu = new KiPidConfigMenu();
	subMenuItems[3] = kdMenu = new KdPidConfigMenu();
	subMenuItems[4] = sampleTimeMenu = new SampleTimePidConfigMenu();
}

void PidConfigMenu::OnSelectedInMenu(){
	pidState.SetState(svPidConfig);
}

ConfigMenu::ConfigMenu():MenuItem(svConfig){
	Caption = F("Config");

	subMenuItems.resize(3);
	subMenuItems[0] = upMenu = new UpMenu(svMain);
	subMenuItems[1] = servoMenu = new ServoConfigMenu();
	subMenuItems[2] = pidMenu = new PidConfigMenu();
}

void ConfigMenu::OnSelectedInMenu(){
	pidState.SetState(svConfig);
}


AutoTuneResultMenu::AutoTuneResultMenu():MenuItem(svRunAutoTuneResult){
	Caption = F("Tune res.");
	subMenuItems.resize(2);

	subMenuItems[0] = confirmNSaveMenu = new CallbackMenuItem(svRun,F("Yes"),[](){
		pidState.ConfirmAutoTuneResult();
		pidState.SetState(svRunAuto);
	});
	subMenuItems[1] = discardMenu = new CallbackMenuItem(svRun,F("No"),[](){
		pidState.SetState(svRunAuto);
	});

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
//	subMenuItems.resize(1);
//	subMenuItems[0] = new UpMenu(svRunAuto);
}
void RunAutoSetpointMenu::HandleEncoderMovement(EncoderMovement mvmnt){
//	if(mvmnt==EncMoveCCW){
//		pidState.Setpoint -= 1;
//		if(pidState.Setpoint<0)pidState.Setpoint=0;
//	}else if(mvmnt==EncMoveCW){
//		pidState.Setpoint+=1;
//		if(pidState.Setpoint>=120)pidState.Setpoint=120;
//	}
}
void RunAutoSetpointMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Setpoint <<<<<<  "));Serial.println(pidState.state);
//	pidState.SetState(svRunAutoSetpoint,false);
	Selected=!Selected;
	Serial.println(pidState.state);
}

RunAutoRampMenu::RunAutoRampMenu():MenuItem(svRunAutoRamp){
	Caption=F("Ramp");
//	subMenuItems.resize(1);
//	subMenuItems[0] = new UpMenu(svRunAuto);
}
void RunAutoRampMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Ramp <<<<<<  "));Serial.println(pidState.state);
//	if(!Selected){
//		pidState.SetState(svRunAutoRamp,false);
//	}else{
//		pidState.SetState(svRunAuto,false);
//	}
	Serial.println(pidState.state);
	Selected = !Selected;
}
void RunAutoRampMenu::HandleEncoderMovement(EncoderMovement mvmnt){
//	if(mvmnt==EncMoveCCW){
//		pidState.Ramp -= 1;
//		if(pidState.Ramp<0)pidState.Ramp=0;
//	}else if(mvmnt==EncMoveCW){
//		pidState.Ramp+=1;
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
	subMenuItems.resize(4);
	
	subMenuItems[0] = new UpMenu(svRun);
	subMenuItems[1] = setpointMenu = new RunAutoSetpointMenu();
	subMenuItems[2] = rampMenu = new RunAutoRampMenu();
	subMenuItems[3] = timerMenu = new RunAutoTimerMenu();
}

void RunAutoMenu::OnSelectedInMenu(){
	pidState.SetState(svRunAuto);
}

void RunAutoMenu::HandleEncoderPush(EncoderPushButtonState pst){
	MenuItem::HandleEncoderPush(pst);
//	pidState.aTune.Cancel();
//	pidState.SetState(svRun);
}

void RunAutoMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(!rampMenu->Selected&&!setpointMenu->Selected){
		MenuItem::HandleEncoderMovement(mvmnt);
	}
	if(rampMenu->Selected){
		if(mvmnt==EncMoveCCW){
			pidState.Ramp -= 1;
			if(pidState.Ramp<0)pidState.Ramp=0;
		}else if(mvmnt==EncMoveCW){
			pidState.Ramp+=1;
		}
	}else if(setpointMenu->Selected){
		if(mvmnt==EncMoveCCW){
			pidState.Setpoint -= 1;
			if(pidState.Setpoint<0)pidState.Setpoint=0;
		}else if(mvmnt==EncMoveCW){
			pidState.Setpoint+=1;
			if(pidState.Setpoint>=120)pidState.Setpoint=120;
		}
	}
}

RunManualMenu::RunManualMenu():MenuItem(svRunManual){
	Caption=F("Manual");
	subMenuItems.resize(1);
    subMenuItems[0] = new UpMenu(svRun);
}
void RunManualMenu::OnSelectedInMenu(){
	pidState.SetState(svRunManual);
	pidState.Output=pidState.servoMinValue-1;
}
void RunManualMenu::HandleEncoderPush(EncoderPushButtonState pst){
	MenuItem::HandleEncoderPush(pst);
}

void RunManualMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		pidState.Output -= 1;
//		if(pidState.Output<0)pidState.Output=0;
		if(pidState.Output<pidState.servoMinValue-1){
			pidState.Output=pidState.servoMinValue-1;
		}
	}else if(mvmnt==EncMoveCW){

		pidState.Output+=1;

		if(pidState.Output>=180)pidState.Output=180;
	}
}

RunMenu::RunMenu():MenuItem(svRun){
	Caption = F("Run");
	subMenuItems.resize(4);

	subMenuItems[0] = new UpMenu(svMain);
	subMenuItems[1] = runAutoMenu = new RunAutoMenu();
	subMenuItems[2] = runManualMenu = new RunManualMenu();
	subMenuItems[3] = runAutoTuneMenu = new RunAutoTuneMenu();
}

void RunMenu::OnSelectedInMenu(){
	pidState.SetState(svRun);
}

MainMenu::MainMenu():MenuItem(-1){
	Caption = F("Main");
	subMenuItems.resize(2);
	subMenuItems[0] = runMenu = new RunMenu();
	subMenuItems[1] = configMenu = new ConfigMenu();
}

void MainMenu::OnSelectedInMenu(){
}

void InitializeMenus(){
	pidState.topMenu = new MainMenu();
	pidState.setCurrentMenu(pidState.topMenu);
}



