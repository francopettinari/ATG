#include "Menu.h"
#include "Pid.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"


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
	int sel = pidState.stateSelection;
	std::vector<MenuItem *> subMenuItems = pidState.getCurrentMenu()->subMenuItems;
	int max = subMenuItems.size()-1;

	if(mvmnt==EncMoveCCW){
		if(pidState.stateSelection==0) return;//not possible to go down
		do{
			sel--;
		} while(sel>=0 && !subMenuItems[sel]->Visible);
	}else if(mvmnt==EncMoveCW){
		if(pidState.stateSelection>=max) return;//not possible to go up
		do{
			sel++;
		} while(sel<=max && !subMenuItems[sel]->Visible);
	}
	if(sel<0) sel=0;
	if(sel>max) sel=max;
	if(!subMenuItems[sel]->Visible)return;//no visible menus available
	pidState.stateSelection=sel;

	//check if currMenuStart has to be decremented
	if(sel<pidState.currMenuStart)pidState.currMenuStart=sel;

	//check if currMenuStart has to be incremented
	int nOfVisible = 0;
	for(int i=sel;i>=pidState.currMenuStart;i--){
		if(subMenuItems[i]->Visible)nOfVisible++;
		if(nOfVisible>=3){
			pidState.currMenuStart=i;
			break;
		}
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
	pidState.writeServoPosition(pidState.servoMinValue,false);
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
	pidState.writeServoPosition(pidState.servoMinValue,false);
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
	pidState.writeServoPosition(pidState.servoMaxValue,false);
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

RunAutoTimerMinutesMenu::RunAutoTimerMinutesMenu():MenuItem(svRunAutoTimerMinutes){
	Caption=F("Minutes");
}
void RunAutoTimerMinutesMenu::OnSelectedInMenu(){
	if(pidState.timerState==3){
		pidState.timerState=0;
		return;
	}
	pidState.SetState(svRunAutoTimerMinutes,false);
}
void RunAutoTimerMinutesMenu::HandleEncoderMovement(EncoderMovement mvmnt) {
	if(mvmnt==EncMoveCCW){
		pidState.timerValueMins -= 1;
		if(pidState.timerValueMins<0)pidState.timerValueMins=0;
	}else if(mvmnt==EncMoveCW){
		pidState.timerValueMins+=1;
	}
}

void RunAutoTimerMinutesMenu::HandleEncoderPush(EncoderPushButtonState pst){
	pidState.SetState(svRunAutoTimer);
	pidState.savetoEEprom();
	if(pidState.timerState==3){
		pidState.timerState=0;
		return;
	}
}

RunAutoTimerMenu::RunAutoTimerMenu():MenuItem(svRunAutoTimer){
	Caption=F("Timer");
	subMenuItems.resize(5);
	subMenuItems[0] = new UpMenu(svRunAuto);
	subMenuItems[1] = timerValueMenu = new RunAutoTimerMinutesMenu();
	subMenuItems[2] = new CallbackMenuItem(svRunAutoTimer,F("Start"),[](){
		pidState.StartTimer();
		pidState.SetState(svRunAutoTimer);
		((MainMenu*)pidState.topMenu)->runMenu->runAutoMenu->timerMenu->SetVisibilities();
	});
	subMenuItems[3] = new CallbackMenuItem(svRunAutoTimer,F("Pause"),[](){
		pidState.PauseTimer();
		pidState.SetState(svRunAutoTimer);
		((MainMenu*)pidState.topMenu)->runMenu->runAutoMenu->timerMenu->SetVisibilities();
	});
	subMenuItems[4] = new CallbackMenuItem(svRunAutoTimer,F("Stop"),[](){
		pidState.StopTimer();
		pidState.SetState(svRunAutoTimer);
		((MainMenu*)pidState.topMenu)->runMenu->runAutoMenu->timerMenu->SetVisibilities();
	});

	SetVisibilities();
}

void RunAutoTimerMenu::SetVisibilities(){
	subMenuItems[2]->Visible = pidState.timerState!=1 && pidState.timerValueMins>0;
	subMenuItems[3]->Visible = pidState.timerState==1;
	subMenuItems[4]->Visible = pidState.timerState==1;
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
//	MenuItem* selMI = pidState.currentMenu->subMenuItems[pidState.stateSelection];
//	if(selMI==subMenuItems[0]){
//		pidState.Output=0;
//		pidState.writeServoPosition(pidState.Output,true);
//	}
	MenuItem::HandleEncoderPush(pst);
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
	subMenuItems.resize(3);

	subMenuItems[0] = new UpMenu(svMain);
	subMenuItems[1] = runAutoMenu = new RunAutoMenu();
	subMenuItems[2] = runManualMenu = new RunManualMenu();
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



