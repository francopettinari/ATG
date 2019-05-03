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
}
void RunAutoSetpointMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Setpoint <<<<<<  "));Serial.println(pidState.state);
	if(Selected)pidState.savetoEEprom();
	Selected=!Selected;
	Serial.println(pidState.state);
}

RunAutoRampMenu::RunAutoRampMenu():MenuItem(svRunAutoRamp){
	Caption=F("Ramp");
}
void RunAutoRampMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Ramp <<<<<<  "));Serial.println(pidState.state);
	Serial.println(pidState.state);
	if(Selected)pidState.savetoEEprom();
	Selected = !Selected;
}

RunAutoSwitch::RunAutoSwitch():MenuItem(svRunAuto){
	if(pidState.autoModeOn>0){
		Caption=F("Auto");
	}else{
		Caption=F("Manual");
	}
}

float lastSwitchPress = 0;
void RunAutoSwitch::OnSelectedInMenu(){
	float now = millis();
    bool dblClick = now-lastSwitchPress<500;

    Serial.print(F(">>>>>> Push Run Switch <<<<<<  "));Serial.print(pidState.autoModeOn);Serial.print(F(" DBLCLICK: "));Serial.print(dblClick);

	if(pidState.autoModeOn==1 && !dblClick){
		lastSwitchPress = now;
		return;
	}
	lastSwitchPress = now;
	//double pressed or single press in manual & selected
	if(dblClick){
		Selected = false;
		if(pidState.autoModeOn==1){
			pidState.autoModeOn = 0;
		}else{
			pidState.autoModeOn = 1;
			pidState.forcedOutput=0;
		}
		pidState.savetoEEprom();
	}else{
		if(pidState.autoModeOn==0){
			Selected = !Selected;
		}
	}

	if(pidState.autoModeOn>0){
		Caption=F("Auto");
	}else{
		Caption=F("Manual");
	}

}

RunAutoMenu::RunAutoMenu():MenuItem(svRunAuto){
	Caption=F("Auto");
	subMenuItems.resize(4);
	
	subMenuItems[0] = new UpMenu(svMain);
	subMenuItems[1] = switchMenu = new RunAutoSwitch();
	subMenuItems[2] = setpointMenu = new RunAutoSetpointMenu();
	subMenuItems[3] = rampMenu = new RunAutoRampMenu();

}

void RunAutoMenu::OnSelectedInMenu(){
	pidState.SetState(svRunAuto);
}

void RunAutoMenu::HandleEncoderPush(EncoderPushButtonState pst){
	MenuItem::HandleEncoderPush(pst);
}

void RunAutoMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(!rampMenu->Selected&&!setpointMenu->Selected && !(pidState.autoModeOn==0 &&switchMenu->Selected)){
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
	} else if(pidState.autoModeOn==0 && switchMenu->Selected){
		//manual mode. handle output percentage

		if(mvmnt==EncMoveCCW){
			Serial.print(F(">>>>>> Change forced output to"));Serial.println(pidState.forcedOutput+1);
			pidState.forcedOutput++;
			if(pidState.forcedOutput>100){
				pidState.forcedOutput=100;
			}
		}else if(mvmnt==EncMoveCW){
			Serial.print(F(">>>>>> Change forced output to"));Serial.println(pidState.forcedOutput-1);
			pidState.forcedOutput--;
			if(pidState.forcedOutput<0){
				pidState.forcedOutput=0;
			}
		}
		pidState.setOutPerc(pidState.forcedOutput);
	}
}

MainMenu::MainMenu():MenuItem(-1){
	Caption = F("Main");
	subMenuItems.resize(2);
	subMenuItems[0] = runMenu = new RunAutoMenu();
	subMenuItems[1] = configMenu = new ConfigMenu();
}

void MainMenu::OnSelectedInMenu(){
}

void InitializeMenus(){
	pidState.topMenu = new MainMenu();
	pidState.setCurrentMenu(pidState.topMenu);
}



