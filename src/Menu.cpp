#include "Menu.h"
#include "LCDHelper.h"

#include "ATG.h"
#include "Controller.h"


MenuItem::MenuItem(MenuItem* parent,int state, String s){
	Caption = s;
	mappedState = state;
	this->parent = parent;
}

//MenuItem::MenuItem(MenuItem* parent,int state){
//	mappedState = state;
//	this->parent = parent;
//}

void MenuItem::OnSelectedInMenu(){
}

void MenuItem::OnLongPress(){
}

void MenuItem::HandleEncoderMovement(EncoderMovement mvmnt){
	int sel = atg.stateSelection;
	std::vector<MenuItem *> subMenuItems = atg.getCurrentMenu()->subMenuItems;
	int max = subMenuItems.size()-1;

	if(mvmnt==EncMoveCCW){
		if(atg.stateSelection==0) return;//not possible to go down
		do{
			sel--;
		} while(sel>=0 && !subMenuItems[sel]->Visible);
	}else if(mvmnt==EncMoveCW){
		if(atg.stateSelection>=max) return;//not possible to go up
		do{
			sel++;
		} while(sel<=max && !subMenuItems[sel]->Visible);
	}
	if(sel<0) sel=0;
	if(sel>max) sel=max;
	if(!subMenuItems[sel]->Visible)return;//no visible menus available
	atg.stateSelection=sel;

	//check if currMenuStart has to be decremented
	if(sel<atg.currMenuStart)atg.currMenuStart=sel;

	//check if currMenuStart has to be incremented
	int nOfVisible = 0;
	for(int i=sel;i>=atg.currMenuStart;i--){
		if(subMenuItems[i]->Visible)nOfVisible++;
		if(nOfVisible>=3){
			atg.currMenuStart=i;
			break;
		}
	}
}

void MenuItem::HandleEncoderPush(EncoderSwStates pst){
	MenuItem* selMI = atg.currentMenu->subMenuItems[atg.stateSelection];
	switch(pst){
	case EncoderPressPressed:
		selMI->OnSelectedInMenu();
		break;
	case EncoderPressLongPressed:
		selMI->OnLongPress();
		break;
	case EncoderPressNone:
		break;
	}
}

 CallbackMenuItem::CallbackMenuItem(MenuItem* parent,int state, String s,SelectedInMenuCallback cb):MenuItem(parent,state,s){
 	Caption = s;
 	callBack=cb;
 }

 void CallbackMenuItem::OnSelectedInMenu(){
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
void CallbackMenuItem::HandleEncoderPush(EncoderSwStates pst){
	if(encoderPushCallback!=NULL){
		Serial.println(F("EncoderPush callBack"));
		encoderPushCallback(pst);
	}
}

UpMenu::UpMenu(MenuItem* parent,int state, bool saveOnUp):MenuItem(parent,-1,F("Up")){
	upState=state;
	this->saveOnUp=saveOnUp;
}

void UpMenu::OnSelectedInMenu(){
	Serial.print(F("Up to "));Serial.println(upState);
	atg.SetState((PidStateValue)upState, saveOnUp);
}

//////////////   ServoConfigDirMenu
ServoConfigDirMenu::ServoConfigDirMenu(MenuItem* parent):MenuItem(parent,svConfig_ServoDirection,F("Dir")){
}
void ServoConfigDirMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_ServoDirection);
}
void ServoConfigDirMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		if(atg.getSelectedController()->servoDirection==ServoDirectionCW)atg.getSelectedController()->setServoDirection(ServoDirectionCCW);
	}else if(mvmnt==EncMoveCW){
		if(atg.getSelectedController()->servoDirection==ServoDirectionCCW)atg.getSelectedController()->setServoDirection(ServoDirectionCW);
	}
}
void ServoConfigDirMenu:: HandleEncoderPush(EncoderSwStates pst){
	atg.state = svServo_Config;
	atg.savetoEEprom();
}
//////////////   ServoConfigDirMenu

//////////////   ServoConfigMinMenu
ServoConfigMinMenu::ServoConfigMinMenu(MenuItem* parent):MenuItem(parent,svConfig_ServoMin,F("Min")){
}
void ServoConfigMinMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_ServoMin);
}
void ServoConfigMinMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.getSelectedController()->decServoMinValue();
	}else if(mvmnt==EncMoveCW){
		atg.getSelectedController()->incServoMinValue();
	}
}
void ServoConfigMinMenu:: HandleEncoderPush(EncoderSwStates pst){
	atg.state = svServo_Config;
	atg.savetoEEprom();
}
//////////////   ServoConfigMinMenu

//////////////   ServoConfigMaxMenu
ServoConfigMaxMenu::ServoConfigMaxMenu(MenuItem* parent):MenuItem(parent,svConfig_ServoMax,F("Max")){
}
void ServoConfigMaxMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_ServoMax); //save automatically done
}
void ServoConfigMaxMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.getSelectedController()->decServoMaxValue();
	}else if(mvmnt==EncMoveCW){
		atg.getSelectedController()->incServoMaxValue();
	}
}
void ServoConfigMaxMenu::HandleEncoderPush(EncoderSwStates pst){
	atg.state = svServo_Config;
	atg.savetoEEprom();
}
//////////////   ServoConfigMaxMenu

ServoConfigMenu::ServoConfigMenu(MenuItem* parent):MenuItem(parent,svServo_Config,F("Servo")){
	subMenuItems.resize(4);
	subMenuItems[0] = new UpMenu(this,svConfig);
	dirMenu = new ServoConfigDirMenu(this);
	minMenu =  new ServoConfigMinMenu(this);
	maxMenu = new ServoConfigMaxMenu(this);
	subMenuItems[1] = dirMenu;
	subMenuItems[2] = minMenu;
	subMenuItems[3] = maxMenu;
}

void ServoConfigMenu::OnSelectedInMenu(){
	atg.SetState(svServo_Config);
}

////KP
KpPidConfigMenu::KpPidConfigMenu(MenuItem* parent):MenuItem(parent,svPidKpiConfig,F("Kp")){
}
void KpPidConfigMenu::OnSelectedInMenu(){
	atg.SetState(svPidKpiConfig);
}
void KpPidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	Serial.print(F(">>>>>> Kp Enc Mov <<<<<<  "));Serial.println(atg.state);

	int ikp = (int)atg.getSelectedController()->GetKp();
	float decPart = (atg.getSelectedController()->GetKp()-ikp);

	if(atg.state==svPidKpiConfig){
		if(mvmnt==EncMoveCCW){
			ikp--;
		}else if(mvmnt==EncMoveCW){
			ikp++;
		}
		atg.getSelectedController()->SetKp(ikp+decPart);
	} else if(atg.state==svPidKpdConfig){
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			//Serial.println(F("CCW"));
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			//Serial.println(F("CW"));
			decPart+=1.0;
		}
		atg.getSelectedController()->SetKp(ikp+(decPart/100.0));
	} else if(atg.state==svPidSampleTimeConfig){
		if(mvmnt==EncMoveCCW){
			atg.getSelectedController()->decSampleTimeSecs();
		}else if(mvmnt==EncMoveCW){
			atg.getSelectedController()->incSampleTimeSecs();
		}
	}
}
void KpPidConfigMenu::HandleEncoderPush(EncoderSwStates pst){
	Serial.print(F(">>>>>> Kp Push <<<<<<  "));Serial.println(atg.state);
	if(atg.state==svPidKpiConfig){
		atg.SetState(svPidKpdConfig);
	} else if(atg.state==svPidKpdConfig){
		atg.state = svPidConfig;
		atg.savetoEEprom();
	}
}

////KI
KiPidConfigMenu::KiPidConfigMenu(MenuItem* parent):MenuItem(parent,svPidKiiConfig,F("Ki")){
}
void KiPidConfigMenu::OnSelectedInMenu(){
	atg.SetState(svPidKiiConfig);
}
void KiPidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(atg.state==svPidKiiConfig){
		int iki = (int)atg.getSelectedController()->GetKi();
		float decPart = (atg.getSelectedController()->GetKi()-iki);
		if(mvmnt==EncMoveCCW){
			iki--;
		}else if(mvmnt==EncMoveCW){
			iki++;
		}
		atg.getSelectedController()->SetKi(iki+decPart);
	} else if(atg.state==svPidKidConfig) {
		int iki = (int)atg.getSelectedController()->GetKi();
		float decPart = (atg.getSelectedController()->GetKi()-iki);
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			decPart+=1.0;
		}
		atg.getSelectedController()->SetKi(iki+(decPart/100.0));
	}else if(atg.state==svPidKicConfig) {
		int iki = (int)atg.getSelectedController()->GetKi();
		float millPart = (atg.getSelectedController()->GetKi()-iki);
		millPart = (int)(millPart*1000);
		if(mvmnt==EncMoveCCW){
			millPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			millPart+=1.0;
		}
		atg.getSelectedController()->SetKi(iki+(millPart/1000.0));
	}
}
void KiPidConfigMenu::HandleEncoderPush(EncoderSwStates pst){
	if(atg.state==svPidKiiConfig){
		atg.state = svPidKidConfig;
	} else if(atg.state==svPidKidConfig) {
		atg.state = svPidKicConfig;
	}else if(atg.state==svPidKicConfig) {
		atg.state = svPidConfig;
		atg.savetoEEprom();
	}
}

//Kd
KdPidConfigMenu::KdPidConfigMenu(MenuItem* parent):MenuItem(parent,svPidKdiConfig,F("Kd")){
}
void KdPidConfigMenu::OnSelectedInMenu(){
	atg.SetState(svPidKdiConfig);
}
void KdPidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(atg.state==svPidKdiConfig) {
		int ikd = (int)atg.getSelectedController()->GetKd();
		float decPart = (atg.getSelectedController()->GetKd()-ikd);
		if(mvmnt==EncMoveCCW){
			ikd-=1.0;
		}else if(mvmnt==EncMoveCW){
			ikd+=1.0;
		}
		atg.getSelectedController()->SetKd(ikd+decPart);
	} else if(atg.state==svPidKddConfig) {
		int ikd = (int)atg.getSelectedController()->GetKd();
		float decPart = (atg.getSelectedController()->GetKd()-ikd);
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			decPart+=1.0;
		}
		atg.getSelectedController()->SetKd(ikd+(decPart/100.0));
	}
}
void KdPidConfigMenu::HandleEncoderPush(EncoderSwStates pst){
	if(atg.state==svPidKdiConfig) {
		atg.state = svPidKddConfig;
	} else if(atg.state==svPidKddConfig) {
		atg.state = svPidConfig;
		atg.savetoEEprom();
	}
}

//Sample time
SampleTimePidConfigMenu::SampleTimePidConfigMenu(MenuItem* parent):MenuItem(parent,svPidSampleTimeConfig,F("ST")){
}
void SampleTimePidConfigMenu::OnSelectedInMenu(){
	atg.SetState(svPidSampleTimeConfig);
}
void SampleTimePidConfigMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.getSelectedController()->decSampleTimeSecs();
	}else if(mvmnt==EncMoveCW){
		atg.getSelectedController()->incSampleTimeSecs();
	}
}
void SampleTimePidConfigMenu::HandleEncoderPush(EncoderSwStates pst){
	atg.state = svPidConfig;
	atg.savetoEEprom();
}

//PID Config
PidConfigMenu::PidConfigMenu(MenuItem* parent):MenuItem(parent,svPidConfig,F("PID")){
	subMenuItems.resize(5);
	subMenuItems[0] = new UpMenu(this,svConfig);
	subMenuItems[1] = kpMenu = new KpPidConfigMenu(this);
	subMenuItems[2] = kiMenu = new KiPidConfigMenu(this);
	subMenuItems[3] = kdMenu = new KdPidConfigMenu(this);
	subMenuItems[4] = sampleTimeMenu = new SampleTimePidConfigMenu(this);
}

void PidConfigMenu::OnSelectedInMenu(){
	atg.SetState(svPidConfig);
}

ProbeCorrectionMenu::ProbeCorrectionMenu(MenuItem* parent):MenuItem(parent,svConfig_ProbeCorrection, F("Temp correction")){
}
void ProbeCorrectionMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_ProbeCorrection);
}
void ProbeCorrectionMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.getSelectedController()->decTempCorrection();
	}else if(mvmnt==EncMoveCW){
		atg.getSelectedController()->incTempCorrection();
	}
}
void ProbeCorrectionMenu::HandleEncoderPush(EncoderSwStates pst){
	atg.SetState(svConfigController, true);
}

ControllerSelection::ControllerSelection(MenuItem* parent):MenuItem(parent,svConfigController,F("Ctrl sel")){
	subMenuItems.resize(0);
}

void ControllerSelection::OnSelectedInMenu(){
	atg.SetState(svConfigController,false);
}

void ControllerSelection::HandleEncoderPush(EncoderSwStates pst){
	atg.SetState(svConfig,false);
}

void ControllerSelection::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.setSelectedController(atg.getSelectedControllerIdx()-1);
	}else if(mvmnt==EncMoveCW){
		atg.setSelectedController(atg.getSelectedControllerIdx()+1);
	}

}

ConfigMenu::ConfigMenu(MenuItem* parent):MenuItem(parent,svConfig,F("Config")){
	subMenuItems.resize(5);
	subMenuItems[0] = upMenu = new UpMenu(this,svMain);
	subMenuItems[1] = configControllerMenu = new ControllerSelection(this);
	subMenuItems[2] = servoMenu = new ServoConfigMenu(this);
	subMenuItems[3] = pidMenu = new PidConfigMenu(this);
	subMenuItems[4] = correctionMenu = new ProbeCorrectionMenu(this);
}

void ConfigMenu::OnSelectedInMenu(){
	atg.SetState(svConfig,false);
}

RunAutoSetpointMenu::RunAutoSetpointMenu(MenuItem* parent,int state, int ctrlIdx):RunAutoBaseMenu(parent,state,F("Setpoint"),ctrlIdx){
}
void RunAutoSetpointMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Setpoint <<<<<<  "));Serial.println(atg.state);
	if(Selected)atg.savetoEEprom();
	Selected=!Selected;
	Serial.println(atg.state);
}

RunAutoRampMenu::RunAutoRampMenu(MenuItem* parent,int state, int ctrlIdx):RunAutoBaseMenu(parent,state,F("Ramp"), ctrlIdx){
}
void RunAutoRampMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Ramp <<<<<<  "));Serial.println(atg.state);
	if(Selected)atg.savetoEEprom();
	Selected = !Selected;
}

RunAutoSwitch::RunAutoSwitch(MenuItem* parent, int ctrlIdx):RunAutoBaseMenu(parent,svRunAuto,F(""),ctrlIdx){
	Controller* pCtrl = atg.getController(CtrlIdx);
	if(pCtrl->autoModeOn>0){
		Caption=F("Auto");
	}else{
		Caption=F("Manual");
	}
}

void RunAutoSwitch::OnLongPress(){
	Controller* pCtrl = atg.getController(CtrlIdx);
    Serial.print(F(">>>>>> Auto Switch long press <<<<<<  "));Serial.println(pCtrl->autoModeOn);
    pCtrl->toggleAutoModeOn();
	if(pCtrl->autoModeOn>0){
		Caption=F("Auto");
	}else{
		Caption=F("Manual");
	}
}

void RunAutoSwitch::OnSelectedInMenu(){
	Controller* pCtrl = atg.getController(CtrlIdx);
	if (!pCtrl->autoModeOn){
		Serial.print(F(">>>>>> Select Auto <<<<<<  "));Serial.println(atg.state);
		Serial.println(atg.state);
		if(Selected)atg.savetoEEprom();
		Selected = !Selected;
	}
}

RunAutoMenu::RunAutoMenu(MenuItem* parent):MenuItem(parent,svRunAuto,F("Auto")){
	subMenuItems.resize(7);
	
	subMenuItems[0] = new UpMenu(this,svMain,false);

	subMenuItems[1] = setpointMenu0 = new RunAutoSetpointMenu(this,svRunAutoSetpoint0,0);
	subMenuItems[2] = switchMenu0 = new RunAutoSwitch(this,0);
	subMenuItems[3] = rampMenu0 = new RunAutoRampMenu(this,svRunAutoRamp0,0);

	subMenuItems[4] = setpointMenu1 = new RunAutoSetpointMenu(this,svRunAutoSetpoint1,1);
	subMenuItems[5] = switchMenu1 = new RunAutoSwitch(this,1);
	subMenuItems[6] = rampMenu1 = new RunAutoRampMenu(this,svRunAutoRamp1,1);

}

void RunAutoMenu::OnSelectedInMenu(){
	atg.SetState(svRunAuto,false);
}

void RunAutoMenu::HandleEncoderPush(EncoderSwStates pst){
	MenuItem::HandleEncoderPush(pst);
}

void RunAutoMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	Controller* pCtrl0 = atg.getController(0);
	Controller* pCtrl1 = atg.getController(1);
	if(!rampMenu0->Selected&&!setpointMenu0->Selected && !(pCtrl0->autoModeOn==0 &&switchMenu0->Selected) &&
	   !rampMenu1->Selected&&!setpointMenu1->Selected && !(pCtrl1->autoModeOn==0 &&switchMenu1->Selected)){
		MenuItem::HandleEncoderMovement(mvmnt);
	}

	//last resort status fix
	if(pCtrl0->autoModeOn==1) switchMenu0->Selected = false;
	if(pCtrl1->autoModeOn==1) switchMenu1->Selected = false;

	if(rampMenu0->Selected){
		if(mvmnt==EncMoveCCW){
			pCtrl0->decRamp();
		}else if(mvmnt==EncMoveCW){
			pCtrl0->incRamp();
		}
	}if(rampMenu1->Selected){
		if(mvmnt==EncMoveCCW){
			pCtrl1->decRamp();
		}else if(mvmnt==EncMoveCW){
			pCtrl1->incRamp();
		}
	}else if(setpointMenu0->Selected){
		if(mvmnt==EncMoveCCW){
			pCtrl0->decSetpoint();
		}else if(mvmnt==EncMoveCW){
			pCtrl0->incSetpoint();
		}
	} else if(setpointMenu1->Selected){
		if(mvmnt==EncMoveCCW){
			pCtrl1->decSetpoint();
		}else if(mvmnt==EncMoveCW){
			pCtrl1->incSetpoint();
		}
	} else if(pCtrl0->autoModeOn==0 && switchMenu0->Selected){
		//manual mode. handle output percentage
		if(mvmnt==EncMoveCCW){
			Serial.print(F(">>>>>> Change forced output 0 to"));Serial.println(pCtrl0->forcedOutput-10);
			pCtrl0->setForcedOutput(pCtrl0->forcedOutput-10);
		}else if(mvmnt==EncMoveCW){
			Serial.print(F(">>>>>> Change forced output 0 to"));Serial.println(pCtrl0->forcedOutput+10);
			pCtrl0->setForcedOutput(pCtrl0->forcedOutput+10);
		}
		pCtrl0->setOutPerc(pCtrl0->forcedOutput);
	} else if(pCtrl1->autoModeOn==0 && switchMenu1->Selected){
		//manual mode. handle output percentage
		if(mvmnt==EncMoveCCW){
			Serial.print(F(">>>>>> Change forced output 1 to"));Serial.println(pCtrl1->forcedOutput-10);
			pCtrl1->setForcedOutput(pCtrl1->forcedOutput-10);
		}else if(mvmnt==EncMoveCW){
			Serial.print(F(">>>>>> Change forced output 1 to"));Serial.println(pCtrl1->forcedOutput+10);
			pCtrl1->setForcedOutput(pCtrl1->forcedOutput+10);
		}
		pCtrl1->setOutPerc(pCtrl1->forcedOutput);
	}
}

MainMenu::MainMenu():MenuItem(NULL,-1,F("Main")){
	subMenuItems.resize(2);
	subMenuItems[0] = runMenu = new RunAutoMenu(this);
	subMenuItems[1] = configMenu = new ConfigMenu(this);
}

void MainMenu::OnSelectedInMenu(){
}



