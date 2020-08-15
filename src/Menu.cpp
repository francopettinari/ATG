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
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
}

void MenuItem::OnLongPress(){
	Serial.print(F(">>>>>> Long press <<<<<<  "));Serial.println(Caption);
}

void MenuItem::OnDoublePress(){
	Serial.print(F(">>>>>> Double press <<<<<<  "));Serial.println(Caption);
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
	Serial.print(F("Menu    :"));Serial.println(atg.getCurrentMenu()->Caption);
	Serial.print(F("Menu len:"));Serial.println(atg.getCurrentMenu()->subMenuItemsLen());
	MenuItem* selMI = atg.currentMenu->subMenuItems[atg.stateSelection];
	switch(pst){
	case EncoderPressPressed:
		Serial.print(F(">>>>>> Push <<<<<<  "));Serial.println(atg.stateSelection);
		selMI->OnSelectedInMenu();
		break;
	case EncoderPressLongPressed:
		Serial.print(F(">>>>>> Push <<<<<<  "));Serial.println(atg.stateSelection);
		selMI->OnLongPress();
		break;
	case EncoderPressDblPressed:
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
void CallbackMenuItem::HandleEncoderPush(EncoderSwStates pst){
	if(encoderPushCallback!=NULL){
		Serial.println(F("EncoderPush callBack"));
		encoderPushCallback(pst);
	}
}

UpMenu::UpMenu(MenuItem* parent,int state):MenuItem(parent,-1,F("Up")){
	upState=state;
}

void UpMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
	Serial.print(F("Up to "));Serial.println(upState);
	atg.SetState((PidStateValue)upState);
}

//////////////   ServoConfigDirMenu
ServoConfigDirMenu::ServoConfigDirMenu(MenuItem* parent):MenuItem(parent,svConfig_ServoDirection,F("Dir")){
}
void ServoConfigDirMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_ServoDirection);
}
void ServoConfigDirMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		if(atg.pidStates[atg.selectedController].servoDirection==ServoDirectionCW)atg.pidStates[atg.selectedController].servoDirection=ServoDirectionCCW;
	}else if(mvmnt==EncMoveCW){
		if(atg.pidStates[atg.selectedController].servoDirection==ServoDirectionCCW)atg.pidStates[atg.selectedController].servoDirection=ServoDirectionCW;
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
		atg.pidStates[atg.selectedController].servoMinValue--;
		if(atg.pidStates[atg.selectedController].servoMinValue<0)atg.pidStates[atg.selectedController].servoMinValue=0;
	}else if(mvmnt==EncMoveCW){
		if(atg.pidStates[atg.selectedController].servoMinValue<atg.pidStates[atg.selectedController].servoMaxValue-1)atg.pidStates[atg.selectedController].servoMinValue++;
		if(atg.pidStates[atg.selectedController].servoMinValue>=180)atg.pidStates[atg.selectedController].servoMinValue=179;
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
		if(atg.pidStates[atg.selectedController].servoMaxValue>atg.pidStates[atg.selectedController].servoMinValue+1)atg.pidStates[atg.selectedController].servoMaxValue--;
		if(atg.pidStates[atg.selectedController].servoMaxValue<=0)atg.pidStates[atg.selectedController].servoMaxValue=1;
	}else if(mvmnt==EncMoveCW){
		atg.pidStates[atg.selectedController].servoMaxValue++;
		if(atg.pidStates[atg.selectedController].servoMaxValue>180)atg.pidStates[atg.selectedController].servoMaxValue=180;
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
	Serial.print(F(">>>>>> Press <<<<<<  "));Serial.println(Caption);
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

	int ikp = (int)atg.pidStates[atg.selectedController].GetKp();
	float decPart = (atg.pidStates[atg.selectedController].GetKp()-ikp);

	if(atg.state==svPidKpiConfig){
		if(mvmnt==EncMoveCCW){
			ikp--;
		}else if(mvmnt==EncMoveCW){
			ikp++;
		}
		atg.pidStates[atg.selectedController].SetKp(ikp+decPart);
	} else if(atg.state==svPidKpdConfig){
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			//Serial.println(F("CCW"));
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			//Serial.println(F("CW"));
			decPart+=1.0;
		}
		atg.pidStates[atg.selectedController].SetKp(ikp+(decPart/100.0));
	} else if(atg.state==svPidSampleTimeConfig){
		if(mvmnt==EncMoveCCW){
			atg.pidStates[atg.selectedController].pidSampleTimeSecs-=1.0;
		}else if(mvmnt==EncMoveCW){
			atg.pidStates[atg.selectedController].pidSampleTimeSecs+=1.0;
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
		int iki = (int)atg.pidStates[atg.selectedController].GetKi();
		float decPart = (atg.pidStates[atg.selectedController].GetKi()-iki);
		if(mvmnt==EncMoveCCW){
			iki--;
		}else if(mvmnt==EncMoveCW){
			iki++;
		}
		atg.pidStates[atg.selectedController].SetKi(iki+decPart);
	} else if(atg.state==svPidKidConfig) {
		int iki = (int)atg.pidStates[atg.selectedController].GetKi();
		float decPart = (atg.pidStates[atg.selectedController].GetKi()-iki);
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			decPart+=1.0;
		}
		atg.pidStates[atg.selectedController].SetKi(iki+(decPart/100.0));
	}else if(atg.state==svPidKicConfig) {
		int iki = (int)atg.pidStates[atg.selectedController].GetKi();
		float millPart = (atg.pidStates[atg.selectedController].GetKi()-iki);
		millPart = (int)(millPart*1000);
		if(mvmnt==EncMoveCCW){
			millPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			millPart+=1.0;
		}
		atg.pidStates[atg.selectedController].SetKi(iki+(millPart/1000.0));
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
		int ikd = (int)atg.pidStates[atg.selectedController].GetKd();
		float decPart = (atg.pidStates[atg.selectedController].GetKd()-ikd);
		if(mvmnt==EncMoveCCW){
			ikd-=1.0;
		}else if(mvmnt==EncMoveCW){
			ikd+=1.0;
		}
		atg.pidStates[atg.selectedController].SetKd(ikd+decPart);
	} else if(atg.state==svPidKddConfig) {
		int ikd = (int)atg.pidStates[atg.selectedController].GetKd();
		float decPart = (atg.pidStates[atg.selectedController].GetKd()-ikd);
		decPart = (int)(decPart*100);
		if(mvmnt==EncMoveCCW){
			decPart-=1.0;
		}else if(mvmnt==EncMoveCW){
			decPart+=1.0;
		}
		atg.pidStates[atg.selectedController].SetKd(ikd+(decPart/100.0));
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
		atg.pidStates[atg.selectedController].pidSampleTimeSecs--;
	}else if(mvmnt==EncMoveCW){
		atg.pidStates[atg.selectedController].pidSampleTimeSecs++;
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
		atg.pidStates[atg.selectedController].temperatureCorrection--;
	}else if(mvmnt==EncMoveCW){
		atg.pidStates[atg.selectedController].temperatureCorrection++;
	}
}
void ProbeCorrectionMenu::HandleEncoderPush(EncoderSwStates pst){
	atg.state = svConfig_Probe;
	atg.savetoEEprom();
}



ProbeAssignmentMenu::ProbeAssignmentMenu(MenuItem* parent):MenuItem(parent,svConfig_ProbeAssign,F("Probe assignment")){
	Serial.println(F("Locating devices..."));
	Serial.print(F("Found "));
	int deviceCount = atg.pidStates[atg.selectedController].probe.getDeviceCount();
	Serial.print(deviceCount, DEC);
	Serial.println(F(" devices."));

	Serial.println("Temperatures...");
	for (int i = 0;  i < deviceCount;  i++){
		Serial.print("Sensor ");Serial.print(i);Serial.print(" : ");atg.pidStates[atg.selectedController].probe.readTemperatureByIndex(i);
//		sensors.getAddress(Thermometer, i);
//		printAddress(Thermometer);
	}

}
void ProbeAssignmentMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_ProbeAssign);
}
//void ProbeAssignmentMenu::HandleEncoderMovement(EncoderMovement mvmnt){
//	if(mvmnt==EncMoveCCW){
//		pidState.temperatureCorrection--;
//	}else if(mvmnt==EncMoveCW){
//		pidState.temperatureCorrection++;
//	}
//}

void ProbeAssignmentMenu::HandleEncoderPush(EncoderSwStates pst){
	atg.state = svConfig_Probe;
	atg.savetoEEprom();
}



ConfigProbeMenu::ConfigProbeMenu(MenuItem* parent):MenuItem(parent,svConfig_Probe,F("Probe configuration")){
	subMenuItems.resize(3);
	subMenuItems[0] = upMenu = new UpMenu(this,svConfig);
	subMenuItems[1] = correctionMenu = new ProbeCorrectionMenu(this);
	subMenuItems[2] = assignmentMenu = new ProbeAssignmentMenu(this);
}
void ConfigProbeMenu::OnSelectedInMenu(){
	atg.SetState(svConfig_Probe);
}
void ConfigProbeMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.pidStates[atg.selectedController].temperatureCorrection--;
	}else if(mvmnt==EncMoveCW){
		atg.pidStates[atg.selectedController].temperatureCorrection++;
	}
}
void ConfigProbeMenu::HandleEncoderPush(EncoderSwStates pst){
	atg.state = svConfig;
	atg.savetoEEprom();
}

ConfigController::ConfigController(MenuItem* parent):MenuItem(parent,svConfigController,F("Ctrl sel")){
	subMenuItems.resize(0);
}

void ConfigController::OnSelectedInMenu(){
	atg.SetState(svConfigController,false);
}

void ConfigController::HandleEncoderPush(EncoderSwStates pst){
	atg.SetState(svConfig,false);
}

void ConfigController::HandleEncoderMovement(EncoderMovement mvmnt){
	if(mvmnt==EncMoveCCW){
		atg.selectedController ++;
	}else if(mvmnt==EncMoveCW){
		atg.selectedController --;
	}
	atg.selectedController = atg.selectedController % atg.nOfControllers;
}

ConfigMenu::ConfigMenu(MenuItem* parent):MenuItem(parent,svConfig,F("Config")){
	subMenuItems.resize(5);
	subMenuItems[0] = upMenu = new UpMenu(this,svMain);
	subMenuItems[1] = configControllerMenu = new ConfigController(this);
	subMenuItems[2] = servoMenu = new ServoConfigMenu(this);
	subMenuItems[3] = pidMenu = new PidConfigMenu(this);
	subMenuItems[4] = probeMenu = new ConfigProbeMenu(this);
}

void ConfigMenu::OnSelectedInMenu(){
	atg.SetState(svConfig,false);
}

RunAutoSetpointMenu::RunAutoSetpointMenu(MenuItem* parent):MenuItem(parent,svRunAutoSetpoint,F("Setpoint")){
}
void RunAutoSetpointMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Setpoint <<<<<<  "));Serial.println(atg.state);
	if(Selected)atg.savetoEEprom();
	Selected=!Selected;
	Serial.println(atg.state);
}

RunAutoRampMenu::RunAutoRampMenu(MenuItem* parent):MenuItem(parent,svRunAutoRamp,F("Ramp")){
}
void RunAutoRampMenu::OnSelectedInMenu(){
	Serial.print(F(">>>>>> Push Run Ramp <<<<<<  "));Serial.println(atg.state);
	Serial.println(atg.state);
	if(Selected)atg.savetoEEprom();
	Selected = !Selected;
}

RunAutoSwitch::RunAutoSwitch(MenuItem* parent):MenuItem(parent,svRunAuto,F("")){
	if(atg.pidStates[atg.selectedController].autoModeOn>0){
		Caption=F("Auto");
	}else{
		Caption=F("Manual");
	}
}

void RunAutoSwitch::OnLongPress(){
    Serial.print(F(">>>>>> Long press Switch <<<<<<  "));Serial.println(atg.pidStates[atg.selectedController].autoModeOn);

	if(atg.pidStates[atg.selectedController].autoModeOn==1){
		atg.pidStates[atg.selectedController].autoModeOn = 0;
	}else{
		atg.pidStates[atg.selectedController].autoModeOn = 1;
		atg.pidStates[atg.selectedController].forcedOutput=0;
	}
	atg.savetoEEprom();

	if(atg.pidStates[atg.selectedController].autoModeOn>0){
		Caption=F("Auto");
	}else{
		Caption=F("Manual");
	}

}

RunAutoMenu::RunAutoMenu(MenuItem* parent):MenuItem(parent,svRunAuto,F("Auto")){
	subMenuItems.resize(4);
	
	subMenuItems[0] = new UpMenu(this,svMain);
	subMenuItems[1] = switchMenu = new RunAutoSwitch(this);
	subMenuItems[2] = setpointMenu = new RunAutoSetpointMenu(this);
	subMenuItems[3] = rampMenu = new RunAutoRampMenu(this);
}

void RunAutoMenu::OnSelectedInMenu(){
	atg.SetState(svRunAuto);
}

void RunAutoMenu::HandleEncoderPush(EncoderSwStates pst){
	MenuItem::HandleEncoderPush(pst);
}

void RunAutoMenu::HandleEncoderMovement(EncoderMovement mvmnt){
	if(!rampMenu->Selected&&!setpointMenu->Selected && !(atg.pidStates[atg.selectedController].autoModeOn==0 &&switchMenu->Selected)){
		MenuItem::HandleEncoderMovement(mvmnt);
	}

	//last resort status fix
	if(atg.pidStates[atg.selectedController].autoModeOn==1) switchMenu->Selected = false;

	if(rampMenu->Selected){
		if(mvmnt==EncMoveCCW){
			atg.pidStates[atg.selectedController].decRamp();
		}else if(mvmnt==EncMoveCW){
			atg.pidStates[atg.selectedController].incRamp();
		}
	}else if(setpointMenu->Selected){
		if(mvmnt==EncMoveCCW){
			atg.pidStates[atg.selectedController].decSetpoint();
		}else if(mvmnt==EncMoveCW){
			atg.pidStates[atg.selectedController].incSetpoint();
		}
	} else if(atg.pidStates[atg.selectedController].autoModeOn==0 && switchMenu->Selected){
		//manual mode. handle output percentage

		if(mvmnt==EncMoveCCW){
			Serial.print(F(">>>>>> Change forced output to"));Serial.println(atg.pidStates[atg.selectedController].forcedOutput+1);
			atg.pidStates[atg.selectedController].forcedOutput++;
			if(atg.pidStates[atg.selectedController].forcedOutput>100){
				atg.pidStates[atg.selectedController].forcedOutput=100;
			}
		}else if(mvmnt==EncMoveCW){
			Serial.print(F(">>>>>> Change forced output to"));Serial.println(atg.pidStates[atg.selectedController].forcedOutput-1);
			atg.pidStates[atg.selectedController].forcedOutput--;
			if(atg.pidStates[atg.selectedController].forcedOutput<0){
				atg.pidStates[atg.selectedController].forcedOutput=0;
			}
		}
		atg.pidStates[atg.selectedController].setOutPerc(atg.pidStates[atg.selectedController].forcedOutput);
	}
}

MainMenu::MainMenu():MenuItem(NULL,-1,F("Main")){
	subMenuItems.resize(2);
	subMenuItems[0] = runMenu = new RunAutoMenu(this);
	subMenuItems[1] = configMenu = new ConfigMenu(this);
}

void MainMenu::OnSelectedInMenu(){
}



