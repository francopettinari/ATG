// Do not remove the include below
#include <WString.h>
#include "Arduino.h"
#include "Pid.h"
#include "LCDHelper.h"
#include <Encoder.h>
#include "PidState.h"
#ifdef DEBUG
#include "gdb.h"
#endif

PidState pidState;
LCDHelper lcdHelper;

const byte encoderCk     = D5; // rotary encoder Clock
const byte pushButtonPin = D7; // rotary encoder pushbutton
const byte encoderDt     = D6; // rotary encoder Data
boolean isEncoderPressed;
Encoder enc(encoderCk, encoderDt);

void handleEncPush() {
	isEncoderPressed = digitalRead(pushButtonPin) == 0;
}
#ifdef DEBUG
void RAMFUNC setup() {
	uart_div_modify(0, UART_CLK_FREQ / 115200);
	Serial.begin(115200);
	gdbstub_init();
#else
void setup() {
	Serial.begin(115200);
	Serial.println(F("Initialized"));
#endif



	pinMode(pushButtonPin, INPUT_PULLUP);
	attachInterrupt(pushButtonPin, handleEncPush, CHANGE);
	InitializeMenus();
}

#ifdef DEBUG
void RAMFUNC loop() {
#else
	void loop() {
#endif
	pidState.update(enc.read(),isEncoderPressed);
	lcdHelper.display(pidState);
}
