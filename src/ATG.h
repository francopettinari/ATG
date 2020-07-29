#ifndef _Pid_H_
#define _Pid_H_

#include "Controller.h"
#include "LCDHelper.h"

#define SERVO1_PIN   14 //GPIO14
#define SERVO2_PIN   27 //GPIO27
#define ROTARY_PINA  15
#define ROTARY_PINB  4
#define ROTARY_PINSW 5

extern Controller pidState;
extern LCDHelper lcdHelper;

extern Servo_ESP32 servo;

//Do not add code below this line
#endif /* _Pid_H_ */
