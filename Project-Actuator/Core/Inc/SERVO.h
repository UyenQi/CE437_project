#ifndef _SERVO_H_
#define _SERVO_H_

#include "tim.h"
#include "main.h"

#define LEFT 		-1
#define	STRAIGHT	0
#define RIGHT 		1

void servoStart();
uint16_t A2PWM(int8_t angle);
void setDefaultAngle();
void turn(uint8_t angle, int8_t direction);

#endif

