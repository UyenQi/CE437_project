#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "tim.h"
#include "gpio.h"

#define FORWARD 	1
#define BACKWARD 	0

void motorStart();
void PWMgen(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle);
void run(uint8_t speed, uint8_t direction);
void stop();

#endif

