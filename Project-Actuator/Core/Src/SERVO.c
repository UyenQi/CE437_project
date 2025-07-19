#include "SERVO.h"

#define NEUTRAL_PULSE 750
#define SCALING_FACTOR 5.56

void servoStart()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	setDefaultAngle();
	HAL_Delay(100);
}

uint16_t A2PWM(int8_t angle)
{
	return (uint16_t)(NEUTRAL_PULSE + angle * SCALING_FACTOR);
}

void setDefaultAngle()
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, A2PWM(0));
}

void turn(uint8_t angle, int8_t direction)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, A2PWM(angle * direction));
}
