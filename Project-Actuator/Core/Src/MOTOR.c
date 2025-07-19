#include "MOTOR.h"

void motorStart()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET);
}

void run(uint8_t speed, uint8_t v_direction)
{
	PWMgen(&htim1,TIM_CHANNEL_4, (v_direction == FORWARD ? speed : 0));
	PWMgen(&htim1,TIM_CHANNEL_1, (v_direction == BACKWARD ? speed : 0));
}

void PWMgen(TIM_HandleTypeDef *htim, uint32_t channel, float duty_cycle)
{
	float load_value = (duty_cycle / 100) * htim->Instance->ARR;
	__HAL_TIM_SET_COMPARE(htim, channel, (uint16_t)load_value);
}

void stop()
{
	PWMgen(&htim1,TIM_CHANNEL_4, 0);
	PWMgen(&htim1,TIM_CHANNEL_1, 0);
	HAL_Delay(200);
}
