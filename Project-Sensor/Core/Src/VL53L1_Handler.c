#include <VL53L1_Handler.h>

uint8_t VL53L1X_bootDualSensors(VL53L1X* sensor1, VL53L1X* sensor2)
{
	int status = SUCCESS;
	TOF_TurnOff(sensor1);
	TOF_TurnOff(sensor2);

	status |= TOF_BootSensor(sensor1);
	status |= TOF_BootSensor(sensor2);

	return status;
}

void VL53L1X_errorHandler(int status) {
	char buffer[10];
	sprintf(buffer, "%d", (int)status);
	HAL_UART_Transmit(&huart1, buffer, strlen(buffer), HAL_MAX_DELAY);
	while (1) {}
}

HAL_StatusTypeDef VL53L1X_getDistance(VL53L1X* sensor) {
	sensor->distance = TOF_GetDistance(sensor);
	return sensor->distance == 0xFFFF ? HAL_ERROR : HAL_OK;
}
