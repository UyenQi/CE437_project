#include "main.h"
#include "usart.h"
#include "i2c.h"
#include "VL53L1X.h"
#include "VL53L1X_api.h"

#ifndef _VL53L1_HANDLER_H_
#define _VL53L1_HANDLER_H_

uint8_t VL53L1X_bootDualSensors(VL53L1X* sensor1, VL53L1X* sensor2);
void VL53L1X_errorHandler(int status);
HAL_StatusTypeDef VL53L1X_getDistance(VL53L1X* sensor);

#endif
