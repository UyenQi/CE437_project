#ifndef VL53L1X_H_
#define VL53L1X_H_

#include "i2c.h"
#include "gpio.h"

#define SUCCESS					0x00
#define CONF_ADDR_ERR_SENSOR_1	0x01
#define CONF_ADDR_ERR_SENSOR_2	0x02
#define CONF_UNKNOWN_ADDR		0x09
#define RANGING_ERR				0x04
#define INIT_ERR				0x03
#define TOO_MANY_SENSOR_ERR		0x01

#define INVALID_ADDR			0x29
#define	SENSOR_1_ADDR			0x20
#define	SENSOR_2_ADDR			0x26

typedef struct
{
	// I2C information
	I2C_HandleTypeDef* hi2c;
	uint8_t address;

	// XSHUT
	struct {
		GPIO_TypeDef* port;
		uint16_t pin;
	}xshut;

	// Used for driver back-end, not directly related to VL53L1X
	uint8_t id;
	volatile int16_t distance;

} VL53L1X;


int TOF_InitStruct(VL53L1X* sensor, I2C_HandleTypeDef* hi2c, uint8_t address, GPIO_TypeDef* xshut_port, uint16_t xshut_pin);

int TOF_BootSensor(VL53L1X* sensor);
int TOF_BootMultipleSensors(VL53L1X** sensors, uint8_t count);

void TOF_TurnOn(const VL53L1X* sensor);
void TOF_TurnOff(const VL53L1X* sensor);

uint16_t TOF_GetDistance(const VL53L1X* sensor);

#endif // VL53L1X_H_
