#include "VL53L1X.h"
#include "VL53L1X_api.h"

// Helper functions & variables
#define _VL53L1X_MAX_SENSORS 10

VL53L1X* registered_sensors[_VL53L1X_MAX_SENSORS] = { 0 };
const size_t registered_sensors_cnt = _VL53L1X_MAX_SENSORS;

// Function definitions
int TOF_InitStruct(
	VL53L1X* sensor,
	I2C_HandleTypeDef* hi2c,
	uint8_t address,
	GPIO_TypeDef* xshut_port,
	uint16_t xshut_pin
)
{
	if (address == INVALID_ADDR)

	sensor->hi2c = hi2c;
	sensor->address = address;

	sensor->xshut.port = xshut_port;
	sensor->xshut.pin = xshut_pin;

	// Find free slot
	uint8_t id;
	for (id = 0; (id < _VL53L1X_MAX_SENSORS) && registered_sensors[id] != NULL; id++);

	if (id >= _VL53L1X_MAX_SENSORS)
	{
		return TOO_MANY_SENSOR_ERR;
	}

	sensor->id = id;
	registered_sensors[id] = sensor;
	return SUCCESS;
}

void TOF_TurnOn(const VL53L1X* sensor)
{
	HAL_GPIO_WritePin(sensor->xshut.port, sensor->xshut.pin, GPIO_PIN_SET);
}

void TOF_TurnOff(const VL53L1X* sensor)
{
	HAL_GPIO_WritePin(sensor->xshut.port, sensor->xshut.pin, GPIO_PIN_RESET);
}

int TOF_BootMultipleSensors(VL53L1X** sensors, uint8_t sensor_number)
{
	int status = SUCCESS;

	for (uint8_t i = 0; i < sensor_number; i++) {
		TOF_TurnOff(sensors[i]);
	}

	for (uint8_t i = 0; i < sensor_number; i++) {
		status |= TOF_BootSensor(sensors[i]);
	}

	return status;
}

int TOF_BootSensor(VL53L1X* sensor)
{
	int status = SUCCESS;
	TOF_TurnOn(sensor);
    HAL_Delay(2); // Sometimes this is too fast and line can't settle

    // Check if sensor is already configured with the correct address
    uint16_t id = 0;
    if (VL53L1X_GetSensorId(sensor->id, &id) != 0)
    {
    	if (VL53L1X_SetI2CAddress(sensor->id, sensor->address) != SUCCESS)
    	{
    		switch(sensor->address){
    			case SENSOR_1_ADDR:
    				status = CONF_ADDR_ERR_SENSOR_1;
    				break;
    			case SENSOR_2_ADDR:
    				status = CONF_ADDR_ERR_SENSOR_2;
    				break;
    			default:
    				status = CONF_UNKNOWN_ADDR;
    				break;
    		}
    		return status;
    	}
    }

	// Initialize sensor
	if (VL53L1X_SensorInit(sensor->id) != SUCCESS)
		status = INIT_ERR;

	if (VL53L1X_StartRanging(sensor->id) != SUCCESS)
		status = RANGING_ERR;

	return status;
}

uint16_t TOF_GetDistance(const VL53L1X* sensor)
{
	uint16_t reading = 0;
	if(VL53L1X_GetDistance(sensor->id, &reading) != 0)
	{
		return 0xFFFF; // VL53L1X_GetDistance() returns the reading for previous sensor if reading failed
	}
	return reading;
}
