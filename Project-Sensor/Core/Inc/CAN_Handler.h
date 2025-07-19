#ifndef _CAN_HANDLER_H_
#define _CAN_HANDLER_H_

#include "can.h"
#include "main.h"

#define ACTUATOR_ADDR	0x0A2
#define SENSOR_ADDR		0X012

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t* buffer;
}CAN_TxHandlerTypeDef;

typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t* buffer;
}CAN_RxHandlerTypeDef;

extern CAN_TxHandlerTypeDef tx_handler;
extern uint32_t Mailbox;

extern CAN_RxHandlerTypeDef rx_handler;
extern CAN_FilterTypeDef can_filter;

void CAN_ComInit_Std(CAN_TxHeaderTypeDef *Tx_Header, CAN_HandleTypeDef *hcan, uint32_t id, uint8_t dlc);
void CAN_Filter_SingleFF0_Config(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *can_filter, uint32_t addr);
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHandlerTypeDef *tx_handler);

#endif
