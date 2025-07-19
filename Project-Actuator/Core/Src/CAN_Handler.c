#include "CAN_Handler.h"

CAN_TxHandlerTypeDef tx_handler;
uint32_t Mailbox = CAN_TX_MAILBOX0;

CAN_RxHandlerTypeDef rx_handler;
CAN_FilterTypeDef can_filter;

void CAN_ComInit_Std(
	CAN_TxHeaderTypeDef *tx_header,
	CAN_HandleTypeDef *hcan,
	uint32_t id,
	uint8_t dlc
)
{
	tx_header->IDE 		= CAN_ID_STD;
	tx_header->DLC 		= dlc;
	tx_header->StdId 	= id;
	tx_header->RTR 		= CAN_RTR_DATA;

	HAL_CAN_Start(hcan);
}

void CAN_Filter_SingleFF0_Config(
	CAN_HandleTypeDef *hcan,
	CAN_FilterTypeDef *can_filter,
	uint32_t addr)
{
	can_filter->FilterActivation = CAN_FILTER_ENABLE;
	can_filter->FilterBank = 0;
	can_filter->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter->FilterIdHigh = addr << 5;
	can_filter->FilterIdLow = 0x0000;
	can_filter->FilterMaskIdHigh = addr << 5;
	can_filter->FilterMaskIdLow = 0x0000;
	can_filter->FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter->FilterScale = CAN_FILTERSCALE_32BIT;

	HAL_CAN_ConfigFilter(hcan, can_filter);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, CAN_TxHandlerTypeDef* tx_handler)
{
	if(HAL_CAN_AddTxMessage(hcan, &tx_handler->header, tx_handler->buffer, &Mailbox) != HAL_OK){
		return HAL_ERROR;
	}
	return HAL_OK;
}
