/*
 * CANAgent.c
 *
 *  Created on: Jul 24, 2022
 *      Author: gilad
 */
#include "main.h"

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
//extern CAN_TxHeaderTypeDef TxHeaderSapState;
uint8_t TxData[8];


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		// Error_Handler();
	}
//	if (ee.DBG == 13)
//	{
//		sprintf(term_buffer, "\r\n%s StdID: -%02X, ", CT(), RxHeader.StdId);
//		PRINT_SCR(true);
//		sprintf(term_buffer, "IDE: -%02X, ", RxHeader.IDE);
//		PRINT_SCR(true);
//		sprintf(term_buffer, "DLC: -%02X, ", RxHeader.DLC);
//		PRINT_SCR(true);
//		for (int i = 0; i < 8; i++)
//		{
//			sprintf(term_buffer, "-%02X", RxData[i]);
//			PRINT_SCR(true);
//		}
//		sprintf(term_buffer, "\r\n");
//		PRINT_SCR(true);
//		memset(term_buffer, 0, 1024);
//	}
}
