/*
 * brs_can_flash.cpp
 *
 *  Created on: 29 Mar 2023
 *      Author: jspin
 */

#include "brs_can_flash.h"

static enum CAN_FLASH_STATE state = CF_IDLE;

enum CAN_FLASH_STATE get_can_flash_state()
{
	return state;
}

enum CAN_FLASH_STATE process_can_idle(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{
	const uint32_t msgId = rxHeader->StdId;

	// Flash start request
	if (msgId == CAN_MSG_ID_FLASH_INIT)
	{
		// Acknowledge
		txHeader->StdId = CAN_MSG_ID_FLASH_ACK;
		txHeader->ExtId = 0;
		txHeader->DLC = 0;
		txHeader->IDE = CAN_ID_STD;
		txHeader->RTR = CAN_RTR_DATA;

		return CF_RX_READY;
	}

	return CF_ERROR;
}

enum CAN_FLASH_STATE process_can_rx_ready(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{
	const uint32_t msgId = rxHeader->StdId;

	// New data to flash
	if (msgId == CAN_MSG_ID_FLASH_DATA)
	{
		// TODO: Flash

		// TODO: Check if flashing is complete
		// If flashing complete
		if (0)
		{
			// TODO: Maybe some integrity checks in flash memory

			txHeader->StdId = CAN_MSG_ID_FLASH_FIN;
			txHeader->ExtId = 0;
			txHeader->DLC = rxHeader->DLC;
			txHeader->IDE = CAN_ID_STD;
			txHeader->RTR = CAN_RTR_DATA;

			for (uint32_t i = 0; i < rxHeader->DLC; i++)
			{
				txData[i] = rxData[i];
			}

			return CF_FINISHED;
		}
		// If flashing not complete
		else
		{
			txHeader->StdId = CAN_MSG_ID_FLASH_ACK;
			txHeader->ExtId = 0;
			txHeader->DLC = rxHeader->DLC;
			txHeader->IDE = CAN_ID_STD;
			txHeader->RTR = CAN_RTR_DATA;

			for (uint32_t i = 0; i < rxHeader->DLC; i++)
			{
				txData[i] = rxData[i];
			}

			return CF_RX_READY;
		}
	}

	return CF_ERROR;
}

enum CAN_FLASH_STATE process_can_finished(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{
	// TODO: Implement post finished can messages

	return CF_ERROR;
}

enum CAN_FLASH_STATE process_can_error(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{
	// TODO: Implement error handling

	return CF_ERROR;
}

void process_can(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{
	switch (state)
	{
	case CF_IDLE:
		state = process_can_idle(rxHeader, rxData, txHeader, txData);
		break;
	case CF_RX_READY:
		state = process_can_rx_ready(rxHeader, rxData, txHeader, txData);
		break;
	case CF_FINISHED:
		state = process_can_finished(rxHeader, rxData, txHeader, txData);
		break;
	case CF_ERROR:
		state = process_can_error(rxHeader, rxData, txHeader, txData);
		break;
	default:
		state = CF_ERROR;
		break;
	}
}
