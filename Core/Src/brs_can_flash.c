/*
 * brs_can_flash.cpp
 *
 *  Created on: 29 Mar 2023
 *      Author: jspin
 */

#include "brs_can_flash.h"
#include "FLASH_SECTOR_F4.h"
#include <stdio.h>

const size_t FLASH_START_ADDR = 0x8008000;

enum CAN_FLASH_STATE state = CF_IDLE;

#define FLASH_BUFFER_WORD_COUNT 1024 // 4 KB
uint32_t flashWordBuf[FLASH_BUFFER_WORD_COUNT];
size_t flashWordBufIdx = 0;

size_t appByteCount = 0;
size_t appWordCount = 0;
size_t flashAddr = FLASH_START_ADDR;

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
		// Get and check rx dlc
		const uint32_t dlc = rxHeader->DLC;
		if (dlc != 8)
		{
			return CF_ERROR;
		}

		// Get app size
		appByteCount = *(size_t*)rxData;
		appWordCount = appByteCount / sizeof(uint32_t);

		// Reset start address
		flashAddr = FLASH_START_ADDR;

		// Acknowledge
		txHeader->StdId = CAN_MSG_ID_FLASH_ACK;
		txHeader->ExtId = 0;
		txHeader->DLC = dlc;
		txHeader->IDE = CAN_ID_STD;
		txHeader->RTR = CAN_RTR_DATA;

		for (int i = 0; i < dlc; i++)
		{
			txData[i] = rxData[i];
		}

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

	// If init flash (can occur in this state)
	if (msgId == CAN_MSG_ID_FLASH_INIT)
	{
		return process_can_idle(rxHeader, rxData, txHeader, txData);
	}
	// If new data to flash
	else if (msgId == CAN_MSG_ID_FLASH_DATA)
	{
		// Get and check rx dlc
		const uint32_t dlc = rxHeader->DLC;
		if (dlc != 8)
		{
			return CF_ERROR;
		}

		// Fill flash word buffer
		const uint32_t* rxWords = (uint32_t*)rxData;
		flashWordBuf[flashWordBufIdx++] = rxWords[0];
		flashWordBuf[flashWordBufIdx++] = rxWords[1];

		// If word buffer full
		if (flashWordBufIdx == FLASH_BUFFER_WORD_COUNT)
		{
			// Flash
			Flash_Write_Data(flashAddr, flashWordBuf, FLASH_BUFFER_WORD_COUNT);

			// Increase flash address
			flashAddr += sizeof(uint32_t) * FLASH_BUFFER_WORD_COUNT;

			// Reset flash word buffer
			flashWordBufIdx = 0;
		}

		// If flashing finished
		if (flashAddr == FLASH_START_ADDR + appByteCount)
		{
			// TODO: Maybe some integrity checks in flash memory

			printf("%x\n", flashAddr);
			// Send flash finished message
			txHeader->StdId = CAN_MSG_ID_FLASH_FIN;
			txHeader->ExtId = 0;
			txHeader->DLC = dlc;
			txHeader->IDE = CAN_ID_STD;
			txHeader->RTR = CAN_RTR_DATA;

			for (uint32_t i = 0; i < dlc; i++)
			{
				txData[i] = rxData[i];
			}

			return CF_FINISHED;
		}
		// If flashing not finished
		else
		{
			// Acknowledge
			txHeader->StdId = CAN_MSG_ID_FLASH_ACK;
			txHeader->ExtId = 0;
			txHeader->DLC = dlc;
			txHeader->IDE = CAN_ID_STD;
			txHeader->RTR = CAN_RTR_DATA;

			for (uint32_t i = 0; i < dlc; i++)
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
	const uint32_t msgId = rxHeader->StdId;

	// If init flash (can occur in this state)
	if (msgId == CAN_MSG_ID_FLASH_INIT)
	{
		return process_can_idle(rxHeader, rxData, txHeader, txData);
	}

	// TODO: Implement post finished can messages

	return CF_ERROR;
}

enum CAN_FLASH_STATE process_can_error(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{
	const uint32_t msgId = rxHeader->StdId;

	// If init flash (can occur in this state)
	if (msgId == CAN_MSG_ID_FLASH_INIT)
	{
		return process_can_idle(rxHeader, rxData, txHeader, txData);
	}

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
