/*
 * brs_can_flash.cpp
 *
 *  Created on: 29 Mar 2023
 *      Author: jspin
 */

#include "brs_can_flash.h"

static CAN_FLASH_STATE state = CF_IDLE;

CAN_FLASH_STATE get_can_flash_state()
{
	return state;
}

CAN_FLASH_STATE process_can_idle(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{

}

CAN_FLASH_STATE process_can_rx_ready(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{

}

CAN_FLASH_STATE process_can_flashing(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{

}

CAN_FLASH_STATE process_can_finished(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{

}

CAN_FLASH_STATE process_can_error(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData)
{

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
	case CF_FLASHING:
		state = process_can_flashing(rxHeader, rxData, txHeader, txData);
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
