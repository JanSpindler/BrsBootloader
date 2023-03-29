/*
 * brs_can_flash.h
 *
 *  Created on: 29 Mar 2023
 *      Author: jspin
 */

#ifndef INC_BRS_CAN_FLASH_H_
#define INC_BRS_CAN_FLASH_H_

#include "main.h"

static const uint32_t CAN_MSG_ID_INIT_FLASH = 0x100; // Sent from host to init flashing
static const uint32_t CAN_MSG_ID_FLASH_DATA = 0x101; // Sent from host to transmit 8 byte of flash data
static const uint32_t CAN_MSG_ID_FLASH_ACK = 0x102; // Sent from controller to acknowledge flash init or data frame

enum CAN_FLASH_STATE
{
	CF_IDLE, // When no flashing has been initialized
	CF_RX_READY, // When ready to receive more flash data
	CF_FLASHING, // When flashing data
	CF_FINISHED, // When flashing is finished
	CF_ERROR // When an error has occurred
};

CAN_FLASH_STATE get_can_flash_state();

void process_can(
	const CAN_RxHeaderTypeDef* rxHeader,
	const uint8_t* rxData,
	CAN_TxHeaderTypeDef* txHeader,
	uint8_t* txData);

#endif /* INC_BRS_CAN_FLASH_H_ */
