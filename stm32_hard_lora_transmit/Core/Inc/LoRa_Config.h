/*
 * LoRa_Config.h
 *
 *  Created on: Apr 9, 2021
 *      Author: alperen
 */

#ifndef INC_LORA_CONFIG_H_
#define INC_LORA_CONFIG_H_

#include "LoRa.h"
#include "main.h"



	uint8_t transmitOK;
	uint8_t receiveOK;
	uint8_t cnt = 0;
	uint8_t send_data[128];
	uint8_t read_data[128];

typedef struct {
	uint8_t transmit_data[128];
	uint8_t receive_data[128];
}LoRa_Transceiver_Handle;
void LoRa_Config_Pin(LoRa *my_lora);

#endif /* INC_LORA_CONFIG_H_ */
