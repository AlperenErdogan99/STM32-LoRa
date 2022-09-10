/*
 * LoRa_Config.c
 *
 *  Created on: Apr 9, 2021
 *      Author: alperen
 */
#include "LoRa.h"
#include "main.h"
extern SPI_HandleTypeDef hspi3;

void LoRa_Config_Pin(LoRa *myLoRa) {

	myLoRa->CS_port = lora_cs_GPIO_Port;
	myLoRa->CS_pin = lora_cs_Pin;
	myLoRa->reset_port = lora_reset_GPIO_Port;
	myLoRa->reset_pin = lora_reset_Pin;
	myLoRa->DIO0_port = GPIOE;
	myLoRa->DIO0_pin = GPIO_PIN_13;
	myLoRa->hSPIx = &hspi3;



}
