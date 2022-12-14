/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MFRC522.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "LoRa.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define rfid_cs_Pin GPIO_PIN_3
#define rfid_cs_GPIO_Port GPIOA
#define Lora_DIO0_Pin GPIO_PIN_13
#define Lora_DIO0_GPIO_Port GPIOE
#define lora_reset_Pin GPIO_PIN_14
#define lora_reset_GPIO_Port GPIOE
#define lora_cs_Pin GPIO_PIN_15
#define lora_cs_GPIO_Port GPIOE
#define lora_led_2_Pin GPIO_PIN_8
#define lora_led_2_GPIO_Port GPIOC
#define lora_led_1_Pin GPIO_PIN_9
#define lora_led_1_GPIO_Port GPIOC
#define rfid_led_3_Pin GPIO_PIN_8
#define rfid_led_3_GPIO_Port GPIOA
#define rfid_led_2_Pin GPIO_PIN_9
#define rfid_led_2_GPIO_Port GPIOA
#define rfid_led_1_Pin GPIO_PIN_10
#define rfid_led_1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
