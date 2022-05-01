/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define RL1_Pin GPIO_PIN_0
#define RL1_GPIO_Port GPIOC
#define EXT_RST_Pin GPIO_PIN_1
#define EXT_RST_GPIO_Port GPIOC
#define LV_RST_Pin GPIO_PIN_2
#define LV_RST_GPIO_Port GPIOC
#define RL2_Pin GPIO_PIN_3
#define RL2_GPIO_Port GPIOC
#define EXT_TX_Pin GPIO_PIN_0
#define EXT_TX_GPIO_Port GPIOA
#define EXT_RX_Pin GPIO_PIN_1
#define EXT_RX_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_2
#define ESP_TX_GPIO_Port GPIOA
#define IR_L_Pin GPIO_PIN_3
#define IR_L_GPIO_Port GPIOA
#define V_R_Pin GPIO_PIN_4
#define V_R_GPIO_Port GPIOA
#define IR_H_Pin GPIO_PIN_5
#define IR_H_GPIO_Port GPIOA
#define IY_L_Pin GPIO_PIN_6
#define IY_L_GPIO_Port GPIOA
#define V_Y_Pin GPIO_PIN_7
#define V_Y_GPIO_Port GPIOA
#define RS_TX_Pin GPIO_PIN_4
#define RS_TX_GPIO_Port GPIOC
#define RS_RX_Pin GPIO_PIN_5
#define RS_RX_GPIO_Port GPIOC
#define IY_H_Pin GPIO_PIN_0
#define IY_H_GPIO_Port GPIOB
#define IB_H_Pin GPIO_PIN_1
#define IB_H_GPIO_Port GPIOB
#define V_B_Pin GPIO_PIN_2
#define V_B_GPIO_Port GPIOB
#define IB_L_Pin GPIO_PIN_10
#define IB_L_GPIO_Port GPIOB
#define RS_DE_Pin GPIO_PIN_14
#define RS_DE_GPIO_Port GPIOB
#define ESP_Int_IN_Pin GPIO_PIN_15
#define ESP_Int_IN_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_8
#define SW3_GPIO_Port GPIOA
#define PROG_TX_Pin GPIO_PIN_9
#define PROG_TX_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_6
#define SW4_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_7
#define SW2_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOD
#define Thermal_Pin GPIO_PIN_9
#define Thermal_GPIO_Port GPIOD
#define PROG_RX_Pin GPIO_PIN_10
#define PROG_RX_GPIO_Port GPIOA
#define MOTOR_STATUS_Pin GPIO_PIN_11
#define MOTOR_STATUS_GPIO_Port GPIOA
#define PICKup_trip_Pin GPIO_PIN_12
#define PICKup_trip_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_15
#define ESP_RX_GPIO_Port GPIOA
#define ESP_IN_OUT_Pin GPIO_PIN_10
#define ESP_IN_OUT_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
