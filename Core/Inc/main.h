/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define cw_sensor_Pin GPIO_PIN_0
#define cw_sensor_GPIO_Port GPIOC
#define ccw_sensor_Pin GPIO_PIN_1
#define ccw_sensor_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Green_Led_Pin GPIO_PIN_5
#define Green_Led_GPIO_Port GPIOA
#define Yellow_Led_Pin GPIO_PIN_6
#define Yellow_Led_GPIO_Port GPIOA
#define Red_Led_Pin GPIO_PIN_7
#define Red_Led_GPIO_Port GPIOA
#define STEPPER_IN4_Pin GPIO_PIN_10
#define STEPPER_IN4_GPIO_Port GPIOB
#define LB_relay_Pin GPIO_PIN_13
#define LB_relay_GPIO_Port GPIOB
#define Fan_relay_Pin GPIO_PIN_14
#define Fan_relay_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Us_Butt_Pin GPIO_PIN_12
#define Us_Butt_GPIO_Port GPIOC
#define Us_Butt_EXTI_IRQn EXTI15_10_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define STEPPER_IN1_Pin GPIO_PIN_4
#define STEPPER_IN1_GPIO_Port GPIOB
#define STEPPER_IN2_Pin GPIO_PIN_5
#define STEPPER_IN2_GPIO_Port GPIOB
#define STEPPER_IN3_Pin GPIO_PIN_6
#define STEPPER_IN3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#include "dht11.h"
#include "stepper_28byj48.h"
#include "lcd_i2c.h"

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
