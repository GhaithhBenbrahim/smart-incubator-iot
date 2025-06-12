/*
 * dht11.h
 *
 *  Created on: Apr 18, 2025
 *      Author: benbr
 */

#ifndef DHT11_H_
#define DHT11_H_

#include "stm32f4xx_hal.h"

// Configuration - adjust these to match your hardware
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

// Function prototypes
void DHT11_Init(TIM_HandleTypeDef *htim);
uint8_t DHT11_ReadData(float *temperature, float *humidity);

// Helper functions (can be made static if only used internally)
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);



#endif /* DHT11_H_ */
