#include "dht11.h"
#include <stdio.h>

// Private variables
static TIM_HandleTypeDef *dht11_timer;

void DHT11_Init(TIM_HandleTypeDef *htim) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	dht11_timer = htim;
    HAL_TIM_Base_Start(dht11_timer);
}

static void Delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(dht11_timer, 0);
    while (__HAL_TIM_GET_COUNTER(dht11_timer) < us);
}

static void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

static void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    Delay_us(18000);  // 18ms delay
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    Delay_us(30);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Check_Response(void) {
    uint8_t response = 0;
    Delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        Delay_us(80);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) response = 1;
        else response = 0;
    }
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // Wait for signal to end
    return response;
}

uint8_t DHT11_Read(void) {
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));  // Wait for HIGH
        Delay_us(40);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) value |= (1 << (7 - i));
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // Wait for LOW
    }
    return value;
}

uint8_t DHT11_ReadData(float *temperature, float *humidity) {
    uint8_t RH_Int, RH_Dec, Temp_Int, Temp_Dec, Checksum;

    DHT11_Start();
    if (DHT11_Check_Response()) {
        RH_Int = DHT11_Read();
        RH_Dec = DHT11_Read();
        Temp_Int = DHT11_Read();
        Temp_Dec = DHT11_Read();
        Checksum = DHT11_Read();

        if ((RH_Int + RH_Dec + Temp_Int + Temp_Dec) == Checksum) {
            *humidity = RH_Int + (float)RH_Dec / 10;
            *temperature = Temp_Int + (float)Temp_Dec / 10;
            return 1;  // Success
        }
    }
    return 0;  // Error
}
