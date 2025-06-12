/*
 * stepper_28byj48.c
 *
 *  Created on: Apr 18, 2025
 *      Author: benbr
 */

#include "stepper_28byj48.h"
#include "main.h"

static const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

static uint16_t step_delay = STEP_DELAY_MS;
StepperDirection current_direction = DIRECTION_CW;
static uint8_t current_step = 0;

void Stepper_Init(void) {
    // Initialize GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(IN1_PORT, &GPIO_InitStruct);
    HAL_GPIO_Init(IN2_PORT, &GPIO_InitStruct);
    HAL_GPIO_Init(IN3_PORT, &GPIO_InitStruct);
    HAL_GPIO_Init(IN4_PORT, &GPIO_InitStruct);

    // Start with all coils off
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
}

void Stepper_SetSpeed(uint16_t rpm) {
    // Convert RPM to step delay (ms)
    if (rpm > 0) {
        step_delay = 60000 / (STEPS_PER_REVOLUTION * rpm);
        if (step_delay < 1) step_delay = 0.005;
    }

}

void Stepper_WritePins(uint8_t state1, uint8_t state2, uint8_t state3, uint8_t state4) {
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, state1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, state2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, state3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, state4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Step(uint32_t steps) {
    for(uint32_t i = 0; i < steps; i++) {
        if(current_direction == DIRECTION_CW) {
            current_step++;
            if(current_step >= 8) current_step = 0;
        } else {
            if(current_step == 0) current_step = 8;
            current_step--;
        }

        Stepper_WritePins(
            step_sequence[current_step][0],
            step_sequence[current_step][1],
            step_sequence[current_step][2],
            step_sequence[current_step][3]
        );

        HAL_Delay(step_delay);
    }
}


void Stepper_RotateContinuous(StepperDirection dir) {
    current_direction = dir;
    while(1)
    {
    	Stepper_Step(1);
    }

}

void Stepper_Stop(void) {
    Stepper_WritePins(0, 0, 0, 0);
}

