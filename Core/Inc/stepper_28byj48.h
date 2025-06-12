/*
 * stepper_28byj48.h
 *
 *  Created on: Apr 18, 2025
 *      Author: benbr
 */

#ifndef STEPPER_28BYJ48_H
#define STEPPER_28BYJ48_H

#include "stm32f4xx_hal.h"

// Adjust these to match your GPIO connections
#define IN1_PIN GPIO_PIN_4
#define IN1_PORT GPIOB
#define IN2_PIN GPIO_PIN_5
#define IN2_PORT GPIOB
#define IN3_PIN GPIO_PIN_6
#define IN3_PORT GPIOB
#define IN4_PIN GPIO_PIN_10
#define IN4_PORT GPIOB

// Motor characteristics
#define STEPS_PER_REVOLUTION 2048  // Full rotation: 2048 steps
#define STEP_DELAY_MS 0.5            // Default step delay (ms)

typedef enum {
    DIRECTION_CW = 0,
    DIRECTION_CCW = 1
} StepperDirection;

extern StepperDirection current_direction;

// Initialization and configuration
void Stepper_Init(void);
void Stepper_SetSpeed(uint16_t rpm);

// Blocking control
void Stepper_Step(uint32_t steps);
void Stepper_Stop(void);

// Non-blocking control

void Stepper_StepNonBlocking(void);

#endif // STEPPER_28BYJ48_H
