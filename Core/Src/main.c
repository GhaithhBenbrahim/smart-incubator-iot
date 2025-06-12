 /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HMIN 50.0f
#define HMAX 55.0f
#define H_MID 52.5f
#define MOY_THRESHOLD 0.5f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorCommTask */
osThreadId_t SensorCommTaskHandle;
const osThreadAttr_t SensorCommTask_attributes = {
  .name = "SensorCommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for StepperTask */
osThreadId_t StepperTaskHandle;
const osThreadAttr_t StepperTask_attributes = {
  .name = "StepperTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FanTask */
osThreadId_t FanTaskHandle;
const osThreadAttr_t FanTask_attributes = {
  .name = "FanTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BulbTask */
osThreadId_t BulbTaskHandle;
const osThreadAttr_t BulbTask_attributes = {
  .name = "BulbTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
volatile uint8_t stepper_timer_flag = 1;


uint32_t tReset = 0;
uint8_t current_screen = 1;
bool Hgood = false;

volatile int8_t StepperState = 0;
volatile uint8_t speed = 0;

uint32_t tFan = 0;
uint32_t tFanOn = 10000;   // initial on time in ms
uint32_t tFanOFF = 20000;  // initial off time in ms
uint8_t FanState = 1;      // 1 = ON, 0 = OFF

bool LBon = false;
uint32_t tLastLBon = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartSensorCommTask(void *argument);
void StartStepperTask(void *argument);
void StartFanTask(void *argument);
void StartBulbTask(void *argument);
void StartLEDTask(void *argument);
void StartDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// System variables

void scan_i2c_bus(void)
{
    char msg[32];
    HAL_UART_Transmit(&huart2, (uint8_t*)"Scanning I2C bus:\r\n", 20, HAL_MAX_DELAY);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
        {
            int len = sprintf(msg, "Found device at 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        }
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"Scan done.\r\n", 12, HAL_MAX_DELAY);
}


float h, f,t, tLBon100;
float tmin = 1000.0f;  // start with a large number
float tmax = -1000.0f; // start with a very small number
float hmin = 1000.0f;
float hmax = -1000.0f;

void Led(float h)
{

    if (h >= HMIN && h <= HMAX)
    {
        HAL_GPIO_WritePin(GPIOA, Red_Led_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, Yellow_Led_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, Green_Led_Pin, GPIO_PIN_SET);
        Hgood = true;
    }
    else if (((h >= HMIN - 1) && (h < HMIN)) || ((h > HMAX) && (h <= HMAX + 1)))
    {
        HAL_GPIO_WritePin(GPIOA, Red_Led_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, Yellow_Led_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, Green_Led_Pin, GPIO_PIN_RESET);
        Hgood = false;
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, Red_Led_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, Yellow_Led_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, Green_Led_Pin, GPIO_PIN_RESET);
        Hgood = false;
    }

}

void Stepper_Update(void) {

    // Normal stepping (if enabled and no stop request)
    if (stepper_timer_flag) {

        Stepper_Step(1);
    }
}



typedef enum {
    STEPPER_IDLE = 0,
    STEPPER_MOVING_CW,
    STEPPER_MOVING_CCW
} StepperRunState;

StepperRunState stepper_run_state = STEPPER_IDLE;

void StepperControl(float h)
{

	static float tab[10] = {0};
	static uint8_t k = 0;
    float sum = 0;

    // Update circular buffer
    tab[k] = h;
    k = (k + 1) % 10;

    if (!Hgood) {
        float fillValue = (h < H_MID) ? HMIN : HMAX;
        for (int j = 0; j < 10; j++) {
            tab[j] = fillValue;
        }
        k = 0;

        // Direct control based on raw h
        speed = 50;
        StepperState = (h > H_MID) ? 1 : -1;

    } else {
        // Filtered control based on average of last 10 readings
        speed = 15;
        for (int j = 0; j < 10; j++) {
            sum += tab[j];
        }

        float moy = sum / 10.0f;
        float diff = moy - H_MID;

        if (diff >= MOY_THRESHOLD)
            StepperState = -1;
        else if (diff <= -MOY_THRESHOLD)
            StepperState = 1;
        else
            StepperState = 0;
    }
    /*
    // Determine speed and StepperState based on Hgood and h
    if (!Hgood)
    {
        speed = 10;
        StepperState = (h > 52.5f) ? -1 : 1;
    }
    else
    {
        speed = 10;
        if (h > 53.0f)
            StepperState = -1;
        else if (h < 52.0f)
            StepperState = 1;
        else
            StepperState = 0;
    }*/

    // Apply the speed setting to the stepper driver
    Stepper_SetSpeed(speed);

    // Update direction and run state
    if (StepperState == 1 && stepper_run_state != STEPPER_MOVING_CCW)
    {
        current_direction = DIRECTION_CCW;
        stepper_run_state = STEPPER_MOVING_CCW;
    }
    else if (StepperState == -1 && stepper_run_state != STEPPER_MOVING_CW)
    {
        current_direction = DIRECTION_CW;
        stepper_run_state = STEPPER_MOVING_CW;
    }
    else if (StepperState == 0)
    {
        Stepper_Stop();
        stepper_run_state = STEPPER_IDLE;
    }

    // Step execution with limit switch checking
    if (stepper_run_state == STEPPER_MOVING_CCW)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET)
        {
            Stepper_Step(1);
        }
        else
        {
            Stepper_Stop();
            stepper_run_state = STEPPER_IDLE;
            StepperState = 0;
        }
    }
    else if (stepper_run_state == STEPPER_MOVING_CW)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET)
        {
            Stepper_Step(1);
        }
        else
        {
            Stepper_Stop();
            stepper_run_state = STEPPER_IDLE;
            StepperState = 0;
        }
    }
}


void FanUpdate(float h)
{
    uint32_t now = HAL_GetTick();

    // Set timing based on humidity and Hgood
    if (Hgood) {
        tFanOn = 10000;
        tFanOFF = 20000;
    }
    if (h > 65) {
        tFanOn = 20000;
        tFanOFF = 15000;
    }
    else if (h < 60) {
        tFanOn = 10000;
        tFanOFF = 30000;
    }

    // Fan control logic
    if ((now - tFan >= tFanOn) && (FanState == 1)) {
        tFan = now;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // FAN OFF (relay HIGH)
        FanState = 0;
        char msg[] = "FAN OFF\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    }
    else if ((now - tFan >= tFanOFF) && (FanState == 0)) {
        tFan = now;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);    // FAN ON (relay LOW)
        FanState = 1;
        char msg[] = "FAN ON\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    }
}


void LBrelay(void)
{
    char msg[64];  // Buffer for UART message

    if ((f <= 99) && (!LBon)) {
        // Close relay (ON) -> HIGH
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);   // Relay HIGH = ON

        LBon = true;
        tLastLBon = HAL_GetTick();

        snprintf(msg, sizeof(msg), "Relay ON at %lu ms, Temp F=%.2f\r\n", tLastLBon, f);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    if ((f >= 105.5) && (LBon)) {
        // Open relay (OFF) -> LOW
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Relay LOW = OFF

        LBon = false;

        snprintf(msg, sizeof(msg), "Relay OFF, Temp F=%.2f\r\n", f);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}


void UpdateDisplay() {

  switch(current_screen) {
    case 1: Display_Screen1(h, t); break;
    case 2: Display_Screen2(hmax, hmin); break;
    case 3: Display_Screen3(tmax, tmin); break;
    case 4: Display_Screen4(StepperState); break;
    case 5: Display_Screen5(tReset); break;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    /*if (GPIO_Pin == GPIO_PIN_13) {
        static uint32_t last_press = 0;
        // Debounce with 300ms threshold
        if (HAL_GetTick() - last_press > 300) {
        	stepper_timer_flag = 0;  // Request stop
            last_press = HAL_GetTick();
        }
    }*/

	volatile uint32_t last_button_press = 0;
    if ((GPIO_Pin == GPIO_PIN_13) || (GPIO_Pin == GPIO_PIN_12 ))  {
        uint32_t now = HAL_GetTick();

        // Simple debounce check
        if (now - last_button_press > 300) {
            last_button_press = now;

            // Your original button handling code
            current_screen = (current_screen % 5) + 1;
            lcd_clear();
            UpdateDisplay();
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char uart_buf[64] = "System starting...\n\r" ;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, sizeof(uart_buf), 10);
  DHT11_Init(&htim1);

  /*
  uint32_t last_dht_read = 0;
  uint32_t last_dis5_read = 0;
  */

  scan_i2c_bus();

  lcd_init();


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorCommTask */
  SensorCommTaskHandle = osThreadNew(StartSensorCommTask, NULL, &SensorCommTask_attributes);

  /* creation of StepperTask */
  StepperTaskHandle = osThreadNew(StartStepperTask, NULL, &StepperTask_attributes);

  /* creation of FanTask */
  FanTaskHandle = osThreadNew(StartFanTask, NULL, &FanTask_attributes);

  /* creation of BulbTask */
  BulbTaskHandle = osThreadNew(StartBulbTask, NULL, &BulbTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  /*



	    // Read DHT11 every 5 seconds
	    if (HAL_GetTick() - last_dht_read >= 5000) {
	        last_dht_read = HAL_GetTick();

	        float temperature, humidity;
	        if (DHT11_ReadData(&temperature, &humidity)) {
	            // Format and send UART data (your existing code)
	            int len = snprintf(uart_buf, sizeof(uart_buf), "Temp: %.1fC, Hum: %.1f%%\r\n", temperature, humidity);
	            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, HAL_MAX_DELAY);

	            // Update min/max (your existing code)
	            h = humidity;
	            t = temperature;
	            f = (temperature * 9.0f / 5.0f) + 32.0f;
	            if (t < tmin) tmin = t;
	            if (t > tmax) tmax = t;
	            if (h < hmin) hmin = h;
	            if (h > hmax) hmax = h;

	            // Send to ESP32
	            char esp32_buf[100];
	            uint32_t seconds = (HAL_GetTick() - tReset) / 1000;
	            uint32_t days = seconds / 86400;
	            seconds %= 86400;
	            uint32_t hours = seconds / 3600;
	            seconds %= 3600;
	            uint32_t minutes = seconds / 60;
	            seconds %= 60;

	            char uptime_str[20];
	            sprintf(uptime_str, "%02lu:%02lu:%02lu:%02lu", days, hours, minutes, seconds);

	            const char *DoorState;
	            if (StepperState == 1)
	                DoorState = "Opening";
	            else if (StepperState == -1)
	                DoorState = "Closing";
	            else
	                DoorState = "Idle";

	            int esp_len = snprintf(esp32_buf, sizeof(esp32_buf),
	                                   "T:%.1fF H:%.1f%% Tmin:%.1f Tmax:%.1f Hmin:%.1f Hmax:%.1f Uptime:%s Door:%s\r\n",
	                                   t, h, tmin, tmax, hmin, hmax, uptime_str, DoorState);

	            HAL_UART_Transmit(&huart1, (uint8_t*)esp32_buf, esp_len, HAL_MAX_DELAY);
	        }
	        Led(h);
	        FanUpdate(h);
	        LBrelay();
	        UpdateDisplay();


	    }
	    if (current_screen == 5){
		    if (HAL_GetTick() - last_dis5_read >= 1000) {
		        last_dis5_read = HAL_GetTick();
		        UpdateDisplay();
		    }


	    }

	    StepperControl(h);  */


	}
	    /*for (int i = 0; i < 8; i++)
	    {
	        GPIOB->ODR = stepper_sequence[i];
	        HAL_Delay(500);  // Adjust delay to see step clearly
	    }*/



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Green_Led_Pin|Yellow_Led_Pin|Red_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEPPER_IN4_Pin|LB_relay_Pin|Fan_relay_Pin|STEPPER_IN1_Pin
                          |STEPPER_IN2_Pin|STEPPER_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : cw_sensor_Pin ccw_sensor_Pin */
  GPIO_InitStruct.Pin = cw_sensor_Pin|ccw_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_Led_Pin Yellow_Led_Pin Red_Led_Pin */
  GPIO_InitStruct.Pin = Green_Led_Pin|Yellow_Led_Pin|Red_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER_IN4_Pin LB_relay_Pin Fan_relay_Pin STEPPER_IN1_Pin
                           STEPPER_IN2_Pin STEPPER_IN3_Pin */
  GPIO_InitStruct.Pin = STEPPER_IN4_Pin|LB_relay_Pin|Fan_relay_Pin|STEPPER_IN1_Pin
                          |STEPPER_IN2_Pin|STEPPER_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Us_Butt_Pin */
  GPIO_InitStruct.Pin = Us_Butt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Us_Butt_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorCommTask */
/**
* @brief Function implementing the SensorCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorCommTask */
void StartSensorCommTask(void *argument)
{
  /* USER CODE BEGIN StartSensorCommTask */
	  float temperature = 0, humidity = 0;
	  char uart_buf[64];
  /* Infinite loop */
  for(;;)
  {
      if (DHT11_ReadData(&temperature, &humidity)) {
          // Format and send UART data (your existing code)
          int len = snprintf(uart_buf, sizeof(uart_buf), "Temp: %.1fC, Hum: %.1f%%\r\n", temperature, humidity);
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, HAL_MAX_DELAY);

          // Update min/max (your existing code)
          h = humidity;
          t = temperature;
          f = (temperature * 9.0f / 5.0f) + 32.0f;
          if (t < tmin) tmin = t;
          if (t > tmax) tmax = t;
          if (h < hmin) hmin = h;
          if (h > hmax) hmax = h;

          // Send to ESP32
          char esp32_buf[100];
          uint32_t seconds = (HAL_GetTick() - tReset) / 1000;
          uint32_t days = seconds / 86400;
          seconds %= 86400;
          uint32_t hours = seconds / 3600;
          seconds %= 3600;
          uint32_t minutes = seconds / 60;
          seconds %= 60;

          char uptime_str[20];
          sprintf(uptime_str, "%02lu:%02lu:%02lu:%02lu", days, hours, minutes, seconds);

          const char *DoorState;
          if (StepperState == 1)
              DoorState = "Opening";
          else if (StepperState == -1)
              DoorState = "Closing";
          else
              DoorState = "Idle";

          int esp_len = snprintf(esp32_buf, sizeof(esp32_buf),
                                 "T:%.1fF H:%.1f%% Tmin:%.1f Tmax:%.1f Hmin:%.1f Hmax:%.1f Uptime:%s Door:%s\r\n",
                                 t, h, tmin, tmax, hmin, hmax, uptime_str, DoorState);

          HAL_UART_Transmit(&huart1, (uint8_t*)esp32_buf, esp_len, HAL_MAX_DELAY);

          // Resume other tasks to process data
          osThreadResume(FanTaskHandle);
          osThreadResume(BulbTaskHandle);
          osThreadResume(LEDTaskHandle);
          osThreadResume(DisplayTaskHandle);
      }
    osDelay(5000);
  }
  /* USER CODE END StartSensorCommTask */
}

/* USER CODE BEGIN Header_StartStepperTask */
/**
* @brief Function implementing the StepperTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStepperTask */
void StartStepperTask(void *argument)
{
  /* USER CODE BEGIN StartStepperTask */
  /* Infinite loop */
  for(;;)
  {
	  StepperControl(h);
      osDelay(1);
  }
  /* USER CODE END StartStepperTask */
}

/* USER CODE BEGIN Header_StartFanTask */
/**
* @brief Function implementing the FanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFanTask */
void StartFanTask(void *argument)
{
  /* USER CODE BEGIN StartFanTask */
  /* Infinite loop */
  for(;;)
  {
	  FanUpdate(h);


	  osThreadSuspend(FanTaskHandle);
  }
  /* USER CODE END StartFanTask */
}

/* USER CODE BEGIN Header_StartBulbTask */
/**
* @brief Function implementing the BulbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBulbTask */
void StartBulbTask(void *argument)
{
  /* USER CODE BEGIN StartBulbTask */
  /* Infinite loop */
  for(;;)
  {
	  LBrelay();

	  osThreadSuspend(BulbTaskHandle);
  }
  /* USER CODE END StartBulbTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
	  Led(h);

	  osThreadSuspend(LEDTaskHandle);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
  for(;;)
  {
	  UpdateDisplay();

	  osThreadSuspend(DisplayTaskHandle);


  }
  /* USER CODE END StartDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
