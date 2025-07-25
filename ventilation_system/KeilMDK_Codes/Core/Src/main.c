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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include "stdio.h"     // For sprintf() function
#include <string.h>
#include "LCD1602.h"   // Custom library for controlling 1602 LCD
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PWM duty cycle value for fan (0-100). Controls fan speed.

uint16_t cmpv = 80;
// UART output refresh flag. Set to 1 to send PWM value over UART.
uint8_t rf = 0;
// latest measured temperature in Celsius
float temperature = 0.0f;
// Detects significant temperature changes for LCD update
float previous_temp = -1.0f;
// Operation mode: 0 = Auto, 1 = Manual (set by button interrupts)
uint8_t mode = 0;
// Fan direction: 0 = Stop, 1 = Forward, 2 = Reverse (set by button interrupts)
uint8_t fan_dir = 0;
// Stores the time when manual mode was entered (for timeout, if needed)
uint32_t manual_timer = 0;
// Used for periodic tasks (temperature polling every 300ms)
uint32_t last_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  
  * @retval 
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
		char str[8];  // Temporary string buffer for UART transmission of PWM value
		char lcd_line1[16];  // First line text for LCD
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Resets of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configures the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initializes all configured peripherals */
		MX_GPIO_Init();          // Initialize GPIO pins
    MX_TIM3_Init();          // Initialize TIM3 for fan PWM
    MX_USART1_UART_Init();   // Initialize UART1
    MX_ADC1_Init();          // Initialize ADC
    MX_TIM2_Init();          // Initialize TIM2 for buzzer PWM
  /* USER CODE BEGIN 2 */
	
	/* USER CODE BEGIN 2 */

 

	   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Start PWM on TIM2_CH1 (PA0 for buzzer)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start PWM on TIM3_CH2 (fan)

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmpv); // Set initial fan PWM value

    // Stops motor by clearing direction pins
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

    LCD_Init(); // Initializes LCD display

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Every 300ms, reads temperature and updates system state
    if (HAL_GetTick() - last_tick >= 300) { // 300 ms interval
        last_tick = HAL_GetTick();

        // --- Read Temperature from LM35 sensor via ADC ---

        HAL_ADC_Start(&hadc1);                          // Start ADC conversion

        // --- Software-based ADC wait loop (instead of HAL_ADC_PollForConversion) ---
        uint32_t adc_wait_start = HAL_GetTick();
        while ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) != HAL_ADC_STATE_REG_EOC) {
            if (HAL_GetTick() - adc_wait_start > 100) break; // Timeout after 100ms
        }
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);    // Read ADC value
        float voltage = (adc_val * 5.0f) / 4095.0f;      // Convert to voltage (assuming 5V ref)
        temperature = voltage * 100.0f / 2;              // Convert voltage to temperature (LM35)

        // --- Update LCD if temperature changes significantly (>=0.5°C) ---
        if (fabs(temperature - previous_temp) >= 0.5f)
        {
            sprintf(lcd_line1, "Temp: %.0f C", temperature); // Format LCD text
            LCD_ShowString(0, 0, lcd_line1);                 // Display on LCD
            previous_temp = temperature;                     // Store current value
        }

        // --- Auto Mode: Fan and indicators controlled by temperature ---
        if (mode == 0)
        {
            if (temperature < 10.0f) {
                // Below 10°C: Fan always turns right, red LED ON, buzzer ON
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);    // Forward direction
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
                fan_dir = 1;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // Red LED ON
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Green LED OFF
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // Buzzer ON
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);     // Buzzer PWM duty

            } else if (temperature > 20.0f) {
                // Above 20°C: Fan always turns left, red LED ON, buzzer ON
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);    // Reverse direction
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
                fan_dir = 2;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);   // Red LED ON
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Green LED OFF
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // Buzzer ON
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);     // Buzzer PWM duty
                
            } else {
                // 10°C <= temp <= 20°C: Stop fan, green LED ON, buzzer OFF
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // Stop motor
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
                fan_dir = 0;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   // Green LED ON
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Red LED OFF
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Buzzer OFF
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);       // Stop buzzer
            }
        }
        // --- Manual Mode: Fan direction set by user buttons ---

        else
        {
            if (fan_dir == 1)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   // Forward
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            }
            else if (fan_dir == 2)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // Reverse
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // Stop
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
            }
        }

        // --- Transmit PWM value over UART if refresh flag is set ---

        if (rf == 1)
        {
            char msg[16];
            sprintf(msg, "PWM:%d\r\n", cmpv);
            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
            {
                HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
            }
            rf = 0; // Clear refresh flag
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * 
  */
 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   * 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE BEGIN 4 */

// --- External Interrupt Callback for Button Handling ---

// Handles button presses for manual mode and PWM adjustment
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0) // FWD button: Set manual mode, forward direction
    {

        mode = 1;
        fan_dir = 1;
        manual_timer = HAL_GetTick();
        rf = 1;
    }

    else if(GPIO_Pin == GPIO_PIN_1) // REV button: Set manual mode, reverse direction
    {
        mode = 1;
        fan_dir = 2;
        manual_timer = HAL_GetTick();
        rf = 1;

    }
    else if(GPIO_Pin == GPIO_PIN_2) // STOP button: Set manual mode, stop fan

    {
        mode = 1;
        fan_dir = 0;
        manual_timer = HAL_GetTick();
        rf = 1;
    }

    else if(GPIO_Pin == GPIO_PIN_3) // PWM Increase button

    {
        if(cmpv < 100) cmpv += 10;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmpv);
        rf = 1;

    }
    else if(GPIO_Pin == GPIO_PIN_4) // PWM Decrease button

    {

        if(cmpv > 0) cmpv -= 10;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmpv);
        rf = 1;
        
    }
}

/* USER CODE END 4 */

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
