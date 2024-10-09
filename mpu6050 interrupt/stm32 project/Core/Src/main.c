/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include<string.h>
#include<stdio.h>
#include<mpu_6050.h>
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const int timer_prescaler = 99;
int timer_count = 0;
uint8_t is_data_ready = 0;
uint16_t dev_address = 0x68 << 1;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart3, (uint8_t*)"USART initialized\n", strlen("USART initialized\n"), HAL_MAX_DELAY);

  // initialize the MPU 6050
  /* MPU6050 Initialization sequence is as follows
   * 1. Setup the power management bits. Wake it up from sleep
   * 2. Set configuration bits. DLPF is set in this register
   * 3. Set SMPLRT_DIV bits. This is used to set the sampling rate
   * 4. Set INT_EN bit. This is used to set the interrupt enable bit
   * 5. Set INT_PIN_CFG bits. This is used to configure the interrupt pin itself.
   */

  uint8_t reg_value = 0;
  HAL_I2C_Mem_Read_IT(&hi2c1, dev_address, MPU6050_POWER_MGMT_REG, 1, &reg_value, 1);
  HAL_Delay(5);
  uint8_t value2write = 0b00000001;
  HAL_I2C_Mem_Write_IT(&hi2c1, dev_address, MPU6050_POWER_MGMT_REG, 1, &value2write, 1);
  HAL_Delay(5);
//  HAL_I2C_Mem_Read_IT(&hi2c1, dev_address, MPU6050_POWER_MGMT_REG, 1, &reg_value, 1);
//  HAL_Delay(5);

  value2write = 1;
  HAL_I2C_Mem_Write_IT(&hi2c1, dev_address, MPU6050_CONFIG_REG, 1, &value2write, 1);
  HAL_Delay(5);

  value2write = 4;
  HAL_I2C_Mem_Write_IT(&hi2c1, dev_address, MPU6050_SMPLRT_DIV_REG, 1, &value2write, 1);
  HAL_Delay(5);

  value2write = 0b10100000;
  HAL_I2C_Mem_Write_IT(&hi2c1, dev_address, MPU6050_INT_PIN_CFG_REG, 1, &value2write, 1);
  HAL_Delay(5);

  value2write = 1;
  HAL_I2C_Mem_Write_IT(&hi2c1, dev_address, MPU6050_INT_EN_REG, 1, &value2write, 1);
  HAL_Delay(5);


//  HAL_I2C_Mem_Write_IT(&hi2c1, 0x21<<1, 0x33, 1, "Hear", strlen("Hear"));
//  HAL_Delay(5);
//  HAL_I2C_Mem_Read_IT(&hi2c1, dev_address, MPU6050_INT_EN_REG, 1, &reg_value, 1);
//  HAL_Delay(5);

  // Read the interrupt register of MPU6050 once
	HAL_I2C_Mem_Read_IT(&hi2c1, dev_address, MPU6050_INT_STATUS_REG, 1, &reg_value, 1);
	HAL_Delay(10);

  //HAL_Delay(2000);
  uint8_t gyrox_low, gyroy_low;
  //read_from_register(hi2c1, 0x44, &gyrox_low);
  //read_from_register(hi2c1, 0x45, &gyroy_low);

  //HAL_UART_Transmit(&huart3, (uint8_t*)gyroy_low, strlen(gyroy_low), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Start the timer
  HAL_TIM_Base_Start(&htim1);

  int nor;
  nor = 1;
  while (1)
  {
    /* USER CODE END WHILE */
	  char counter_value[5];
	  //read_from_register(hi2c1, 0x45, &gyroy_low);
	  //sprintf(counter_value, "%lu\n", (timer_count*(timer_prescaler+1)/64000));
	  //HAL_UART_Transmit(&huart3, (uint8_t*)counter_value, strlen(counter_value), HAL_MAX_DELAY);
	  //HAL_UART_Transmit(&huart3, (uint8_t*)gyroy_low, strlen(gyroy_low), HAL_MAX_DELAY);

	  //HAL_Delay(0.01);
    /* USER CODE BEGIN 3 */

	  if(is_data_ready == 1){
		  data_ready_handler();
		  is_data_ready = 0;

		  sprintf(counter_value, "%lu\n", (timer_count*(timer_prescaler+1)/64000));
		  //sprintf(counter_value, "%d \n", timer_count);
		  HAL_UART_Transmit(&huart3, (uint8_t*)counter_value, strlen(counter_value), HAL_MAX_DELAY);
	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = timer_prescaler;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/* Interrupt Handler for EXTI line 0 (PA0) */
void EXTI0_IRQHandler(void) {
  /* Handle the interrupt */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/* This function is called inside the HAL library when an EXTI event occurs */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	is_data_ready = 1;
//  if (GPIO_Pin == GPIO_PIN_0) {
//    // Handle the interrupt here (e.g., toggle an LED)
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  // Example: Toggle an LED on PB0
//  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* setup the GPIO interrupt */
  /* Configure GPIO pin PA0 as input with external interrupt (falling edge) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;             // Pin PA0
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // No pull-up/pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable and set EXTI line 0 Interrupt in the NVIC (PA0 is connected to EXTI0) */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* Configure GPIO pin PB0 as output for the LED */
  GPIO_InitStruct.Pin = GPIO_PIN_0;             // Pin PB0
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Output push-pull mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // No pull-up/pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Low frequency
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

void data_ready_handler(){
	// read the data from the registers
	timer_count = htim1.Instance->CNT;
	htim1.Instance->CNT = 0;
	//set_sampling_rate(&hi2c1, 20);
	uint8_t int_status;
	HAL_I2C_Mem_Read_IT(&hi2c1, dev_address, MPU6050_INT_STATUS_REG, 1, &int_status, 1);
	HAL_Delay(1);

  //HAL_I2C_Mem_Write_IT(&hi2c1, 0x21<<1, 0x33, 1, "interrupted", strlen("interrupted"));
  //HAL_Delay(5);

	// clear the data ready register in the MPU6050
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_UART_Transmit(&huart3, (uint8_t*)("Error Occurred"), strlen("Error Occurred"), HAL_MAX_DELAY);
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
