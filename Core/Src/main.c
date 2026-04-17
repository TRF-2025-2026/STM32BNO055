/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "accel.h"
#include "i2c.h"
#include "eeprom.h"
#include <stdio.h>
#include <stdbool.h>
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
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* --- MEMORY SAFETY FIXES --- */
//volatile uint8_t dma_rx_complete = 0;
volatile uint8_t imu_reading[6]; // Made volatile for DMA safety

int16_t raw_x, raw_y, raw_z;
float acc_x, acc_y, acc_z;

uint8_t euler_reading[6];
uint16_t bno_offsets[11];
int16_t raw_yaw, raw_roll, raw_pitch;
float yaw, roll, pitch;

uint8_t calib_status = 0;
bool is_calibrated = false;

/* --- STATE MACHINE TRACKER --- */
HAL_StatusTypeDef sys_status = HAL_OK;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	while (HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10)) {
	};
	return ch;
}

void I2C_Bus_Recovery(void); // Prototype for our new recovery function

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Manually pulses the I2C clock to un-stick the BNO055
 * NOTE: Adjust PB8 and PB9 if your I2C pins are different (e.g., PB6/PB7)
 */
void I2C_Bus_Recovery(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Enable GPIO Clock for Port B
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 2. Temporarily configure SCL (PB8) and SDA (PB9) as standard GPIOs
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // Open Drain
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Set both lines HIGH initially
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    // 4. Send 9 Clock Pulses on SCL to flush stuck bytes out of the BNO055
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL LOW
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL HIGH
        HAL_Delay(1);
    }

    // 5. Generate a manual I2C STOP condition (SCL High, then SDA Low -> High)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL HIGH
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // SDA LOW
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // SDA HIGH
    HAL_Delay(1);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

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
	MX_DMA_Init();

    /* --- CRITICAL BOOT SEQUENCE FIX --- */
    I2C_Bus_Recovery(); // Free the bus before the HAL tries to grab it!

	MX_I2C1_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	printf("\r\n--- Terminal Link Active ---\r\n");

	/* 1. Initialize EEPROM Emulation Logic */
	// This checks the headers of Sector 2 and 3 [cite: 150, 324]
	if (EE_Init() != HAL_OK) {
	    printf("EEPROM Init Failed!\r\n"); // [cite: 340, 541]
	} else {
	    /* 2. Check for saved calibration offsets in Flash */
	    uint16_t first_val;
	    // 0x0001 is our Virtual Address for Accel_X [cite: 156, 175]
	    if (EE_ReadVariable(0x0001, &first_val) == 0) {
	        printf("Saved Calibration Found! Loading...\r\n");
	        for(int i = 0; i < 11; i++) {
	            // Read all 11 variables (22 bytes) [cite: 156, 585]
	            EE_ReadVariable(0x0001 + i, &bno_offsets[i]);
	        }
	        // Load offsets into the sensor registers [cite: 54, 157]
	        BNO055_Set_Offsets(&hi2c1, bno_offsets);
	    }
	}

	/* 3. Standard BNO055 Startup */
	sys_status = BNO055_Init_I2C(&hi2c1);
	if (sys_status != HAL_OK) {
	    printf("ERROR: BNO055 Init Failed!\r\n");
	} else {
	    printf("BNO055 Initialized Successfully.\r\n");
	}

	HAL_Delay(800); // Wait for NDOF fusion

//	dma_rx_complete = 0;
	sys_status = GetAccelData(&hi2c1, (uint8_t*)imu_reading);
	/* USER CODE END 2 */
	/* Infinite loop */
	/* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
	    /* ======================================================================
	       STATE 1: NORMAL OPERATION (DMA Data Processing)
	       ====================================================================== */
	    if (sys_status == HAL_OK)
	    {
//	      dma_rx_complete = 0; // Clear flag immediately to allow next DMA cycle

	      // --- 1. Process Accelerometer Data (from DMA Buffer) ---
	      raw_x = (int16_t) ((imu_reading[1] << 8) | imu_reading[0]);
	      raw_y = (int16_t) ((imu_reading[3] << 8) | imu_reading[2]);
	      raw_z = (int16_t) ((imu_reading[5] << 8) | imu_reading[4]);

	      acc_x = (float) raw_x / 100.0f;
	      acc_y = (float) raw_y / 100.0f;
	      acc_z = (float) raw_z / 100.0f;

	      // --- 2. Read & Process Euler Angles (Standard I2C Read) ---
	      // Uses a 100ms timeout to prevent software lockup if the bus glitche [cite: 614]
	      if (HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO << 1, BNO055_EUL_HEADING_LSB,
	                           I2C_MEMADD_SIZE_8BIT, euler_reading, 6, 100) == HAL_OK)
	      {
	        raw_yaw   = (int16_t)((euler_reading[1] << 8) | euler_reading[0]);
	        raw_roll  = (int16_t)((euler_reading[3] << 8) | euler_reading[2]);
	        raw_pitch = (int16_t)((euler_reading[5] << 8) | euler_reading[4]);

	        yaw   = (float) raw_yaw / 16.0f;
	        roll  = (float) raw_roll / 16.0f;
	        pitch = (float) raw_pitch / 16.0f;
	      }

	      // --- 3. Auto-Save Calibration Offsets (EEPROM Emulation) ---
	      // Check the CALIB_STAT register (0x35) to see if fusion is complete [cite: 312]
	      if (HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR_LO << 1, 0x35, 1, &calib_status, 1, 100) == HAL_OK)
	      {
	        // 0xFF indicates Sys, Gyro, Accel, and Mag are all at Level 3 calibration
	        if (calib_status == 0xFF && !is_calibrated)
	        {
	          printf("Calibration 100%%! Persisting offsets to Flash...\r\n");

	          // Read 22 bytes of offsets (0x55 to 0x6A) into our local array
	          BNO055_Get_Offsets(&hi2c1, bno_offsets);

	          HAL_I2C_DeInit(&hi2c1);

	          // Store variables in Flash. Each update uses 4 bytes of Flash [cite: 164, 585]
	          for(int i = 0; i < 11; i++)
	          {
	            EE_WriteVariable(0x0001 + i, bno_offsets[i]);
	          }

	          is_calibrated = true; // Flag to prevent saving again and wearing out Flash [cite: 458, 560]
	          printf("Offsets Saved Successfully.\r\n");
	          I2C_Bus_Recovery();
	          MX_I2C1_Init();
	          sys_status = GetAccelData(&hi2c1, (uint8_t*)imu_reading);
	        }
	      }

	      // --- 4. Loop Pacing & Next Cycle Trigger ---
	      HAL_Delay(50); // Small delay so we don't saturate the I2C bus

	      // Start the NEXT DMA background read
	      sys_status = GetAccelData(&hi2c1, (uint8_t*)imu_reading);
	    }

	    /* ======================================================================
	       STATE 2: ERROR RECOVERY (The HAL_BUSY & Hardware Hang Killer)
	       ====================================================================== */
	    else if (sys_status != HAL_OK)
	    {
	      printf("I2C Error or Bus Busy. Starting Recovery Procedure...\r\n");
	      HAL_DMA_Abort(&hdma_i2c1_rx);

	      // 1. Fully reset the I2C peripheral state
	      HAL_I2C_DeInit(&hi2c1);
	      hi2c1.State = HAL_I2C_STATE_RESET;
	      __HAL_UNLOCK(&hi2c1);

	      // 2. Perform Physical Bus Recovery (Clock Pulsing) to un-stick SDA line
	      I2C_Bus_Recovery();

	      // 3. Re-initialize the I2C peripheral [cite: 340]
	      MX_I2C1_Init();

	      // 4. Attempt to restart the DMA cycle
//	      dma_rx_complete = 0;
	      sys_status = GetAccelData(&hi2c1, (uint8_t*)imu_reading);

	      HAL_Delay(100); // Give the bus a moment to stabilize
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
void MX_I2C1_Init(void) {

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
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(2);
	__HAL_RCC_I2C1_RELEASE_RESET();
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	if (hi2c->Instance == I2C1) {
//		dma_rx_complete = 1; // Signal the main loop
//	}
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
