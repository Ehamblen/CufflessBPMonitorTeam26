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

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	MAX30102_OK = 0,
	MAX30102_ERROR_I2C,
	MAX30102_ERROR_NOT_FOUND,
	MAX30102_ERROR_BAD_ID,
	MAX30102_ERROR_CONFIG,
	MAX30102_ERROR_FIFO
} MAX30102_Status;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t red1; // MAX30102
    uint32_t ir1; // MAX30102
    uint32_t green2; // MAX30101
} PPG_DataPoint;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* BELOW: REGISTER MAP FOR MAX30102 */

#define MAX30102_ADDR (0x57 << 1) // 7-bit I2C address, with required left shift

/* Status */
#define MAX30102_REG_INT_STATUS1 0x00
#define MAX30102_REG_INT_STATUS2 0x01
#define MAX30102_REG_INT_ENABLE1 0x02
#define MAX30102_REG_INT_ENABLE2 0x03

/* FIFO */
#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_OVF_COUNTER 0x05
#define MAX30102_REG_FIFO_RD_PTR 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_FIFO_CONFIG 0x08

/* Configuration */
#define MAX30102_REG_MODE_CONFIG 0x09
#define MAX30102_REG_SPO2_CONFIG 0x0A
#define MAX30102_REG_LED1_PA 0x0C
#define MAX30102_REG_LED2_PA 0x0D

/* Identification */
#define MAX30102_REG_REV_ID 0xFE
#define MAX30102_REG_PART_ID 0xFF
#define MAX30102_EXPECTED_ID 0x15

/* MAX30101 additional registers */
#define MAX30101_REG_LED3_PA          0x0E
#define MAX30101_REG_LED4_PA          0x0F
#define MAX30101_REG_MULTI_LED_CTRL1  0x11
#define MAX30101_REG_MULTI_LED_CTRL2  0x12

/* MAX30101 Green LED Sensor configuration */
#define SENSOR2_GREEN_CURRENT    0x1F
/* Temporary threshold for initial green testing */
#define SENSOR2_GREEN_THRESHOLD  0

#define PPG_BUFFER_SIZE 100

#define EVENT_THRESHOLD 10000 // arbitrary

// VARYING CURRENT FOR MAX30102 Sensors
/* Sensor 1: Finger */
#define SENSOR1_RED_CURRENT  0x1F
#define SENSOR1_IR_CURRENT   0x1F

/* Forced Counter Delay */
#define WARMUP_SAMPLES 20
#define MAX_PPG_SAMPLES 1000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* Global Sample Variables for Debug View */
uint32_t ppg_red1 = 0;
uint32_t ppg_ir1 = 0;
uint32_t ppg_green2 = 0;

/* Error Case Store */
MAX30102_Status sensor_status1 = MAX30102_OK;
MAX30102_Status sensor_status2 = MAX30102_OK;
volatile uint32_t i2c_error_count1 = 0;
volatile uint32_t i2c_last_error1 = 0;
volatile uint32_t i2c_error_count2 = 0;
volatile uint32_t i2c_last_error2 = 0;

/* Debug Variable for MAX30102 Connection */
HAL_StatusTypeDef max30102_status1;
HAL_StatusTypeDef max30102_status2;

/* Sample & FIFO Variables */
uint32_t ppg_sample_count = 0;
uint32_t ppg_last_sample_time = 0;

uint32_t ppg_fifo_error_count1 = 0;
uint32_t ppg_fifo_error_count2 = 0;

/* Flag for New MAX30102 Sample */
uint8_t sensor1_new_sample = 0;
uint8_t sensor2_new_sample = 0;

/* Storing PPG data */
PPG_DataPoint ppg_buffer[PPG_BUFFER_SIZE];
uint16_t ppg_write_index = 0;

/* Event Flag */
volatile uint8_t max30102_data_ready1 = 0;
volatile uint8_t max30102_data_ready2 = 0;
volatile uint32_t interrupt_count1 = 0;
volatile uint32_t interrupt_count2 = 0;

uint8_t pulse1_present = 0;
uint8_t pulse2_present = 0;

/* Forced Delay */
uint8_t sensor1_warmup_count = 0;
uint8_t sensor2_warmup_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */

HAL_StatusTypeDef MAX30102_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t value);
HAL_StatusTypeDef MAX30102_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *value);

HAL_StatusTypeDef MAX30102_Reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *hi2c);

uint8_t MAX30102_SamplesAvailable(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MAX30102_ReadSample(I2C_HandleTypeDef *hi2c, uint32_t *red,
		uint32_t *ir);

void PPG_HandleSample(uint8_t sensor_number, uint32_t red, uint32_t ir, uint32_t green); // General Function for Sample Handling
void PPG_StoreSample(uint32_t timestamp_ms, uint32_t red1, uint32_t ir1, uint32_t green2); // Store Sample

/* Interrupt */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); // Interrupt Callback
void PPG_HandleInterrupt(I2C_HandleTypeDef *hi2c, uint8_t sensor_number);

/* Varing Current */
HAL_StatusTypeDef MAX30102_SetLEDCurrent(I2C_HandleTypeDef *hi2c, uint8_t red_current, uint8_t ir_current);

/* Initialization for MAX30101 */
HAL_StatusTypeDef MAX30101_InitGreen(I2C_HandleTypeDef *hi2c);

/* Read Green Light Sensor Data */
HAL_StatusTypeDef MAX30101_ReadGreen(I2C_HandleTypeDef *hi2c, uint32_t *green);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * SWV ITM Data Console
 *
 * For CSV Exporting
 *
 */
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USB_PCD_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	/* REAL Communication with MAX30102 */

	// Register Checks
	uint8_t id1;
	uint8_t id2;

	/* Sensor 1 */

	max30102_status1 = HAL_I2C_IsDeviceReady(&hi2c1,
	MAX30102_ADDR, 3, 100);

	if (max30102_status1 != HAL_OK) {
		sensor_status1 = MAX30102_ERROR_NOT_FOUND;
		Error_Handler();
	}

	if (MAX30102_ReadReg(&hi2c1, MAX30102_REG_PART_ID, &id1) != HAL_OK) {
		sensor_status1 = MAX30102_ERROR_I2C;
		Error_Handler();
	}

	if (id1 != MAX30102_EXPECTED_ID) {
		sensor_status1 = MAX30102_ERROR_BAD_ID;
		Error_Handler();
	}

	if (MAX30102_Init(&hi2c1) != HAL_OK) {
		sensor_status1 = MAX30102_ERROR_CONFIG;
		Error_Handler();
	}

	/* Sensor 2 */

	max30102_status2 = HAL_I2C_IsDeviceReady(&hi2c2,
	MAX30102_ADDR, 3, 100);

	if (max30102_status2 != HAL_OK) {
		sensor_status2 = MAX30102_ERROR_NOT_FOUND;
		Error_Handler();
	}

	if (MAX30102_ReadReg(&hi2c2, MAX30102_REG_PART_ID, &id2) != HAL_OK) {
		sensor_status2 = MAX30102_ERROR_I2C;
		Error_Handler();
	}

	if (id2 != MAX30102_EXPECTED_ID) { // SAME ID FOR MAX30101, 0x15
		sensor_status2 = MAX30102_ERROR_BAD_ID;
		Error_Handler();
	}

	if (MAX30101_InitGreen(&hi2c2) != HAL_OK) {
	    sensor_status2 = MAX30102_ERROR_CONFIG;
	    Error_Handler();
	}


	/* Clear any existing interrupt */
	uint8_t dummy_status;

	/* Clear Sensor 1 */
	MAX30102_ReadReg(&hi2c1, MAX30102_REG_INT_STATUS1, &dummy_status);

	MAX30102_ReadReg(&hi2c1, MAX30102_REG_INT_STATUS2, &dummy_status);

	/* Clear Sensor 2 */
	MAX30102_ReadReg(&hi2c2, MAX30102_REG_INT_STATUS1, &dummy_status);

	MAX30102_ReadReg(&hi2c2, MAX30102_REG_INT_STATUS2, &dummy_status);

	// Clear Interrupt Pins
	__HAL_GPIO_EXTI_CLEAR_IT(DRDY_1_Pin);
	__HAL_GPIO_EXTI_CLEAR_IT(DRDY_2_Pin);

	/*
	 *
	 * SUCCESSFUL INITIALIZATION OF MAX30102
	 *
	 * Clock now synchronized with SWV ITM Data Console
	 */

	// Header Line for CSV
	printf("Timestamp (ms),IR-1 PPG,GREEN PPG\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	while (1) { // While Loop to Continuously Collect + Process Samples
		// Sensor 1 Sampling
		if (max30102_data_ready1) {
			max30102_data_ready1 = 0;

			PPG_HandleInterrupt(&hi2c1, 1);
		}

		// Sensor 2 Sampling
		if (max30102_data_ready2) {
			max30102_data_ready2 = 0;

			PPG_HandleInterrupt(&hi2c2, 2);
		}
	}
}

/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1
			| RCC_PERIPHCLK_I2C2;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00201D2B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00201D2B;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

	/* USER CODE BEGIN USB_Init 0 */

	/* USER CODE END USB_Init 0 */

	/* USER CODE BEGIN USB_Init 1 */

	/* USER CODE END USB_Init 1 */
	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_Init 2 */

	/* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin | LD9_Pin
					| LD10_Pin | LD8_Pin | LD6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
	 LD7_Pin LD9_Pin LD10_Pin LD8_Pin
	 LD6_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin
			| LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin */
	GPIO_InitStruct.Pin = MEMS_INT3_Pin | MEMS_INT4_Pin | MEMS_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DRDY_1_Pin DRDY_2_Pin */
	GPIO_InitStruct.Pin = DRDY_1_Pin | DRDY_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

////
////
////
////
/* Write & Read Functions */
HAL_StatusTypeDef MAX30102_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(hi2c,
	MAX30102_ADDR,
	reg,
	I2C_MEMADD_SIZE_8BIT,
	&value,
	1,
	100);
}

HAL_StatusTypeDef MAX30102_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(hi2c,
	MAX30102_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT,
	value,
	1,
	100);
}

/* Reset Function */
HAL_StatusTypeDef MAX30102_Reset(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status;
	uint8_t mode;
	uint32_t timeout;

	/* Reset Bit */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_MODE_CONFIG, 0x40);

	if (status != HAL_OK)
		return status;

	timeout = HAL_GetTick();

	do {
		status = MAX30102_ReadReg(hi2c, MAX30102_REG_MODE_CONFIG, &mode);

		if (status != HAL_OK)
			return status;

		if ((mode & 0x40) == 0)
			return HAL_OK;

	} while ((HAL_GetTick() - timeout) < 100);

	return HAL_TIMEOUT;
}

/* MAX30102 Initialization */
HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status;

	status = MAX30102_Reset(hi2c);

	if (status != HAL_OK) {
		return status;
	}

	/* Clear FIFO write pointer */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_WR_PTR, 0x00);

	if (status != HAL_OK) {
		return status;
	}

	/* Clear FIFO overflow counter */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_OVF_COUNTER, 0x00);

	if (status != HAL_OK) {
		return status;
	}

	/* Clear FIFO read pointer */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_RD_PTR, 0x00);

	if (status != HAL_OK) {
		return status;
	}

	/* Configure FIFO */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_CONFIG, 0x10);

	if (status != HAL_OK) {
		return status;
	}

	/* Sp02 Configuration
	 *
	 * Originally 0x27
	 *
	 */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_SPO2_CONFIG, 0x27);

	if (status != HAL_OK) {
		return status;
	}

	/* Set Red LED current */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_LED1_PA, 0x1F);

	if (status != HAL_OK) {
		return status;
	}

	/* Set IR LED current */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_LED2_PA, 0x1F);

	if (status != HAL_OK) {
		return status;
	}

	/* Enable PPG Ready interrupt */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_INT_ENABLE1, 0x40);

	if (status != HAL_OK) {
		return status;
	}

	/* Start SpO2 mode: Red + IR */
	status = MAX30102_WriteReg(hi2c, MAX30102_REG_MODE_CONFIG, 0x03);

	return status;

}

/* MAX30101 (GREEN) Initialization */

HAL_StatusTypeDef MAX30101_InitGreen(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    /* Reset Sensor 2 */
    status = MAX30102_Reset(hi2c);
    if (status != HAL_OK)
        return status;

    /* Configure FIFO */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_FIFO_CONFIG, 0x10);
    if (status != HAL_OK)
        return status;

    /* 100 SPS, 18-bit ADC, 411 us pulse width */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_SPO2_CONFIG, 0x27);
    if (status != HAL_OK)
        return status;

    /* Disable Red LED current */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_LED1_PA, 0x00);
    if (status != HAL_OK)
        return status;

    /* Disable IR LED current */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_LED2_PA, 0x00);
    if (status != HAL_OK)
        return status;

    /* Configure both Green LED current drivers */
    status = MAX30102_WriteReg(hi2c,
        MAX30101_REG_LED3_PA, SENSOR2_GREEN_CURRENT);
    if (status != HAL_OK)
        return status;

    status = MAX30102_WriteReg(hi2c,
        MAX30101_REG_LED4_PA, SENSOR2_GREEN_CURRENT);
    if (status != HAL_OK)
        return status;

    /* Enable Green in SLOT1; disable SLOT2 */
    status = MAX30102_WriteReg(hi2c,
        MAX30101_REG_MULTI_LED_CTRL1, 0x03);
    if (status != HAL_OK)
        return status;

    /* Disable SLOT3 and SLOT4 */
    status = MAX30102_WriteReg(hi2c,
        MAX30101_REG_MULTI_LED_CTRL2, 0x00);
    if (status != HAL_OK)
        return status;

    /* Reset FIFO pointers */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (status != HAL_OK)
        return status;

    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_OVF_COUNTER, 0x00);
    if (status != HAL_OK)
        return status;

    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (status != HAL_OK)
        return status;

    /* Enable PPG Ready interrupt */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_INT_ENABLE1, 0x40);
    if (status != HAL_OK)
        return status;

    /* Start Multi-LED Mode */
    status = MAX30102_WriteReg(hi2c,
        MAX30102_REG_MODE_CONFIG, 0x07);

    return status;
}

/* Green Light Data Reading */
HAL_StatusTypeDef MAX30101_ReadGreen(I2C_HandleTypeDef *hi2c, uint32_t *green)
{
    uint8_t data[3];
    HAL_StatusTypeDef status;

    /* Read one Green FIFO sample */
    status = HAL_I2C_Mem_Read(
        hi2c,
        MAX30102_ADDR,
        MAX30102_REG_FIFO_DATA,
        I2C_MEMADD_SIZE_8BIT,
        data,
        3,
        100
    );

    if (status != HAL_OK)
        return status;

    /* Convert 3 bytes into 18-bit ADC measurement */
    *green = ((uint32_t)data[0] << 16) |
             ((uint32_t)data[1] << 8)  |
              (uint32_t)data[2];

    *green &= 0x3FFFF;

    return HAL_OK;
}

/* Samples Available Function */
uint8_t MAX30102_SamplesAvailable(I2C_HandleTypeDef *hi2c) {

	uint8_t write_ptr;
	uint8_t read_ptr;

	if (MAX30102_ReadReg(hi2c, MAX30102_REG_FIFO_WR_PTR, &write_ptr) != HAL_OK)
		return 0;

	if (MAX30102_ReadReg(hi2c, MAX30102_REG_FIFO_RD_PTR, &read_ptr) != HAL_OK)
		return 0;

	return (write_ptr - read_ptr) & 0x1F;
}

/* Read Sample Function */
HAL_StatusTypeDef MAX30102_ReadSample(I2C_HandleTypeDef *hi2c, uint32_t *red,
		uint32_t *ir) {

	uint8_t data[6];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(hi2c,
	MAX30102_ADDR,
	MAX30102_REG_FIFO_DATA,
	I2C_MEMADD_SIZE_8BIT,
	data,
	6,
	100);

	if (status != HAL_OK)
		return status;

	*red = ((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | data[2];

	*ir = ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 8) | data[5];

	/* MAX30102 ADC data is 18 bits */
	*red &= 0x3FFFF;
	*ir &= 0x3FFFF;

	return HAL_OK;
}

void PPG_StoreSample(uint32_t timestamp_ms,uint32_t red1, uint32_t ir1, uint32_t green2)
{
    ppg_buffer[ppg_write_index].timestamp_ms = timestamp_ms;
    ppg_buffer[ppg_write_index].red1 = red1;
    ppg_buffer[ppg_write_index].ir1 = ir1;
    ppg_buffer[ppg_write_index].green2 = green2;

    ppg_write_index++;

    if (ppg_write_index >= PPG_BUFFER_SIZE) {
        ppg_write_index = 0;

        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    }
}

void PPG_HandleSample(uint8_t sensor_number, uint32_t red, uint32_t ir, uint32_t green)
{
    uint32_t timestamp = HAL_GetTick();

    /* Sensor 1: MAX30102 Finger */
    if (sensor_number == 1) {

        if ((red > EVENT_THRESHOLD) &&
            (ir > EVENT_THRESHOLD)) {

            if (sensor1_warmup_count < WARMUP_SAMPLES) {
                sensor1_warmup_count++;
                sensor1_new_sample = 0;
                return;
            }

            pulse1_present = 1;

            ppg_red1 = red;
            ppg_ir1 = ir;

            sensor1_new_sample = 1;

        } else {
            pulse1_present = 0;
            sensor1_new_sample = 0;
        }
    }

    /* Sensor 2: MAX30101 Wrist */
    else if (sensor_number == 2) {

        if (green > SENSOR2_GREEN_THRESHOLD) {

            if (sensor2_warmup_count < WARMUP_SAMPLES) {
                sensor2_warmup_count++;
                sensor2_new_sample = 0;
                return;
            }

            pulse2_present = 1;

            ppg_green2 = green;

            sensor2_new_sample = 1;

        } else {
            pulse2_present = 0;
            sensor2_new_sample = 0;
        }
    }

    /* Combine and store samples */
    if (sensor1_new_sample &&
        sensor2_new_sample &&
        (ppg_sample_count < MAX_PPG_SAMPLES)) {

        PPG_StoreSample(
            timestamp,
            ppg_red1,
            ppg_ir1,
            ppg_green2
        );

        ppg_sample_count++;
        ppg_last_sample_time = timestamp;

        /* SWV ITM Console output */
        printf("%lu,%lu,%lu\r\n",
            (unsigned long)timestamp,
            (unsigned long)ppg_ir1,
            (unsigned long)ppg_green2
        );

        sensor1_new_sample = 0;
        sensor2_new_sample = 0;
    }
}

/* Interrupt Function */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Sensor 1
	if (GPIO_Pin == DRDY_1_Pin) {
		interrupt_count1++;
		max30102_data_ready1 = 1;

		/* Toggle LED every 50 interrupts */
		if ((interrupt_count1 % 50) == 0) {
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		}

	} else if (GPIO_Pin == DRDY_2_Pin) {
		interrupt_count2++;
		max30102_data_ready2 = 1;
		if ((interrupt_count2 % 50) == 0) {
			HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		}
	}
}

void PPG_HandleInterrupt(
    I2C_HandleTypeDef *hi2c,
    uint8_t sensor_number)
{
    uint8_t interrupt_status;
    uint8_t samples;

    /* Read and clear interrupt status */
    if (MAX30102_ReadReg(
        hi2c,
        MAX30102_REG_INT_STATUS1,
        &interrupt_status) != HAL_OK) {

        if (sensor_number == 1) {
            i2c_error_count1++;
            i2c_last_error1 = HAL_I2C_GetError(hi2c);
            sensor_status1 = MAX30102_ERROR_I2C;
        }
        else if (sensor_number == 2) {
            i2c_error_count2++;
            i2c_last_error2 = HAL_I2C_GetError(hi2c);
            sensor_status2 = MAX30102_ERROR_I2C;
        }

        return;
    }

    /* Successful interrupt-status read */
    if (sensor_number == 1)
        sensor_status1 = MAX30102_OK;
    else if (sensor_number == 2)
        sensor_status2 = MAX30102_OK;

    /* Check PPG Ready interrupt */
    if (interrupt_status & 0x40) {

        samples = MAX30102_SamplesAvailable(hi2c);

        while (samples > 0) {

            uint32_t ppg_red = 0;
            uint32_t ppg_ir = 0;
            uint32_t ppg_green = 0;

            HAL_StatusTypeDef read_status;

            /* Sensor 1: Red + IR */
            if (sensor_number == 1) {

                read_status = MAX30102_ReadSample(
                    hi2c,
                    &ppg_red,
                    &ppg_ir
                );
            }

            /* Sensor 2: Green */
            else if (sensor_number == 2) {

                read_status = MAX30101_ReadGreen(
                    hi2c,
                    &ppg_green
                );
            }
            else {
                return;
            }

            if (read_status == HAL_OK) {

                PPG_HandleSample(
                    sensor_number,
                    ppg_red,
                    ppg_ir,
                    ppg_green
                );

            } else {

                if (sensor_number == 1) {
                    ppg_fifo_error_count1++;
                    i2c_error_count1++;
                    i2c_last_error1 = HAL_I2C_GetError(hi2c);
                    sensor_status1 = MAX30102_ERROR_FIFO;
                }
                else if (sensor_number == 2) {
                    ppg_fifo_error_count2++;
                    i2c_error_count2++;
                    i2c_last_error2 = HAL_I2C_GetError(hi2c);
                    sensor_status2 = MAX30102_ERROR_FIFO;
                }

                break;
            }

            samples--;
        }
    }
}

/* Configure MAX30102 LED Drive Current */
HAL_StatusTypeDef MAX30102_SetLEDCurrent(I2C_HandleTypeDef *hi2c, uint8_t red_current, uint8_t ir_current) {
    HAL_StatusTypeDef status;

    /* Configure RED LED current */
    status = MAX30102_WriteReg(hi2c, MAX30102_REG_LED1_PA, red_current);

    if (status != HAL_OK) {
        return status;
    }

    /* Configure IR LED current */
    status = MAX30102_WriteReg(hi2c, MAX30102_REG_LED2_PA, ir_current);

    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

////
////
////
////

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
