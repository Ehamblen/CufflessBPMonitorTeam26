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
#include "FindHeartbeat.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

typedef enum {

    SENSOR_OK = 0,

    SENSOR_ERROR_I2C,

    SENSOR_ERROR_NOT_FOUND,

    SENSOR_ERROR_BAD_ID,

    SENSOR_ERROR_CONFIG,

    SENSOR_ERROR_FIFO

} Sensor_Status;

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

#define MAX32664_ADDR (0x55 << 1)

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
#define HB_BATCH_RATE_HZ 100U
#define HB_BATCH_SECONDS 10U
#define HB_BATCH_SAMPLES (HB_BATCH_RATE_HZ * HB_BATCH_SECONDS)
#define HB_MAX_BEAT_INDICES 100U

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

/* MAX32664 Debug */

volatile uint8_t hub_command_status = 0xFF;

volatile uint8_t hub_device_mode = 0xFF;

volatile uint8_t hub_fifo_count = 0;

volatile uint8_t hub_status = 0;

volatile uint32_t hub_ir2 = 0;

volatile uint32_t hub_red2 = 0;

volatile uint32_t hub_sample_count = 0;

volatile uint8_t hub_init_step = 0;

volatile HAL_StatusTypeDef hub_last_hal_status = HAL_OK;

volatile uint8_t hub_green_step = 0;

volatile uint8_t hub_mode_reg = 0xFF;

volatile uint8_t hub_slots1_reg = 0xFF;

volatile uint8_t hub_slots2_reg = 0xFF;

//volatile uint32_t hub_led3_field = 0;

//volatile uint32_t hub_led4_field = 0;

/* Error Case Store */

Sensor_Status sensor_status1 = SENSOR_OK;

Sensor_Status sensor_status2 = SENSOR_OK;

volatile uint32_t i2c_error_count1 = 0;

volatile uint32_t i2c_last_error1 = 0;

volatile uint32_t i2c_error_count2 = 0;

volatile uint32_t i2c_last_error2 = 0;

/* Debug Variable for MAX30102 Connection */

HAL_StatusTypeDef max30102_connection_status = HAL_ERROR;

HAL_StatusTypeDef max32664_connection_status = HAL_ERROR;

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

/* Static RAM storage for a 10-second finger-only IR batch. */
static double hb_samples[HB_BATCH_SAMPLES];
static int hb_beat_indices[HB_MAX_BEAT_INDICES];
static uint16_t hb_count = 0;
static uint8_t hb_ready = 0;
volatile uint32_t hb_batches_processed = 0;
volatile uint32_t hb_fifo_overflows = 0;
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

void PPG_HandleSample(uint8_t sensor_number, uint32_t red, uint32_t ir,

		uint32_t green); // General Function for Sample Handling

void PPG_StoreSample(uint32_t timestamp_ms, uint32_t red1, uint32_t ir1,

		uint32_t green2); // Store Sample

/* Interrupt */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); // Interrupt Callback

void PPG_HandleInterrupt(I2C_HandleTypeDef *hi2c, uint8_t sensor_number);

/* Varing Current */

HAL_StatusTypeDef MAX30102_SetLEDCurrent(I2C_HandleTypeDef *hi2c,

		uint8_t red_current, uint8_t ir_current);

HAL_StatusTypeDef HUB_Boot(void);

HAL_StatusTypeDef HUB_Command(const uint8_t *cmd, uint16_t cmd_len,

		uint8_t *data, uint16_t data_len, uint32_t delay_ms);

HAL_StatusTypeDef HUB_InitRaw(void);

void HUB_ReadRawSamples(void);

HAL_StatusTypeDef HUB_EnableGreenTest(void);

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

uint8_t id1 = 0;

	uint8_t dummy_status = 0;

//	const uint8_t mode_cmd[] = { 0x02, 0x00 };

//	uint8_t mode = 0xFF;

max30102_connection_status = HAL_I2C_IsDeviceReady(

	    &hi2c1,

	    MAX30102_ADDR,

	    3,

	    100

	);

if (max30102_connection_status != HAL_OK) {

	    sensor_status1 = SENSOR_ERROR_NOT_FOUND;

	    Error_Handler();

	}

if (MAX30102_ReadReg(&hi2c1,

	MAX30102_REG_PART_ID, &id1) != HAL_OK) {

sensor_status1 = SENSOR_ERROR_I2C;

		Error_Handler();

	}

if (id1 != MAX30102_EXPECTED_ID) {

		sensor_status1 = SENSOR_ERROR_BAD_ID;

		Error_Handler();

	}

/* Finger-only milestone: bypass the MAX32664 wrist hub. */
    sensor_status2 = SENSOR_OK;

    if (MAX30102_Init(&hi2c1) != HAL_OK) {

		sensor_status1 = SENSOR_ERROR_CONFIG;

		Error_Handler();

	}

sensor_status1 = SENSOR_OK;

if (MAX30102_ReadReg(&hi2c1,

	MAX30102_REG_INT_STATUS1, &dummy_status) != HAL_OK) {

sensor_status1 = SENSOR_ERROR_I2C;

		Error_Handler();

	}

if (MAX30102_ReadReg(&hi2c1,

	MAX30102_REG_INT_STATUS2, &dummy_status) != HAL_OK) {

sensor_status1 = SENSOR_ERROR_I2C;

		Error_Handler();

	}

/* Clear pending external interrupt */

	__HAL_GPIO_EXTI_CLEAR_IT(DRDY_1_Pin);

printf("HAM finger-only batch heartbeat test\r\n");

/* Initialization complete */

/* USER CODE END 2 */

/* Infinite loop */

	/* USER CODE BEGIN WHILE */

uint32_t last_finger_poll = HAL_GetTick();
    printf("Collecting 1000 finger IR samples...\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();
        if (!hb_ready && (max30102_data_ready1 ||
            (uint32_t)(now - last_finger_poll) >= 10U)) {
            max30102_data_ready1 = 0;
            last_finger_poll = now;
            PPG_HandleInterrupt(&hi2c1, 1);
        }

        if (hb_ready) {
            int beats = FindHeartbeats(hb_samples, (int)HB_BATCH_SAMPLES,
                                       (double)HB_BATCH_RATE_HZ, hb_beat_indices);
            printf("BATCH,%lu,samples=%u,beats=%d\r\n",
                   (unsigned long)(hb_batches_processed + 1U),
                   (unsigned)HB_BATCH_SAMPLES, beats);
            if (beats >= 2) {
                int span = hb_beat_indices[beats - 1] - hb_beat_indices[0];
                if (span > 0) {
                    double bpm = 60.0 * (double)(beats - 1) *
                                 (double)HB_BATCH_RATE_HZ / (double)span;
                    printf("BPM,%.2f\r\n", bpm);
                }
            } else {
                printf("BPM,unavailable\r\n");
            }
            hb_batches_processed++;

            /* Discard all FIFO data collected during processing. */
            if (MAX30102_WriteReg(&hi2c1, MAX30102_REG_FIFO_WR_PTR, 0) != HAL_OK ||
                MAX30102_WriteReg(&hi2c1, MAX30102_REG_OVF_COUNTER, 0) != HAL_OK ||
                MAX30102_WriteReg(&hi2c1, MAX30102_REG_FIFO_RD_PTR, 0) != HAL_OK) {
                sensor_status1 = SENSOR_ERROR_I2C;
                Error_Handler();
            }
            hb_count = 0;
            hb_ready = 0;
            sensor1_warmup_count = 0;
            max30102_data_ready1 = 0;
            last_finger_poll = HAL_GetTick();
            printf("Collecting next batch...\r\n");
        }
    }

    /* USER CODE END WHILE */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

}

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

/*Configure GPIO pin Output Level */

	HAL_GPIO_WritePin(HUB_RST_GPIO_Port, HUB_RST_Pin, GPIO_PIN_SET);

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

/*Configure GPIO pin : HUB_RST_Pin */

	GPIO_InitStruct.Pin = HUB_RST_Pin;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(HUB_RST_GPIO_Port, &GPIO_InitStruct);

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

HAL_StatusTypeDef MAX30102_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg,

		uint8_t value) {

	return HAL_I2C_Mem_Write(hi2c,

	MAX30102_ADDR, reg,

	I2C_MEMADD_SIZE_8BIT, &value, 1, 100);

}

HAL_StatusTypeDef MAX30102_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg,

		uint8_t *value) {

	return HAL_I2C_Mem_Read(hi2c,

	MAX30102_ADDR, reg,

	I2C_MEMADD_SIZE_8BIT, value, 1, 100);

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

/* MAX32664 */

HAL_StatusTypeDef HUB_Command(const uint8_t *cmd, uint16_t cmd_len,

		uint8_t *data, uint16_t data_len, uint32_t delay_ms) {

	uint8_t response[13] = { 0 };

if ((cmd == NULL) || (cmd_len < 2) || (data_len > 12)

			|| (data_len > 0 && data == NULL)) {

		return HAL_ERROR;

	}

HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2,

	MAX32664_ADDR, (uint8_t*) cmd, cmd_len, 100);

hub_last_hal_status = status;

if (status != HAL_OK) {

		return status;

	}

HAL_Delay(delay_ms);

status = HAL_I2C_Master_Receive(&hi2c2,

	MAX32664_ADDR, response, data_len + 1, 100);

hub_last_hal_status = status;

if (status != HAL_OK) {

		return status;

	}

/* First response byte is command status */

	hub_command_status = response[0];

if (hub_command_status != 0x00) {

		return HAL_ERROR;

	}

/* Remaining bytes contain command data */

	for (uint16_t i = 0; i < data_len; i++) {

		data[i] = response[i + 1];

	}

return HAL_OK;

}

HAL_StatusTypeDef HUB_Boot(void)

{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

/* Disable Sensor 2 EXTI */

    HAL_NVIC_DisableIRQ(EXTI2_TSC_IRQn);

/* Assert MAX32664 reset */

    HAL_GPIO_WritePin(

        HUB_RST_GPIO_Port,

        HUB_RST_Pin,

        GPIO_PIN_RESET

    );

/* Prepare MFIO HIGH for application mode */

    HAL_GPIO_WritePin(

        DRDY_2_GPIO_Port,

        DRDY_2_Pin,

        GPIO_PIN_SET

    );

GPIO_InitStruct.Pin = DRDY_2_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

HAL_GPIO_Init(DRDY_2_GPIO_Port, &GPIO_InitStruct);

HAL_Delay(10);

/* Release MAX32664 reset */

    HAL_GPIO_WritePin(

        HUB_RST_GPIO_Port,

        HUB_RST_Pin,

        GPIO_PIN_SET

    );

/* Allow the hub to enter application mode */

    HAL_Delay(50);

/* Release MFIO; Sensor 2 uses polling */

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

HAL_GPIO_Init(DRDY_2_GPIO_Port, &GPIO_InitStruct);

/* Wait for application initialization */

    HAL_Delay(950);

/* Verify I2C communication */

    return HAL_I2C_IsDeviceReady(

        &hi2c2,

        MAX32664_ADDR,

        3,

        100

    );

}

HAL_StatusTypeDef HUB_InitRaw(void) {

	/* 1. Sensor-only output */

	const uint8_t cmd1[] = { 0x10, 0x00, 0x01 };

/* 2. FIFO threshold: 1 sample */

	const uint8_t cmd2[] = { 0x10, 0x01, 0x01 };

/* 3. Enable MAX30101 */

	const uint8_t cmd3[] = { 0x44, 0x03, 0x01 };

/* 4. Enable algorithm mode 1 */

	const uint8_t cmd4[] = { 0x52, 0x02, 0x01 };

/* 5. Disable automatic gain control */

	const uint8_t cmd5[] = { 0x52, 0x00, 0x00 };

/* 6. Configure Red LED current */

	const uint8_t cmd6[] = { 0x40, 0x03, 0x0C, 0x1F };

/* 7. Configure IR LED current */

	const uint8_t cmd7[] = { 0x40, 0x03, 0x0D, 0x1F };

hub_init_step = 0;

if (HUB_Command(cmd1, sizeof(cmd1),

	NULL, 0, 5) != HAL_OK)

		return HAL_ERROR;

	hub_init_step = 1;

if (HUB_Command(cmd2, sizeof(cmd2),

	NULL, 0, 5) != HAL_OK)

		return HAL_ERROR;

	hub_init_step = 2;

/* Allow previous configuration to settle */

	HAL_Delay(50);

/* Enable MAX30101 optical sensor */

	if (HUB_Command(cmd3, sizeof(cmd3),

	                NULL, 0, 100) != HAL_OK)

	    return HAL_ERROR;

hub_init_step = 3;

if (HUB_Command(cmd4, sizeof(cmd4),

	NULL, 0, 100) != HAL_OK)

		return HAL_ERROR;

	hub_init_step = 4;

/* Give the newly enabled algorithm time to settle */

	HAL_Delay(100);

if (HUB_Command(

	        cmd5,

	        sizeof(cmd5),

	        NULL,

	        0,

	        100) != HAL_OK) {

return HAL_ERROR;

	}

hub_init_step = 5;

/* Wait before changing MAX30101 registers */

	HAL_Delay(100);

if (HUB_Command(cmd6, sizeof(cmd6),

	NULL, 0, 5) != HAL_OK)

		return HAL_ERROR;

	hub_init_step = 6;

if (HUB_Command(cmd7, sizeof(cmd7),

	NULL, 0, 5) != HAL_OK)

		return HAL_ERROR;

	hub_init_step = 7;

return HAL_OK;

}

void HUB_ReadRawSamples(void)

{

    const uint8_t count_cmd[] = {0x12, 0x00};

    const uint8_t read_cmd[]  = {0x12, 0x01};

uint8_t count = 0;

    uint8_t raw[6] = {0};

/* Determine how many FIFO records are available */

    if (HUB_Command(count_cmd, sizeof(count_cmd),

                    &count, 1, 2) != HAL_OK)

    {

        i2c_error_count2++;

        sensor_status2 = SENSOR_ERROR_FIFO;

        return;

    }

hub_fifo_count = count;

/* Drain available FIFO records */

    while (count > 0)

    {

        if (HUB_Command(read_cmd, sizeof(read_cmd),

                        raw, sizeof(raw), 2) != HAL_OK)

        {

            i2c_error_count2++;

            sensor_status2 = SENSOR_ERROR_FIFO;

            return;

        }

/*

         * MAX32664 sensor-only output:

         *

         * Bytes 0-2: First reported optical channel

         * Bytes 3-5: Second reported optical channel

         *

         * Nominally IR and Red in the standard

         * SparkFun configuration.

         *

         * With custom LED slots, channel identity

         * must be verified experimentally.

         */

uint32_t channel1 =

            (((uint32_t)raw[0] << 16) |

             ((uint32_t)raw[1] << 8)  |

              (uint32_t)raw[2]) & 0x3FFFF;

uint32_t channel2 =

            (((uint32_t)raw[3] << 16) |

             ((uint32_t)raw[4] << 8)  |

              (uint32_t)raw[5]) & 0x3FFFF;

/* Preserve existing debug variables */

        hub_ir2 = channel1;

        hub_red2 = channel2;

hub_sample_count++;

        sensor_status2 = SENSOR_OK;

/*

         * Print every fifth sample.

         * Keep both channels until we establish

         * which one represents green.

         */

        if ((hub_sample_count % 5) == 0)

        {

            printf("W,%lu,%lu,%lu\r\n",

                (unsigned long)HAL_GetTick(),

                (unsigned long)channel1,

                (unsigned long)channel2

            );

        }

count--;

    }

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

	I2C_MEMADD_SIZE_8BIT, data, 6, 100);

if (status != HAL_OK)

		return status;

*red = ((uint32_t) data[0] << 16) | ((uint32_t) data[1] << 8) | data[2];

*ir = ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 8) | data[5];

/* MAX30102 ADC data is 18 bits */

	*red &= 0x3FFFF;

	*ir &= 0x3FFFF;

return HAL_OK;

}

void PPG_StoreSample(uint32_t timestamp_ms, uint32_t red1, uint32_t ir1,

		uint32_t green2) {

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

void PPG_HandleSample(uint8_t sensor_number, uint32_t red, uint32_t ir,
                      uint32_t green) {
    (void)green;
    if (sensor_number != 1 || hb_ready) return;
    ppg_red1 = red;
    ppg_ir1 = ir;
    sensor1_new_sample = 1;

    /* Previous EVENT_THRESHOLD=10000 rejected your ~600-count finger data. */
    if (sensor1_warmup_count < WARMUP_SAMPLES) {
        sensor1_warmup_count++;
        return;
    }
    if (hb_count < HB_BATCH_SAMPLES) {
        hb_samples[hb_count++] = (double)ir;
    }
    if (hb_count >= HB_BATCH_SAMPLES) hb_ready = 1;
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

void PPG_HandleInterrupt(I2C_HandleTypeDef *hi2c, uint8_t sensor_number) {

	uint8_t interrupt_status;

	uint8_t samples;

/* Read and clear interrupt status */

	if (MAX30102_ReadReg(hi2c,

	MAX30102_REG_INT_STATUS1, &interrupt_status) != HAL_OK) {

if (sensor_number == 1) {

			i2c_error_count1++;

			i2c_last_error1 = HAL_I2C_GetError(hi2c);

			sensor_status1 = SENSOR_ERROR_I2C;

		} else if (sensor_number == 2) {

			i2c_error_count2++;

			i2c_last_error2 = HAL_I2C_GetError(hi2c);

			sensor_status2 = SENSOR_ERROR_I2C;

		}

return;

	}

/* Successful interrupt-status read */

	if (sensor_number == 1)

		sensor_status1 = SENSOR_OK;

	else if (sensor_number == 2)

		sensor_status2 = SENSOR_OK;

/* Check PPG Ready interrupt */

	if ((interrupt_status & 0x40) || sensor_number == 1) {

uint8_t overflow = 0;
        if (MAX30102_ReadReg(hi2c, MAX30102_REG_OVF_COUNTER, &overflow) == HAL_OK && overflow != 0) {
            hb_fifo_overflows++;
            hb_count = 0;
            sensor1_warmup_count = 0;
            MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_WR_PTR, 0);
            MAX30102_WriteReg(hi2c, MAX30102_REG_OVF_COUNTER, 0);
            MAX30102_WriteReg(hi2c, MAX30102_REG_FIFO_RD_PTR, 0);
            return;
        }
        samples = MAX30102_SamplesAvailable(hi2c);

while (samples > 0) {

uint32_t ppg_red = 0;

			uint32_t ppg_ir = 0;

			uint32_t ppg_green = 0;

HAL_StatusTypeDef read_status;

/* Sensor 1: Red + IR */

			if (sensor_number == 1) {

read_status = MAX30102_ReadSample(hi2c, &ppg_red, &ppg_ir);

			}

else {

				return;

			}

if (read_status == HAL_OK) {

PPG_HandleSample(sensor_number, ppg_red, ppg_ir, ppg_green);

} else {

if (sensor_number == 1) {

					ppg_fifo_error_count1++;

					i2c_error_count1++;

					i2c_last_error1 = HAL_I2C_GetError(hi2c);

					sensor_status1 = SENSOR_ERROR_FIFO;

				} else if (sensor_number == 2) {

					ppg_fifo_error_count2++;

					i2c_error_count2++;

					i2c_last_error2 = HAL_I2C_GetError(hi2c);

					sensor_status2 = SENSOR_ERROR_FIFO;

				}

break;

			}

samples--;
        if (hb_ready) break;

		}

	}

}

/* Configure MAX30102 LED Drive Current */

HAL_StatusTypeDef MAX30102_SetLEDCurrent(I2C_HandleTypeDef *hi2c,

		uint8_t red_current, uint8_t ir_current) {

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

HAL_StatusTypeDef HUB_EnableGreenTest(void)

{

    const uint8_t green3_current[] =

        {0x40, 0x03, 0x0E, 0x1F};

const uint8_t green4_current[] =

        {0x40, 0x03, 0x0F, 0x1F};

/* Slot 1 = Green, Slot 2 = IR */

    const uint8_t slots12[] =

        {0x40, 0x03, 0x11, 0x23};

/* Disable Slots 3 and 4 */

    const uint8_t slots34[] =

        {0x40, 0x03, 0x12, 0x00};

/* Enable Multi-LED mode */

    const uint8_t multi_led_mode[] =

        {0x40, 0x03, 0x09, 0x07};

/* Register readback commands */

    const uint8_t read_mode[] =

        {0x41, 0x03, 0x09};

const uint8_t read_slots12[] =

        {0x41, 0x03, 0x11};

const uint8_t read_slots34[] =

        {0x41, 0x03, 0x12};

hub_green_step = 0;

/* Let hub initialization settle */

    HAL_Delay(100);

if (HUB_Command(green3_current, 4,

                    NULL, 0, 50) != HAL_OK)

        return HAL_ERROR;

    hub_green_step = 1;

if (HUB_Command(green4_current, 4,

                    NULL, 0, 50) != HAL_OK)

        return HAL_ERROR;

    hub_green_step = 2;

if (HUB_Command(slots12, 4,

                    NULL, 0, 50) != HAL_OK)

        return HAL_ERROR;

    hub_green_step = 3;

if (HUB_Command(slots34, 4,

                    NULL, 0, 50) != HAL_OK)

        return HAL_ERROR;

    hub_green_step = 4;

if (HUB_Command(multi_led_mode, 4,

                    NULL, 0, 50) != HAL_OK)

        return HAL_ERROR;

    hub_green_step = 5;

/* Verify that settings were accepted */

    if (HUB_Command(read_mode, 3,

                    (uint8_t *)&hub_mode_reg,

                    1, 20) != HAL_OK)

        return HAL_ERROR;

if (HUB_Command(read_slots12, 3,

                    (uint8_t *)&hub_slots1_reg,

                    1, 20) != HAL_OK)

        return HAL_ERROR;

if (HUB_Command(read_slots34, 3,

                    (uint8_t *)&hub_slots2_reg,

                    1, 20) != HAL_OK)

        return HAL_ERROR;

if ((hub_mode_reg & 0x07) != 0x07 ||

        hub_slots1_reg != 0x23 ||

        hub_slots2_reg != 0x00)

    {

        return HAL_ERROR;

    }

hub_green_step = 6;

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
