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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

typedef enum
{
    MAX30102_OK = 0,
    MAX30102_ERROR_I2C,
    MAX30102_ERROR_NOT_FOUND,
    MAX30102_ERROR_BAD_ID,
    MAX30102_ERROR_RESET,
    MAX30102_ERROR_CONFIG,
    MAX30102_ERROR_FIFO
} MAX30102_Status;

typedef struct
{
    /* Simulated or measured Red-light value */
    uint32_t red;
    /* Simulated or measured infrared-light value */
    uint32_t ir;
} PPG_Sample;

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t red;
    uint32_t ir;
} PPG_DataPoint;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX30102_ADDR (0x57 << 1) // 7-bit I2C address, with required left shift

/* BELOW: REGISTER MAP FOR MAX30102 */

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

/* SIMULATED PPG DATA */
#define PPG_SIMULATION 0 // 1 = simulation; 0 = real
#define PPG_BUFFER_SIZE 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* Error Case Store */
MAX30102_Status sensor_status = MAX30102_OK;

uint8_t max30102_id = 0;
HAL_StatusTypeDef max30102_status;
HAL_StatusTypeDef max30102_read_status;

uint32_t ppg_red = 0;
uint32_t ppg_ir = 0;

volatile uint32_t ppg_sample_count = 0;
volatile uint32_t ppg_i2c_error_count = 0;
volatile uint32_t ppg_last_sample_time = 0;

/* FIFO Variables */
volatile uint8_t ppg_last_fifo_count = 0;
volatile uint32_t ppg_fifo_error_count = 0;

/* Simulation Variables + Array */

static const PPG_Sample test_ppg[] =
{
    {50000, 70000},
    {50100, 70120},
    {50300, 70350},
    {50800, 70900},
    {51600, 71800},
    {52500, 72800},
    {53100, 73500},
    {52500, 72900},
    {51600, 71900},
    {50700, 70900},
    {50200, 70300},
    {50000, 70000}
};

#define TEST_PPG_LENGTH \
    (sizeof(test_ppg) / sizeof(test_ppg[0]))

/* Storing PPG data */
PPG_DataPoint ppg_buffer[PPG_BUFFER_SIZE];
uint16_t ppg_write_index = 0;

/* Simulation Display */
char ppg_output_line[64];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef MAX30102_ReadReg(uint8_t reg, uint8_t *value);

HAL_StatusTypeDef MAX30102_Reset(void);
HAL_StatusTypeDef MAX30102_Init(void);

uint8_t MAX30102_SamplesAvailable(void);
HAL_StatusTypeDef MAX30102_ReadSample(uint32_t *red, uint32_t *ir);

void PPG_RunSimulation(void); // Simulate PPG Sample
void PPG_HandleSample(uint32_t red, uint32_t ir); // General Function for Sample Handling
void PPG_ProcessSample(uint32_t red, uint32_t ir); // Simulate Red + IR Sample
void PPG_StoreSample(uint32_t timestamp_ms, uint32_t red, uint32_t ir); // Store Sample
void PPG_SendSample(uint32_t timestamp_ms, uint32_t red, uint32_t ir); // Sending Sample for Display


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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

  /* USER CODE BEGIN 2 */

  /* REAL Communication with MAX30102 */
  if (PPG_SIMULATION == 0)
  {
	  uint8_t id;

	  max30102_status = HAL_I2C_IsDeviceReady(
		  &hi2c1,
		  MAX30102_ADDR,
		  3,
		  100
	  );

	  if (max30102_status != HAL_OK)
	  {
		  sensor_status = MAX30102_ERROR_NOT_FOUND;
		  Error_Handler();
	  }

	  if (MAX30102_ReadReg(MAX30102_REG_PART_ID, &id) != HAL_OK)
	  {
		  sensor_status = MAX30102_ERROR_I2C;
		  Error_Handler();
	  }

	  max30102_id = id;

	  if (max30102_id != MAX30102_EXPECTED_ID)
	  {
		  sensor_status = MAX30102_ERROR_BAD_ID;
		  Error_Handler();
	  }

	  if (MAX30102_Init() != HAL_OK)
	  {
		  sensor_status = MAX30102_ERROR_CONFIG;
		  Error_Handler();
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* SIMULATION */
	  if (PPG_SIMULATION == 1)
	  {
		  PPG_RunSimulation(); // Create 1 Red + IR Sample
		  HAL_Delay(10); // Delay of 10ms
	  }
	  else
	  {
	      ppg_last_fifo_count = MAX30102_SamplesAvailable();

	      while (ppg_last_fifo_count > 0)
	      {
	          if (MAX30102_ReadSample(&ppg_red, &ppg_ir) == HAL_OK)
	          {
	              PPG_HandleSample(ppg_red, ppg_ir);
	          }
	          else
	          {
	              ppg_fifo_error_count++;
	              sensor_status = MAX30102_ERROR_FIFO;
	              break;
	          }

	          ppg_last_fifo_count = MAX30102_SamplesAvailable();
	      }
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_USB_PCD_Init(void)
{

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
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* WRITE + READ FUNCTIONS */

HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        MAX30102_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}


HAL_StatusTypeDef MAX30102_ReadReg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        MAX30102_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        100
    );
}

/* RESET FUNCTION */

HAL_StatusTypeDef MAX30102_Reset(void)
{
    HAL_StatusTypeDef status;
    uint8_t mode;
    uint32_t timeout;

    /* Reset Bit */
    status = MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x40);

    if (status != HAL_OK)
        return status;

    timeout = HAL_GetTick();

    do
    {
        status = MAX30102_ReadReg(MAX30102_REG_MODE_CONFIG, &mode);

        if (status != HAL_OK)
            return status;

        if ((mode & 0x40) == 0)
            return HAL_OK;

    } while ((HAL_GetTick() - timeout) < 100);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef MAX30102_Init(void)
{
    HAL_StatusTypeDef status;

    status = MAX30102_Reset();

    if (status != HAL_OK)
    {
        return status;
    }

    /* Clear FIFO write pointer */
    status = MAX30102_WriteReg(
        MAX30102_REG_FIFO_WR_PTR,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Clear FIFO overflow counter */
    status = MAX30102_WriteReg(
        MAX30102_REG_OVF_COUNTER,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Clear FIFO read pointer */
    status = MAX30102_WriteReg(
        MAX30102_REG_FIFO_RD_PTR,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Configure FIFO */
    status = MAX30102_WriteReg(
        MAX30102_REG_FIFO_CONFIG,
        0x10
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Configure SpO2 ADC:
     *
     * ADC range   = 4096 nA
     * Sample rate = 100 samples/sec
     * Resolution  = 18 bits
     */
    status = MAX30102_WriteReg(
        MAX30102_REG_SPO2_CONFIG,
        0x27
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Set Red LED current */
    status = MAX30102_WriteReg(
        MAX30102_REG_LED1_PA,
        0x1F
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Set IR LED current */
    status = MAX30102_WriteReg(
        MAX30102_REG_LED2_PA,
        0x1F
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /* Start SpO2 mode: Red + IR */
    status = MAX30102_WriteReg(
        MAX30102_REG_MODE_CONFIG,
        0x03
    );

    return status;
}

uint8_t MAX30102_SamplesAvailable(void)
{
    uint8_t write_ptr;
    uint8_t read_ptr;

    if (MAX30102_ReadReg(MAX30102_REG_FIFO_WR_PTR, &write_ptr) != HAL_OK)
        return 0;

    if (MAX30102_ReadReg(MAX30102_REG_FIFO_RD_PTR, &read_ptr) != HAL_OK)
        return 0;

    return (write_ptr - read_ptr) & 0x1F;
}

HAL_StatusTypeDef MAX30102_ReadSample(uint32_t *red, uint32_t *ir)
{
    uint8_t data[6];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        MAX30102_ADDR,
        MAX30102_REG_FIFO_DATA,
        I2C_MEMADD_SIZE_8BIT,
        data,
        6,
        100
    );

    if (status != HAL_OK)
        return status;

    *red = ((uint32_t)data[0] << 16) |
           ((uint32_t)data[1] << 8) |
           data[2];

    *ir = ((uint32_t)data[3] << 16) |
          ((uint32_t)data[4] << 8) |
          data[5];

    /* MAX30102 ADC data is 18 bits */
    *red &= 0x3FFFF;
    *ir  &= 0x3FFFF;

    return HAL_OK;
}

/*
 * Function:
 * PPG_ProcessSample
 *
 * Purpose:
 * Take one Red + IR PPG measurement and perform
 * signal processing on it.
 *
 * For now, this function does nothing.
 * Later, this is where you can add:
 *
 * - DC removal
 * - filtering
 * - peak detection
 * - heart-rate calculation
 * - PPG feature extraction
 * - blood-pressure estimation
 */
void PPG_ProcessSample(uint32_t red, uint32_t ir)
{
    /*
     * These lines prevent compiler warnings while
     * we are not yet using red and ir.
     */
    (void)red;
    (void)ir;
}

/* Simulation Function Calls */
void PPG_RunSimulation(void)
{
    static uint32_t index = 0;

    ppg_red = test_ppg[index].red;
    ppg_ir  = test_ppg[index].ir;

    PPG_HandleSample(ppg_red, ppg_ir);

    index++;

    if (index >= TEST_PPG_LENGTH)
    {
        index = 0;
    }
}

/*
 * Function:
 * PPG_StoreSample
 *
 * Purpose:
 * Store one Red + IR PPG measurement
 * in the ppg_buffer array.
 */

void PPG_StoreSample(uint32_t timestamp_ms, uint32_t red, uint32_t ir)
{
    ppg_buffer[ppg_write_index].timestamp_ms = timestamp_ms;
    ppg_buffer[ppg_write_index].red = red;
    ppg_buffer[ppg_write_index].ir = ir;

    ppg_write_index++; // Iterate Through Array

    if (ppg_write_index >= PPG_BUFFER_SIZE)
    {
        ppg_write_index = 0;
    }
}

/*
 * Function:
 * PPG_SendSample
 *
 * Purpose:
 * Eventually send one PPG measurement to the computer.
 *
 * Desired output format:
 *
 * timestamp_ms,red,ir
 *
 * Example:
 *
 * 1020,52341,71320
 */

void PPG_SendSample(uint32_t timestamp_ms, uint32_t red, uint32_t ir)
{
    (void)timestamp_ms;
    (void)red;
    (void)ir;

    /* Print Statement */
    snprintf(
        ppg_output_line,              // Where the text will be stored
        sizeof(ppg_output_line),      // Max memory available
        "%lu,%lu,%lu\r\n",            // CSV formatting
        (unsigned long)timestamp_ms,  // Time
        (unsigned long)red,           // Red PPG value
        (unsigned long)ir             // IR PPG value
    );
}

void PPG_HandleSample(uint32_t red, uint32_t ir)
{
    uint32_t timestamp = HAL_GetTick();

    PPG_StoreSample(timestamp, red, ir);

    PPG_ProcessSample(red, ir);

    PPG_SendSample(timestamp, red, ir);

    ppg_sample_count++;
    ppg_last_sample_time = timestamp;
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
