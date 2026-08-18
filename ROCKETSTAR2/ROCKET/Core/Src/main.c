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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

#include "MPU9250.h"
#include "BMP280.h"
#include "sdcard.h"
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
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
MPU9250_t MPU9250;
static BMP280_t bmp280;

/* Buffer de test pour la carte SD (bloc de 512 octets) */
uint8_t sd_buffer[512];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USB_Print(const char* str)
{
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

void USB_Printf(const char* format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  MPU9250.settings.gFullScaleRange = GFSR_500DPS;
  MPU9250.settings.aFullScaleRange = AFSR_4G;
  MPU9250.settings.CS_PIN = GPIO_PIN_12;
  MPU9250.settings.CS_PORT = GPIOB;
  MPU9250.attitude.tau = 0.98;
  MPU9250.attitude.dt = 0.004;
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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  /* Désactiver le CS de la Carte SD avant d'initialiser d'autres cartes SPI */
  SDCARD_Unselect();

  /* 1. Initialisation MPU9250 */
  if (MPU_begin(&hspi2, &MPU9250) != 1)
  {
      USB_Printf("MPU9250 init failed!\r\n");
  } else {
      USB_Printf("MPU9250 ready\r\n");
  }

  /* 2. Initialisation BMP280 */
  BMP280_Attach(&bmp280, &hspi2, GPIOC, GPIO_PIN_15);
  BMP280_Status_t st = BMP280_Init(&bmp280,
                                    BMP280_OSRS_X2,          /* temp oversampling */
                                    BMP280_OSRS_X16,         /* pressure oversampling */
                                    BMP280_MODE_NORMAL,      /* free-running mode */
                                    BMP280_STANDBY_62_5MS,   /* standby duration */
                                    BMP280_FILTER_4);        /* IIR filter */

  if (st != BMP280_OK) {
      USB_Printf("BMP280 init failed, err=%d\r\n", (int)st);
  } else {
      USB_Printf("BMP280 ready\r\n");
  }

  MPU_calibrateGyro(&hspi2, &MPU9250, 1500);

  /* 3. Initialisation Carte SD */
  int sd_res = SDCARD_Init();
  if (sd_res == 0) {
      uint32_t blocks = 0;
      if (SDCARD_GetBlocksNumber(&blocks) == 0) {
          USB_Printf("SD Card Init OK! Total blocks: %lu (Size: %lu MB)\r\n", blocks, (blocks / 2048));
      } else {
          USB_Printf("SD Card Init OK, but failed to read block count.\r\n");
      }
  } else {
      USB_Printf("SD Card Init Failed, err=%d\r\n", sd_res);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

      /* ---- Lecture MPU9250 ---- */
      MPU_calcAttitude(&hspi2, &MPU9250);
      int16_t roll  = roundf(10 * MPU9250.attitude.r);
      int16_t pitch = roundf(10 * MPU9250.attitude.p);
      int16_t yaw   = roundf(10 * MPU9250.attitude.y);

      USB_Printf("Attitude: %d.%d, %d.%d, %d.%d\r\n",
                 roll / 10, abs(roll % 10),
                 pitch / 10, abs(pitch % 10),
                 yaw / 10, abs(yaw % 10));

      /* ---- Lecture BMP280 (Température, Pression et Altitude) ---- */
      float temp_c = 0.0f;
      float press_hpa = 0.0f;
      float alt_m = 0.0f;

      if (BMP280_ReadAltitude(&bmp280, SEALEVEL_PRESSURE_HPA, &temp_c, &press_hpa, &alt_m) == BMP280_OK) {
          int16_t temp_int  = (int16_t)(temp_c * 100.0f);
          int32_t press_int = (int32_t)(press_hpa * 100.0f);
          int16_t alt_int   = (int16_t)(alt_m * 10.0f);

          USB_Printf("T = %d.%02d C   P = %ld.%02ld hPa   Alt = %d.%d m\r\n",
                     temp_int / 100, abs(temp_int % 100),
                     press_int / 100, abs((int)(press_int % 100)),
                     alt_int / 10, abs(alt_int % 10));
      } else {
          USB_Printf("BMP280 read error\r\n");
      }

      HAL_Delay(1000);
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Niveaux de sortie par défaut */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); /* BMP280 CS */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); /* SD Card CS */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); /* MPU9250 CS */

  /* Configuration PC13 (LED), PC14 (SD CS), PC15 (BMP280 CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configuration PB12 (MPU9250 CS) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
