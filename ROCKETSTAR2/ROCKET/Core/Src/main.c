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

#include "MPU9250.h"
#include "BMP280.h"
#include "sdcard.h"
#include "qmc5883p.h"
#include "LoRa.h"
#include "gps.h"

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

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

MPU9250_t MPU9250;
static BMP280_t bmp280;

QMC5883P_HandleTypeDef qmc;
volatile bool qmc_dataReady = false;

LoRa myLora;

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

  if (MPU_begin(&hspi2, &MPU9250) != 1)
  {
      USB_Printf("ERROR\n\r");
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  BMP280_Attach(&bmp280, &hspi2, GPIOA, GPIO_PIN_3);
  BMP280_Status_t st = BMP280_Init(&bmp280,
                                    BMP280_OSRS_X2,          /* temp oversampling */
                                    BMP280_OSRS_X16,         /* pressure oversampling */
                                    BMP280_MODE_NORMAL,      /* free-running */
                                    BMP280_STANDBY_62_5MS,   /* time between samples */
                                    BMP280_FILTER_4);        /* IIR filter */

  if (st != BMP280_OK) {
      USB_Printf("BMP280 init failed, err=%d\r\n", (int)st);
  }

  USB_Printf("BMP280 ready\r\n");

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  MPU_calibrateGyro(&hspi2, &MPU9250, 1500);

  /* --- Début Code SD Card --- */

  // IMPORTANT : mettre CS à l'état haut AVANT toute autre init SPI
  SDCARD_Unselect();

  int ret = SDCARD_Init();
  if (ret != 0) {
      USB_Printf("SDCARD_Init failed: %d\r\n", ret);
      Error_Handler();
  }
  USB_Printf("SDCARD_Init OK\r\n");

  // Lire le nombre de blocs de la carte
  uint32_t numBlocks = 0;
  if (SDCARD_GetBlocksNumber(&numBlocks) == 0) {
      USB_Printf("Nombre de blocs : %lu (~%lu Mo)\r\n",
                 (unsigned long)numBlocks,
                 (unsigned long)(numBlocks / 2 / 1024));
  } else {
      USB_Printf("Erreur lecture CSD\r\n");
  }

  // Ecrire un bloc de test
  uint8_t writeBuf[512];
  memset(writeBuf, 0xAA, sizeof(writeBuf));
  strcpy((char*)writeBuf, "Hello SD card!");

  if (SDCARD_WriteSingleBlock(0, writeBuf) == 0) {
      USB_Printf("Ecriture bloc 0 OK\r\n");
  } else {
      USB_Printf("Erreur ecriture bloc 0\r\n");
  }

  // Relire ce bloc pour vérifier
  uint8_t readBuf[512] = {0};
  if (SDCARD_ReadSingleBlock(0, readBuf) == 0) {
      USB_Printf("Lecture bloc 0 OK : %s\r\n", (char*)readBuf);
  } else {
      USB_Printf("Erreur lecture bloc 0\r\n");
  }

  // Lecture multi-blocs (blocs 0 à 3)
  uint8_t multiBuf[512];
  if (SDCARD_ReadBegin(0) == 0) {
      for (int i = 0; i < 4; i++) {
          if (SDCARD_ReadData(multiBuf) != 0) {
              USB_Printf("Erreur lecture multi-bloc %d\r\n", i);
              break;
          }
          USB_Printf("Bloc %d lu, premier octet = 0x%02X\r\n", i, multiBuf[0]);
      }
      SDCARD_ReadEnd();
  }

  /* --- Fin Code SD Card --- */

  /* --- Début Code QMC5883P --- */

  if (!qmc5883p_init(&qmc, &hi2c1, QMC5883P_I2C_ADDR))
  {
      USB_Printf("QMC5883P init failed\r\n");
      Error_Handler();
  }
  USB_Printf("QMC5883P ready\r\n");

  /* Offsets/scales par defaut, a remplacer par les valeurs de calibration */
  qmc5883p_set_offsets(&qmc, 0.0f, 0.0f, 0.0f);
  qmc5883p_set_scales(&qmc, 1.0f, 1.0f, 1.0f);

  /* --- Fin Code QMC5883P --- */

  /* --- Début Code LoRa SX1278 (Ra-02) --- */

  myLora = newLoRa();          // hSPIx est deja fixe sur hspi2 dans la bibliotheque

  myLora.CS_port    = GPIOA;
  myLora.CS_pin     = GPIO_PIN_7;   // NSS
  myLora.reset_port = GPIOB;
  myLora.reset_pin  = GPIO_PIN_0;   // RESET
  myLora.DIO0_port  = GPIOA;
  myLora.DIO0_pin   = GPIO_PIN_6;   // DIO0

  // NSS doit etre au repos a l'etat haut (module non selectionne)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

  LoRa_reset(&myLora);

  uint16_t loraStatus = LoRa_init(&myLora);
  if (loraStatus != LORA_OK)
  {
      USB_Printf("LoRa init failed: %d\r\n", loraStatus);
      Error_Handler();
  }
  USB_Printf("LoRa ready\r\n");

  LoRa_startReceiving(&myLora);

  /* --- Fin Code LoRa SX1278 --- */

  /* --- Debut Code GPS (USART1) --- */

  GPS_Init();   // arme la reception 1 octet en IT sur huart1

  /* --- Fin Code GPS --- */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

      MPU_calcAttitude(&hspi2, &MPU9250);
      int16_t roll = roundf(10 * MPU9250.attitude.r);
      uint8_t rollDecimal = abs(roll % 10);
      int16_t pitch = roundf(10 * MPU9250.attitude.p);
      uint8_t pitchDecimal = abs(pitch % 10);
      int16_t yaw = roundf(10 * MPU9250.attitude.y);
      uint8_t yawDecimal = abs(yaw % 10);

      USB_Printf("%d.%d,%d.%d,%d.%d\n\r",roll/10, rollDecimal, pitch/10, pitchDecimal, yaw/10, yawDecimal);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

      int32_t t100;
      uint32_t p256;

      if (BMP280_ReadCompensated(&bmp280, &t100, &p256) == BMP280_OK) {
          USB_Printf("T = %ld.%02ld C   P = %lu.%02lu hPa\r\n",
                     t100 / 100, abs((int)(t100 % 100)),
                     p256 / 256, ((p256 % 256) * 100) / 256);
      } else {
          USB_Printf("BMP280 read error\r\n");
      }

      HAL_Delay(1000);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

      HAL_Delay(1000);

      if (qmc_dataReady)
      {
          qmc_dataReady = false;

          float xyz[3];
          if (qmc5883p_read_xyz(&qmc, xyz))
          {
              float heading = qmc5883p_get_heading(&qmc, 0.0f); /* 0.0f = declinaison locale */
              USB_Printf("Mag X=%.2f Y=%.2f Z=%.2f  Heading=%.1f deg\r\n",
                         xyz[0], xyz[1], xyz[2], heading);
          }
      }

      /* --- Affichage GPS --- */
      if (GPS.lock > 0) {
          USB_Printf("GPS lat=%.6f lon=%.6f sat=%d hdop=%.1f\r\n",
                     GPS.dec_latitude, GPS.dec_longitude,
                     GPS.satelites, GPS.hdop);
      } else {
          USB_Printf("GPS: pas de fix\r\n");
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
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
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /* Necessaire pour que HAL_UART_Receive_IT (utilise par GPS_Init) fonctionne :
   * si le fichier stm32f4xx_hal_msp.c (genere par CubeMX) active deja
   * USART1_IRQn dans HAL_UART_MspInit(), ces deux lignes sont redondantes
   * mais sans danger. */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* EXTI interrupt init for PA5 (QMC5883P DRDY) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5) /* PA5 = QMC5883P DRDY */
    {
        qmc_dataReady = true;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        GPS_UART_CallBack();
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
