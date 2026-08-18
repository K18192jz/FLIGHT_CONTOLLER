/*
 * BMP280.h
 *
 * BMP280 pressure/temperature sensor driver - SPI (4-wire), STM32F411 + HAL
 * Specifically configured for SPI2 and PC15 Chip Select (CS)
 *
 * Wiring for STM32F411:
 *   SCK  -> PB13 (SPI2_SCK) or PB10
 *   SDI  -> PB15 (SPI2_MOSI)
 *   SDO  -> PB14 (SPI2_MISO)
 *   CSB  -> PC15 (GPIO CS, active LOW)
 */

#ifndef BMP280_SPI_H
#define BMP280_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ---- Standard Sea Level Pressure Definition --------------------------- */
#ifndef SEALEVEL_PRESSURE_HPA
#define SEALEVEL_PRESSURE_HPA 1013.25f
#endif

/* ---- Register map ---------------------------------------------------- */
#define BMP280_REG_CALIB_START   0x88u   /* 0x88 - 0xA1, 26 bytes */
#define BMP280_REG_ID            0xD0u
#define BMP280_REG_RESET         0xE0u
#define BMP280_REG_STATUS        0xF3u
#define BMP280_REG_CTRL_MEAS     0xF4u
#define BMP280_REG_CONFIG        0xF5u
#define BMP280_REG_PRESS_MSB     0xF7u
#define BMP280_REG_PRESS_LSB     0xF8u
#define BMP280_REG_PRESS_XLSB    0xF9u
#define BMP280_REG_TEMP_MSB      0xFAu
#define BMP280_REG_TEMP_LSB      0xFBu
#define BMP280_REG_TEMP_XLSB     0xFCu

#define BMP280_CHIP_ID           0x58u   /* expected value in ID reg */
#define BMP280_SOFT_RESET_CMD    0xB6u

/* ---- ctrl_meas: oversampling + power mode ----------------------------- */
typedef enum {
    BMP280_OSRS_SKIP = 0x0,
    BMP280_OSRS_X1   = 0x1,
    BMP280_OSRS_X2   = 0x2,
    BMP280_OSRS_X4   = 0x3,
    BMP280_OSRS_X8   = 0x4,
    BMP280_OSRS_X16  = 0x5,
} BMP280_Oversampling_t;

typedef enum {
    BMP280_MODE_SLEEP  = 0x0,
    BMP280_MODE_FORCED = 0x1,   /* 0x2 also maps to forced */
    BMP280_MODE_NORMAL = 0x3,
} BMP280_Mode_t;

/* ---- config: standby time + IIR filter -------------------------------- */
typedef enum {
    BMP280_STANDBY_0_5MS  = 0x0,
    BMP280_STANDBY_62_5MS = 0x1,
    BMP280_STANDBY_125MS  = 0x2,
    BMP280_STANDBY_250MS  = 0x3,
    BMP280_STANDBY_500MS  = 0x4,
    BMP280_STANDBY_1000MS = 0x5,
    BMP280_STANDBY_2000MS = 0x6,
    BMP280_STANDBY_4000MS = 0x7,
} BMP280_Standby_t;

typedef enum {
    BMP280_FILTER_OFF = 0x0,
    BMP280_FILTER_2   = 0x1,
    BMP280_FILTER_4   = 0x2,
    BMP280_FILTER_8   = 0x3,
    BMP280_FILTER_16  = 0x4,
} BMP280_Filter_t;

typedef enum {
    BMP280_OK = 0,
    BMP280_ERR_SPI,
    BMP280_ERR_ID,
    BMP280_ERR_TIMEOUT,
} BMP280_Status_t;

/* ---- Calibration data (read once from NVM at init) --------------------- */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_Calib_t;

/* ---- Device handle ------------------------------------------------------ */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef       *cs_port;
    uint16_t            cs_pin;

    BMP280_Calib_t       calib;
    int32_t              t_fine;   /* carries temp->pressure compensation state */
} BMP280_t;

/* ---- API Functions ------------------------------------------------------- */

/* Attache l'instance SPI2 et le Pin CS (ex: GPIOC, GPIO_PIN_15) */
void BMP280_Attach(BMP280_t *dev, SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Vérifie l'ID du capteur, effectue un soft-reset et lit la calibration */
BMP280_Status_t BMP280_Init(BMP280_t *dev,
                             BMP280_Oversampling_t osrs_t,
                             BMP280_Oversampling_t osrs_p,
                             BMP280_Mode_t mode,
                             BMP280_Standby_t standby,
                             BMP280_Filter_t filter);

BMP280_Status_t BMP280_SoftReset(BMP280_t *dev);
BMP280_Status_t BMP280_ReadChipId(BMP280_t *dev, uint8_t *id_out);

BMP280_Status_t BMP280_TriggerForcedMeasurement(BMP280_t *dev);
BMP280_Status_t BMP280_WaitMeasuring(BMP280_t *dev, uint32_t timeout_ms);

/* Lecture des valeurs brutes ADC */
BMP280_Status_t BMP280_ReadRaw(BMP280_t *dev, int32_t *raw_temp, int32_t *raw_press);

/* Lecture des valeurs compensées (brutes Bosch) */
BMP280_Status_t BMP280_ReadCompensated(BMP280_t *dev,
                                        int32_t *temperature_c100,
                                        uint32_t *pressure_pa256);

/* Conversion directe en valeurs float (C° et hPa) */
BMP280_Status_t BMP280_ReadFloat(BMP280_t *dev, float *temperature_c, float *pressure_hpa);

/* Calcul d'altitude barométrique */
float BMP280_CalculateAltitude(float pressure_hpa, float sea_level_hpa);

/* Combined read: Température, Pression et Altitude en une seule étape */
BMP280_Status_t BMP280_ReadAltitude(BMP280_t *dev, float sea_level_hpa,
                                     float *temperature_c, float *pressure_hpa, float *altitude_m);

/* Registres bas niveau */
BMP280_Status_t BMP280_ReadRegs(BMP280_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
BMP280_Status_t BMP280_WriteReg(BMP280_t *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_SPI_H */
