/*
 * BMP280.h
 *
 * BMP280 pressure/temperature sensor driver - SPI (4-wire), STM32F411 + HAL
 *
 * Wiring:
 *   SCK  -> SPIx_SCK
 *   SDI  -> SPIx_MOSI   (sensor's SDI pin)
 *   SDO  -> SPIx_MISO   (sensor's SDO pin)
 *   CSB  -> any GPIO, driven manually as chip select (active low)
 *
 * Notes on the BMP280 SPI protocol:
 *   - Mode 0 (CPOL=0, CPHA=0) or Mode 3 (CPOL=1, CPHA=1) both work.
 *   - Max SPI clock is 10 MHz.
 *   - Register read:  first byte = (addr | 0x80), then clock out dummy
 *     bytes to receive the data.
 *   - Register write: first byte = (addr & 0x7F), second byte = data.
 *     Only single-byte writes are supported by the chip.
 */

#ifndef BMP280_SPI_H
#define BMP280_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

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

/* ---- API ----------------------------------------------------------------- */

/* Wire up the handle. Call before Init(). CS pin must already be configured
 * as GPIO output push-pull, idle HIGH, by your MX_GPIO_Init(). */
void BMP280_Attach(BMP280_t *dev, SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Verifies chip ID, soft-resets the sensor, reads calibration trim values,
 * and programs ctrl_meas/config with the given settings. */
BMP280_Status_t BMP280_Init(BMP280_t *dev,
                             BMP280_Oversampling_t osrs_t,
                             BMP280_Oversampling_t osrs_p,
                             BMP280_Mode_t mode,
                             BMP280_Standby_t standby,
                             BMP280_Filter_t filter);

BMP280_Status_t BMP280_SoftReset(BMP280_t *dev);
BMP280_Status_t BMP280_ReadChipId(BMP280_t *dev, uint8_t *id_out);

/* Forces a single one-shot conversion when the device is in SLEEP mode
 * (only meaningful if you initialized with BMP280_MODE_FORCED and want to
 * trigger each sample manually). Returns once BMP280_WaitMeasuring() clears. */
BMP280_Status_t BMP280_TriggerForcedMeasurement(BMP280_t *dev);

/* Polls the status register until the "measuring" bit clears (or timeout). */
BMP280_Status_t BMP280_WaitMeasuring(BMP280_t *dev, uint32_t timeout_ms);

/* Reads raw 20-bit ADC values for temperature and pressure in one burst. */
BMP280_Status_t BMP280_ReadRaw(BMP280_t *dev, int32_t *raw_temp, int32_t *raw_press);

/* Full reading, compensated to real-world units.
 * temperature_c100  = temperature in 0.01 degC (e.g. 2534 -> 25.34 C)
 * pressure_pa256     = pressure in Pa, Q24.8 fixed point (divide by 256 for Pa)
 */
BMP280_Status_t BMP280_ReadCompensated(BMP280_t *dev,
                                        int32_t *temperature_c100,
                                        uint32_t *pressure_pa256);

/* Convenience float wrapper (uses the fixed-point path internally, then
 * converts - keeps a single source of truth for the compensation math). */
BMP280_Status_t BMP280_ReadFloat(BMP280_t *dev, float *temperature_c, float *pressure_hpa);

/* Low-level single/multi register access, exposed in case you need it
 * directly (e.g. reading OTP/NVM regions not covered above). */
BMP280_Status_t BMP280_ReadRegs(BMP280_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);
BMP280_Status_t BMP280_WriteReg(BMP280_t *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_SPI_H */
