/*
 * BMP280.c
 *
 * BMP280 SPI driver implementation for STM32F411 + HAL
 */

#include "BMP280.h"

/* ---- Control du CS (Active LOW) ------------------------------------- */
static inline void bmp280_cs_low(BMP280_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void bmp280_cs_high(BMP280_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

#define BMP280_SPI_TIMEOUT_MS  100u

/* ---- Protocoles SPI bas niveau --------------------------------------- */

BMP280_Status_t BMP280_ReadRegs(BMP280_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = reg | 0x80u; /* Bit MSB=1 indique une lecture SPI */
    HAL_StatusTypeDef st;

    bmp280_cs_low(dev);

    st = HAL_SPI_Transmit(dev->hspi, &addr, 1, BMP280_SPI_TIMEOUT_MS);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(dev->hspi, buf, len, BMP280_SPI_TIMEOUT_MS);
    }

    bmp280_cs_high(dev);

    if (st == HAL_TIMEOUT) return BMP280_ERR_TIMEOUT;
    if (st != HAL_OK)      return BMP280_ERR_SPI;
    return BMP280_OK;
}

BMP280_Status_t BMP280_WriteReg(BMP280_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    HAL_StatusTypeDef st;

    tx[0] = reg & 0x7Fu; /* Bit MSB=0 indique une écriture SPI */
    tx[1] = value;

    bmp280_cs_low(dev);
    st = HAL_SPI_Transmit(dev->hspi, tx, 2, BMP280_SPI_TIMEOUT_MS);
    bmp280_cs_high(dev);

    if (st == HAL_TIMEOUT) return BMP280_ERR_TIMEOUT;
    if (st != HAL_OK)      return BMP280_ERR_SPI;
    return BMP280_OK;
}

/* ---- Configuration et Initialisation ---------------------------------- */

void BMP280_Attach(BMP280_t *dev, SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->t_fine  = 0;
    bmp280_cs_high(dev); /* Ligne CS au repos HIGH */
}

BMP280_Status_t BMP280_ReadChipId(BMP280_t *dev, uint8_t *id_out)
{
    return BMP280_ReadRegs(dev, BMP280_REG_ID, id_out, 1);
}

BMP280_Status_t BMP280_SoftReset(BMP280_t *dev)
{
    BMP280_Status_t st = BMP280_WriteReg(dev, BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    if (st != BMP280_OK) return st;
    HAL_Delay(5); /* Attente requise après réinitialisation */
    return BMP280_OK;
}

static BMP280_Status_t bmp280_read_calibration(BMP280_t *dev)
{
    uint8_t raw[24];
    BMP280_Status_t st = BMP280_ReadRegs(dev, BMP280_REG_CALIB_START, raw, sizeof(raw));
    if (st != BMP280_OK) return st;

    dev->calib.dig_T1 = (uint16_t)(raw[0]  | (raw[1]  << 8));
    dev->calib.dig_T2 = (int16_t) (raw[2]  | (raw[3]  << 8));
    dev->calib.dig_T3 = (int16_t) (raw[4]  | (raw[5]  << 8));

    dev->calib.dig_P1 = (uint16_t)(raw[6]  | (raw[7]  << 8));
    dev->calib.dig_P2 = (int16_t) (raw[8]  | (raw[9]  << 8));
    dev->calib.dig_P3 = (int16_t) (raw[10] | (raw[11] << 8));
    dev->calib.dig_P4 = (int16_t) (raw[12] | (raw[13] << 8));
    dev->calib.dig_P5 = (int16_t) (raw[14] | (raw[15] << 8));
    dev->calib.dig_P6 = (int16_t) (raw[16] | (raw[17] << 8));
    dev->calib.dig_P7 = (int16_t) (raw[18] | (raw[19] << 8));
    dev->calib.dig_P8 = (int16_t) (raw[20] | (raw[21] << 8));
    dev->calib.dig_P9 = (int16_t) (raw[22] | (raw[23] << 8));

    return BMP280_OK;
}

BMP280_Status_t BMP280_Init(BMP280_t *dev,
                             BMP280_Oversampling_t osrs_t,
                             BMP280_Oversampling_t osrs_p,
                             BMP280_Mode_t mode,
                             BMP280_Standby_t standby,
                             BMP280_Filter_t filter)
{
    uint8_t id = 0;
    BMP280_Status_t st;

    st = BMP280_ReadChipId(dev, &id);
    if (st != BMP280_OK) return st;
    if (id != BMP280_CHIP_ID) return BMP280_ERR_ID;

    st = BMP280_SoftReset(dev);
    if (st != BMP280_OK) return st;

    st = bmp280_read_calibration(dev);
    if (st != BMP280_OK) return st;

    uint8_t config_reg = (uint8_t)((standby << 5) | (filter << 2));
    st = BMP280_WriteReg(dev, BMP280_REG_CONFIG, config_reg);
    if (st != BMP280_OK) return st;

    uint8_t ctrl_meas = (uint8_t)((osrs_t << 5) | (osrs_p << 2) | mode);
    st = BMP280_WriteReg(dev, BMP280_REG_CTRL_MEAS, ctrl_meas);
    if (st != BMP280_OK) return st;

    return BMP280_OK;
}

BMP280_Status_t BMP280_TriggerForcedMeasurement(BMP280_t *dev)
{
    uint8_t ctrl_meas;
    BMP280_Status_t st = BMP280_ReadRegs(dev, BMP280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (st != BMP280_OK) return st;

    ctrl_meas = (uint8_t)((ctrl_meas & ~0x03u) | BMP280_MODE_FORCED);
    return BMP280_WriteReg(dev, BMP280_REG_CTRL_MEAS, ctrl_meas);
}

BMP280_Status_t BMP280_WaitMeasuring(BMP280_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status;

    do {
        BMP280_Status_t st = BMP280_ReadRegs(dev, BMP280_REG_STATUS, &status, 1);
        if (st != BMP280_OK) return st;

        if ((status & 0x08u) == 0) {
            return BMP280_OK;
        }
    } while ((HAL_GetTick() - start) < timeout_ms);

    return BMP280_ERR_TIMEOUT;
}

/* ---- Algorithmes de compensation Bosch BMP280 ------------------------ */

BMP280_Status_t BMP280_ReadRaw(BMP280_t *dev, int32_t *raw_temp, int32_t *raw_press)
{
    uint8_t buf[6];
    BMP280_Status_t st = BMP280_ReadRegs(dev, BMP280_REG_PRESS_MSB, buf, sizeof(buf));
    if (st != BMP280_OK) return st;

    *raw_press = (int32_t)(((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4));
    *raw_temp  = (int32_t)(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4));

    return BMP280_OK;
}

static int32_t bmp280_compensate_T_int32(BMP280_t *dev, int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * (int32_t)dev->calib.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)dev->calib.dig_T1) *
              ((adc_T >> 4) - (int32_t)dev->calib.dig_T1)) >> 12) * (int32_t)dev->calib.dig_T3) >> 14;

    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T; /* en 0.01 °C */
}

static uint32_t bmp280_compensate_P_int64(BMP280_t *dev, int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = (int64_t)dev->t_fine - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) +
           ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * (int64_t)dev->calib.dig_P1 >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

    return (uint32_t)p; /* Format Q24.8 (Pa = p / 256) */
}

BMP280_Status_t BMP280_ReadCompensated(BMP280_t *dev,
                                        int32_t *temperature_c100,
                                        uint32_t *pressure_pa256)
{
    int32_t raw_t, raw_p;
    BMP280_Status_t st = BMP280_ReadRaw(dev, &raw_t, &raw_p);
    if (st != BMP280_OK) return st;

    *temperature_c100 = bmp280_compensate_T_int32(dev, raw_t);
    *pressure_pa256    = bmp280_compensate_P_int64(dev, raw_p);

    return BMP280_OK;
}

BMP280_Status_t BMP280_ReadFloat(BMP280_t *dev, float *temperature_c, float *pressure_hpa)
{
    int32_t t100;
    uint32_t p256;

    BMP280_Status_t st = BMP280_ReadCompensated(dev, &t100, &p256);
    if (st != BMP280_OK) return st;

    *temperature_c = (float)t100 / 100.0f;
    *pressure_hpa  = ((float)p256 / 256.0f) / 100.0f; /* Pa -> hPa */

    return BMP280_OK;
}

/* Algorithme Hypsométrique pour le calcul d'altitude */
float BMP280_CalculateAltitude(float pressure_hpa, float sea_level_hpa)
{
    if (pressure_hpa <= 0.0f || sea_level_hpa <= 0.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.19029495f));
}

BMP280_Status_t BMP280_ReadAltitude(BMP280_t *dev, float sea_level_hpa,
                                     float *temperature_c, float *pressure_hpa, float *altitude_m)
{
    BMP280_Status_t st = BMP280_ReadFloat(dev, temperature_c, pressure_hpa);
    if (st != BMP280_OK) return st;

    *altitude_m = BMP280_CalculateAltitude(*pressure_hpa, sea_level_hpa);
    return BMP280_OK;
}
