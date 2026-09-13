#ifndef __QMC5883P_H__
#define __QMC5883P_H__

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C address */
#define QMC5883P_I2C_ADDR  0x2C

/* Registers */
#define QMC5883P_CHIP_ID        0x00
#define QMC5883P_DATA_X_LSB     0x01
#define QMC5883P_STATUS         0x09
#define QMC5883P_CTL1           0x0A
#define QMC5883P_CTL2           0x0B

typedef struct {

    /* HAL I2C handle */
    I2C_HandleTypeDef *i2c;

    /* device address */
    uint16_t addr;

    /* calibration */
    float offX;
    float offY;
    float offZ;

    float scaleX;
    float scaleY;
    float scaleZ;

    /* last raw values */
    int16_t rawX;
    int16_t rawY;
    int16_t rawZ;

    uint32_t lastRead;

} QMC5883P_HandleTypeDef;

/* init */
bool qmc5883p_init(QMC5883P_HandleTypeDef *dev, I2C_HandleTypeDef *i2c, uint16_t addr);

/* read raw xyz (float in µT) */
bool qmc5883p_read_xyz(QMC5883P_HandleTypeDef *dev, float *xyz);

/* heading */
float qmc5883p_get_heading(QMC5883P_HandleTypeDef *dev, float declination_deg);

/* calibration */
void qmc5883p_set_offsets(QMC5883P_HandleTypeDef *dev, float x, float y, float z);
void qmc5883p_set_scales(QMC5883P_HandleTypeDef *dev, float x, float y, float z);

/* low level */
bool qmc5883p_read_reg(QMC5883P_HandleTypeDef *dev, uint8_t reg, uint8_t *buf, uint8_t len);
bool qmc5883p_write_reg(QMC5883P_HandleTypeDef *dev, uint8_t reg, uint8_t val);
bool qmc5883p_read_raw(QMC5883P_HandleTypeDef *dev);

#endif
