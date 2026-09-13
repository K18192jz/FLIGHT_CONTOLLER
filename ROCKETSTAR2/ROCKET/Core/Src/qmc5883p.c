/*
 * qmc5883p.c
 */

#include "qmc5883p.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* ===================== REGISTERS ===================== */
#define REG_CHIP_ID        0x00
#define REG_DATA_OUT_X_LSB 0x01
#define REG_STATUS         0x09
#define REG_CTL1           0x0A
#define REG_CTL2           0x0B

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f
#define TWO_PI      6.2831853f

/* ===================== INIT ===================== */

bool qmc5883p_init(QMC5883P_HandleTypeDef *dev, I2C_HandleTypeDef *i2c, uint16_t addr)
{
    dev->i2c = i2c;
    dev->addr = addr << 1;

    dev->offX = 0;
    dev->offY = 0;
    dev->offZ = 0;

    dev->scaleX = 1;
    dev->scaleY = 1;
    dev->scaleZ = 1;

    dev->rawX = 0;
    dev->rawY = 0;
    dev->rawZ = 0;

    dev->lastRead = 0;

    uint8_t id = 0;

    if (!qmc5883p_read_reg(dev, REG_CHIP_ID, &id, 1))
        return false;

    if (id != 0x80)
        return false;

    HAL_Delay(10);
    qmc5883p_write_reg(dev, 0x0D, 0x40);
    HAL_Delay(10);
    qmc5883p_write_reg(dev, 0x29, 0x06);
    HAL_Delay(10);
    qmc5883p_write_reg(dev, REG_CTL1, 0xCF);
    HAL_Delay(10);
    qmc5883p_write_reg(dev, REG_CTL2, 0x00);

    return true;
}

/* ===================== LOW LEVEL I2C ===================== */

bool qmc5883p_read_reg(QMC5883P_HandleTypeDef *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Mem_Read(dev->i2c,
                         dev->addr,
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         len,
                         100) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool qmc5883p_write_reg(QMC5883P_HandleTypeDef *dev, uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg, val};
    return (HAL_I2C_Master_Transmit(dev->i2c, dev->addr, data, 2, 100) == HAL_OK);
}

/* ===================== RAW READING (NON-BLOCKING STREAM) ===================== */

bool qmc5883p_read_raw(QMC5883P_HandleTypeDef *dev)
{
    uint8_t buf[6];

    // Direct read (NO status filtering, NO timing blocking)
    if (!qmc5883p_read_reg(dev, REG_DATA_OUT_X_LSB, buf, 6))
        return false;

    dev->rawX = (int16_t)((buf[1] << 8) | buf[0]);
    dev->rawY = (int16_t)((buf[3] << 8) | buf[2]);
    dev->rawZ = (int16_t)((buf[5] << 8) | buf[4]);

    dev->lastRead = HAL_GetTick();

    return true;
}

/* ===================== XYZ OUTPUT ===================== */

bool qmc5883p_read_xyz(QMC5883P_HandleTypeDef *dev, float *xyz)
{
    // Try update, but NEVER block output
    qmc5883p_read_raw(dev);

    float x = dev->rawX / 1000.0f;
    float y = dev->rawY / 1000.0f;
    float z = dev->rawZ / 1000.0f;

    xyz[0] = (x - dev->offX) * dev->scaleX;
    xyz[1] = (y - dev->offY) * dev->scaleY;
    xyz[2] = (z - dev->offZ) * dev->scaleZ;

    return true;
}

/* ===================== HEADING ===================== */

float qmc5883p_get_heading(QMC5883P_HandleTypeDef *dev, float declination_deg)
{
    qmc5883p_read_raw(dev);

    float x = (dev->rawX / 1000.0f - dev->offX) * dev->scaleX;
    float y = (dev->rawY / 1000.0f - dev->offY) * dev->scaleY;

    float heading = atan2f(y, x);

    heading += declination_deg * DEG_TO_RAD;

    if (heading < 0)
        heading += TWO_PI;
    else if (heading > TWO_PI)
        heading -= TWO_PI;

    return heading * RAD_TO_DEG;
}

/* ===================== CALIBRATION ===================== */

void qmc5883p_set_offsets(QMC5883P_HandleTypeDef *dev, float x, float y, float z)
{
    dev->offX = x;
    dev->offY = y;
    dev->offZ = z;
}

void qmc5883p_set_scales(QMC5883P_HandleTypeDef *dev, float x, float y, float z)
{
    dev->scaleX = x;
    dev->scaleY = y;
    dev->scaleZ = z;
}
