/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_QMC5883L QMC5883L Functions
 * @brief Deals with the hardware interface to the QMC5883L magnetometer
 * @{
 *
 * @file       pios_qmc5883l.c
 * @author     The LibrePilot Project, http://www.librepilot.org, Copyright (C) 2018
 * @brief      QMC5883L functions implementation.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "pios.h"
#ifdef PIOS_INCLUDE_QMC5883L

#include <pios_sensors.h>
#include "pios_qmc5883l.h"

#define PIOS_QMC5883L_I2C_ADDR                 0x0D
#define PIOS_QMC5883L_CHIP_ID                  0xFF

#define PIOS_QMC5883L_REG_DATA                 0x00
#define PIOS_QMC5883L_REG_STATUS               0x06
#define PIOS_QMC5883L_REG_TEMPL                0x07
#define PIOS_QMC5883L_REG_TEMPH                0x08
#define PIOS_QMC5883L_REG_CTRL1                0x09
#define PIOS_QMC5883L_REG_CTRL2                0x0A
#define PIOS_QMC5883L_REG_FBR                  0x0B
#define PIOS_QMC5883L_REG_CHIP_ID              0x0D


#define    PIOS_QMC5883L_CTRL1_ODR_10HZ        (0 << 2)
#define    PIOS_QMC5883L_CTRL1_ODR_50HZ        (1 << 2)
#define    PIOS_QMC5883L_CTRL1_ODR_100HZ       (2 << 2)
#define    PIOS_QMC5883L_CTRL1_ODR_200HZ       (3 << 2)

#define    PIOS_QMC5883L_CTRL1_OSR_512         (0 << 6)
#define    PIOS_QMC5883L_CTRL1_OSR_256         (1 << 6)
#define    PIOS_QMC5883L_CTRL1_OSR_128         (2 << 6)
#define    PIOS_QMC5883L_CTRL1_OSR_64          (3 << 6)

#define    PIOS_QMC5883L_CTRL1_RNG_2G          (0 << 4)
#define    PIOS_QMC5883L_CTRL1_RNG_8G          (1 << 4)

#define    PIOS_QMC5883L_CTRL1_MODE_STANDBY    (0 << 0)
#define    PIOS_QMC5883L_CTRL1_MODE_CONTINUOUS (1 << 0)

#define PIOS_QMC5883L_CTRL2_INT_ENB            (1 << 0)
#define PIOS_QMC5883L_CTRL2_ROL_PNT            (1 << 6)
#define PIOS_QMC5883L_CTRL2_SOFT_RST           (1 << 7)

#define PIOS_QMC5883L_FBR_RECOMMENDED          0x01

#define    PIOS_QMC5883L_STATUS_DOR            (1 << 2)
#define    PIOS_QMC5883L_STATUS_OVL            (1 << 1)
#define    PIOS_QMC5883L_STATUS_DRDY           (1 << 0)


#define PIOS_QMC5883L_I2C_CONFIG_RETRY_DELAY   1000000
#define PIOS_QMC5883L_I2C_CONFIG_DATA_DELAY    10000
#define PIOS_QMC5883L_I2C_RETRIES              2

enum pios_qmc5883l_dev_magic {
    PIOS_QMC5883L_MAGIC = 0xF8D3262A
};

struct pios_qmc5883l_dev {
    enum pios_qmc5883l_dev_magic magic;
    uint32_t i2c_id;

    bool     sensorIsAlive;
    uint32_t configTime;
    uint32_t readTime;

    int16_t  data_X, data_Y, data_Z;
};


static int32_t PIOS_QMC5883L_Read(uintptr_t i2c_id, uint8_t address, uint8_t *buffer, uint8_t len);
static int32_t PIOS_QMC5883L_Write(uintptr_t i2c_id, uint8_t address, uint8_t buffer);
static int32_t PIOS_QMC5883L_Configure(struct pios_qmc5883l_dev *dev);

// sensor driver interface
static bool PIOS_QMC5883L_driver_Test(uintptr_t context);
static void PIOS_QMC5883L_driver_Reset(uintptr_t context);
static void PIOS_QMC5883L_driver_get_scale(float *scales, uint8_t size, uintptr_t context);
static void PIOS_QMC5883L_driver_fetch(void *, uint8_t size, uintptr_t context);
static bool PIOS_QMC5883L_driver_poll(uintptr_t context);

const PIOS_SENSORS_Driver PIOS_QMC5883L_Driver = {
    .test      = PIOS_QMC5883L_driver_Test,
    .poll      = PIOS_QMC5883L_driver_poll,
    .fetch     = PIOS_QMC5883L_driver_fetch,
    .reset     = PIOS_QMC5883L_driver_Reset,
    .get_queue = NULL,
    .get_scale = PIOS_QMC5883L_driver_get_scale,
    .is_polled = true,
};

static bool PIOS_QMC5883L_Validate(struct pios_qmc5883l_dev *dev)
{
    return dev && (dev->magic == PIOS_QMC5883L_MAGIC);
}

pios_qmc5883l_dev_t PIOS_QMC5883L_Init(uint32_t i2c_device)
{
    struct pios_qmc5883l_dev *dev = (struct pios_qmc5883l_dev *)pios_malloc(sizeof(*dev));

    memset(dev, 0, sizeof(*dev));

    dev->i2c_id = i2c_device;
    dev->magic  = PIOS_QMC5883L_MAGIC;

    PIOS_SENSORS_Register(&PIOS_QMC5883L_Driver, PIOS_SENSORS_TYPE_3AXIS_AUXMAG, (uintptr_t)dev);

    return dev;
}

static int32_t PIOS_QMC5883L_Configure(struct pios_qmc5883l_dev *dev)
{
    // read chip id?
    uint8_t chip_id;

    if (dev->sensorIsAlive) {
        return 0;
    }

    if (PIOS_DELAY_DiffuS(dev->configTime) < PIOS_QMC5883L_I2C_CONFIG_RETRY_DELAY) { // Do not reinitialize too often
        return -1;
    }

    dev->configTime    = PIOS_DELAY_GetRaw();
    // Reset chip
    dev->sensorIsAlive = (PIOS_QMC5883L_Write(dev->i2c_id, PIOS_QMC5883L_REG_CTRL2, PIOS_QMC5883L_CTRL2_SOFT_RST) == 0);
    if (!dev->sensorIsAlive) {
        return -1;
    }

    // Read ID
    dev->sensorIsAlive = (PIOS_QMC5883L_Read(dev->i2c_id, PIOS_QMC5883L_REG_CHIP_ID, &chip_id, sizeof(chip_id)) == 0);
    if (!dev->sensorIsAlive) {
        return -1;
    }

    if (chip_id != PIOS_QMC5883L_CHIP_ID) {
        return -2;
    }

    // Set FBR
    dev->sensorIsAlive = (PIOS_QMC5883L_Write(dev->i2c_id, PIOS_QMC5883L_REG_FBR, PIOS_QMC5883L_FBR_RECOMMENDED) == 0);
    if (!dev->sensorIsAlive) {
        return -1;
    }

    // Set control registers
    dev->sensorIsAlive = (PIOS_QMC5883L_Write(dev->i2c_id, PIOS_QMC5883L_REG_CTRL1, PIOS_QMC5883L_CTRL1_MODE_CONTINUOUS | PIOS_QMC5883L_CTRL1_ODR_200HZ | PIOS_QMC5883L_CTRL1_OSR_512 | PIOS_QMC5883L_CTRL1_RNG_8G) == 0);
    if (!dev->sensorIsAlive) {
        return -1;
    }

    return 0;
}

/**
 * Reads one or more bytes into a buffer
 * \param[in] the command indicating the address to read
 * \param[out] buffer destination buffer
 * \param[in] len number of bytes which should be read
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 */
static int32_t PIOS_QMC5883L_Read(uintptr_t i2c_id, uint8_t address, uint8_t *buffer, uint8_t len)
{
    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = PIOS_QMC5883L_I2C_ADDR,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = 1,
            .buf  = &address,
        }
        ,
        {
            .info = __func__,
            .addr = PIOS_QMC5883L_I2C_ADDR,
            .rw   = PIOS_I2C_TXN_READ,
            .len  = len,
            .buf  = buffer,
        }
    };

    for (uint8_t retry = PIOS_QMC5883L_I2C_RETRIES; retry > 0; --retry) {
        if (PIOS_I2C_Transfer(i2c_id, txn_list, NELEMENTS(txn_list)) == 0) {
            return 0;
        }
    }

    return -1;
}

static int32_t PIOS_QMC5883L_Write(uintptr_t i2c_id, uint8_t address, uint8_t value)
{
    uint8_t data[] = { address, value };

    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = PIOS_QMC5883L_I2C_ADDR,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = sizeof(data),
            .buf  = data,
        }
    };

    for (uint8_t retry = PIOS_QMC5883L_I2C_RETRIES; retry > 0; --retry) {
        if (PIOS_I2C_Transfer(i2c_id, txn_list, NELEMENTS(txn_list)) == 0) {
            return 0;
        }
    }

    return -1;
}

bool PIOS_QMC5883L_driver_Test(__attribute__((unused)) uintptr_t context)
{
    return true;
}

static void PIOS_QMC5883L_driver_Reset(__attribute__((unused)) uintptr_t context)
{}

static void PIOS_QMC5883L_driver_get_scale(float *scales, uint8_t size, __attribute__((unused))  uintptr_t context)
{
    PIOS_Assert(size > 0);
    scales[0] = 0.5;
}

static void PIOS_QMC5883L_driver_fetch(void *data, __attribute__((unused)) uint8_t size, uintptr_t context)
{
    struct pios_qmc5883l_dev *dev = (struct pios_qmc5883l_dev *)context;

    PIOS_Assert(PIOS_QMC5883L_Validate(dev));
    PIOS_Assert(data);

    PIOS_SENSORS_3Axis_SensorsWithTemp *tmp = data;

    tmp->count = 1;
    tmp->sample[0].x = dev->data_X;
    tmp->sample[0].y = dev->data_Y;
    tmp->sample[0].z = dev->data_Z;
    tmp->temperature = 0;
}

static bool PIOS_QMC5883L_driver_poll(uintptr_t context)
{
    struct pios_qmc5883l_dev *dev = (struct pios_qmc5883l_dev *)context;

    PIOS_Assert(PIOS_QMC5883L_Validate(dev));

    if (!dev->sensorIsAlive) {
        if (PIOS_QMC5883L_Configure(dev) < 0) {
            return false;
        }
    }

    if (PIOS_DELAY_DiffuS(dev->readTime) < PIOS_QMC5883L_I2C_CONFIG_DATA_DELAY) {
        return false;
    }

    // Read Status
    uint8_t status;
    dev->sensorIsAlive = (PIOS_QMC5883L_Read(dev->i2c_id, PIOS_QMC5883L_REG_STATUS, &status, sizeof(status)) == 0);
    if (!dev->sensorIsAlive) {
        return false;
    }

    // Is it ready?
    if (!(status & PIOS_QMC5883L_STATUS_DRDY)) {
        return false;
    }

    uint8_t data[6];
    dev->sensorIsAlive = (PIOS_QMC5883L_Read(dev->i2c_id, PIOS_QMC5883L_REG_DATA, data, sizeof(data)) == 0);
    if (!dev->sensorIsAlive) {
        return false;
    }

    dev->readTime = PIOS_DELAY_GetRaw();

    dev->data_X   = (int16_t)(data[1] << 8 | data[0]);
    dev->data_Y   = (int16_t)(data[3] << 8 | data[2]);
    dev->data_Z   = (int16_t)(data[5] << 8 | data[4]);

    return true;
}


#endif /* PIOS_INCLUDE_QMC5883L */
