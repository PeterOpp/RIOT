/*
 * Copyright (C) 2015 Freie Universität Berlin
 *               2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mpu9x50
 * @{
 *
 * @file
 * @brief       Device driver implementation for the MPU-9X50 (MPU9150 and MPU9250) 9-Axis Motion Sensor
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Jannes Volkens <jannes.volkens@haw-hamburg.de>
 *
 * @}
 */

#include "mpu9x50.h"
#include "mpu9x50_regs.h"
#include "mpu9x50_internal.h"
#include "periph/i2c.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "xtimer.h"
#include "byteorder.h"

#define ENABLE_DEBUG (1)
#if ENABLE_DEBUG
#include "debug.h"
#endif

#define REG_RESET (0x00)
#define MAX_VALUE (0x7FFF)

#define DEV_I2C (dev->params.i2c)
#define DEV_ADDR (dev->params.addr)
#define DEV_COMP_ADDR (dev->params.comp_addr)

/* Default config settings */
static const mpu9x50_status_t DEFAULT_STATUS = {
    .accel_pwr = MPU9X50_SENSOR_PWR_ON,
    .gyro_pwr = MPU9X50_SENSOR_PWR_ON,
    .compass_pwr = MPU9X50_SENSOR_PWR_ON,
    .gyro_fsr = MPU9X50_GYRO_FSR_250DPS,
    .accel_fsr = MPU9X50_ACCEL_FSR_16G,
    .sample_rate = 0,
    .compass_sample_rate = 0,
    .compass_x_adj = 0,
    .compass_y_adj = 0,
    .compass_z_adj = 0,
};

/* Internal function prototypes */
static int compass_init(mpu9x50_t *dev);
static void conf_bypass(const mpu9x50_t *dev, uint8_t bypass_enable);
static void conf_lpf(const mpu9x50_t *dev, uint16_t rate);

inline int acquire_bus(const mpu9x50_t *dev);
inline void release_bus(const mpu9x50_t *dev);
inline void write_register(const mpu9x50_t *dev, uint8_t regAddress, uint8_t data);
inline void read_register(const mpu9x50_t *dev, uint8_t regAddress, uint8_t *target, uint8_t flags);
inline void read_registers(const mpu9x50_t *dev, uint8_t regAddress, uint8_t *buffer, uint16_t length);
inline float getFullRange(const mpu9x50_t *dev);
void normalize_accel_sample(const mpu9x50_t *dev, uint8_t *rawBytes, mpu9x50_results_t *result);

/*---------------------------------------------------------------------------*
 *                          MPU9X50 Core API                                 *
 *---------------------------------------------------------------------------*/

int mpu9x50_init(mpu9x50_t *dev, const mpu9x50_params_t *params)
{
    dev->params = *params;

    uint8_t temp;

    dev->conf = DEFAULT_STATUS;

    /* Acquire exclusive access */
    acquire_bus(dev);

    if (params->use_spi)
    {
        printf("Configuring SPI\n");
        spi_init(dev->params.spi);
        int res = spi_init_cs(dev->params.spi, dev->params.spi_cs);
        if (res < 0)
        {
            printf("NO SPI\n");
        }
        printf("MOU9X50: init on SPI_DEV(%u)\n", dev->params.spi);
        write_register(dev, MPU9X50_I2CDIS_REG, 0b00011011);
    }

#if ENABLE_DEBUG
    uint8_t i2cdis = 0;
    read_register(dev, MPU9X50_I2CDIS_REG, &i2cdis, 0);
    printf("I2CDis = 0x%x", i2cdis);
    uint8_t whoami = 0;
    read_register(dev, MPU9X50_WHO_AM_I_REG, &whoami, 0);
    if (whoami != 0x71)
    {
        printf("Whoami is 0x%x\n", whoami);
    }
#endif

    /* Reset MPU9X50 registers and afterwards wake up the chip */
    write_register(dev, MPU9X50_PWR_MGMT_1_REG, MPU9X50_PWR_RESET);
    xtimer_usleep(MPU9X50_RESET_SLEEP_US);
    write_register(dev, MPU9X50_PWR_MGMT_1_REG, MPU9X50_PWR_WAKEUP);

    /* Release the bus, it is acquired again inside each function */
    release_bus(dev);

    /* Set default full scale ranges and sample rate */
    mpu9x50_set_gyro_fsr(dev, MPU9X50_GYRO_FSR_2000DPS);
    mpu9x50_set_accel_fsr(dev, MPU9X50_ACCEL_FSR_2G);
    mpu9x50_set_sample_rate(dev, dev->params.sample_rate);

    /* Disable interrupt generation */
    acquire_bus(dev);
    write_register(dev, MPU9X50_INT_ENABLE_REG, REG_RESET);

    /* Initialize magnetometer */
    if (dev->params.use_spi == 0)
    {
        if (compass_init(dev))
        {
            release_bus(dev);
            return -2;
        }
    }

    /* Release the bus, it is acquired again inside each function */
    release_bus(dev);
    mpu9x50_set_compass_sample_rate(dev, 10);
    /* Enable all sensors */

    acquire_bus(dev);
    write_register(dev, MPU9X50_PWR_MGMT_1_REG, MPU9X50_PWR_PLL);
    read_register(dev, MPU9X50_PWR_MGMT_2_REG, &temp, 0);
    temp &= ~(MPU9X50_PWR_ACCEL | MPU9X50_PWR_GYRO);
    write_register(dev, MPU9X50_PWR_MGMT_2_REG, temp);
    release_bus(dev);

    xtimer_usleep(MPU9X50_PWR_CHANGE_SLEEP_US);

    /* Set Interrupt Config to keep high until status register is read*/
    uint8_t reg = BIT_INT_PIN_LATCH_INT_EN;
    write_register(dev, MPU9X50_INT_PIN_CFG_REG, reg);

    return 0;
}

int mpu9x50_set_accel_power(mpu9x50_t *dev, mpu9x50_pwr_t pwr_conf)
{
    uint8_t pwr_1_setting, pwr_2_setting;

    if (dev->conf.accel_pwr == pwr_conf)
    {
        return 0;
    }

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }

    /* Read current power management 2 configuration */
    read_register(dev, MPU9X50_PWR_MGMT_2_REG, &pwr_2_setting, 0);
    /* Prepare power register settings */
    if (pwr_conf == MPU9X50_SENSOR_PWR_ON)
    {
        pwr_1_setting = MPU9X50_PWR_WAKEUP;
        pwr_2_setting &= ~(MPU9X50_PWR_ACCEL);
    }
    else
    {
        pwr_1_setting = BIT_PWR_MGMT1_SLEEP;
        pwr_2_setting |= MPU9X50_PWR_ACCEL;
    }
    /* Configure power management 1 register if needed */
    if ((dev->conf.gyro_pwr == MPU9X50_SENSOR_PWR_OFF) && (dev->conf.compass_pwr == MPU9X50_SENSOR_PWR_OFF))
    {
        write_register(dev, MPU9X50_PWR_MGMT_1_REG, pwr_1_setting);
    }
    /* Enable/disable accelerometer standby in power management 2 register */
    write_register(dev, MPU9X50_PWR_MGMT_2_REG, pwr_2_setting);

    /* Release the bus */
    release_bus(dev);

    dev->conf.accel_pwr = pwr_conf;
    xtimer_usleep(MPU9X50_PWR_CHANGE_SLEEP_US);

    return 0;
}

int mpu9x50_set_gyro_power(mpu9x50_t *dev, mpu9x50_pwr_t pwr_conf)
{
    uint8_t pwr_2_setting;

    if (dev->conf.gyro_pwr == pwr_conf)
    {
        return 0;
    }

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }

    /* Read current power management 2 configuration */
    read_register(dev, MPU9X50_PWR_MGMT_2_REG, &pwr_2_setting, 0);
    /* Prepare power register settings */
    if (pwr_conf == MPU9X50_SENSOR_PWR_ON)
    {
        /* Set clock to pll */
        write_register(dev, MPU9X50_PWR_MGMT_1_REG, MPU9X50_PWR_PLL);
        pwr_2_setting &= ~(MPU9X50_PWR_GYRO);
    }
    else
    {
        /* Configure power management 1 register */
        if ((dev->conf.accel_pwr == MPU9X50_SENSOR_PWR_OFF) && (dev->conf.compass_pwr == MPU9X50_SENSOR_PWR_OFF))
        {
            /* All sensors turned off, put the MPU-9X50 to sleep */
            write_register(dev, MPU9X50_PWR_MGMT_1_REG, BIT_PWR_MGMT1_SLEEP);
        }
        else
        {
            /* Reset clock to internal oscillator */
            write_register(dev, MPU9X50_PWR_MGMT_1_REG, MPU9X50_PWR_WAKEUP);
        }
        pwr_2_setting |= MPU9X50_PWR_GYRO;
    }
    /* Enable/disable gyroscope standby in power management 2 register */
    write_register(dev, MPU9X50_PWR_MGMT_2_REG, pwr_2_setting);

    /* Release the bus */
    release_bus(dev);

    dev->conf.gyro_pwr = pwr_conf;
    xtimer_usleep(MPU9X50_PWR_CHANGE_SLEEP_US);

    return 0;
}

int mpu9x50_set_compass_power(mpu9x50_t *dev, mpu9x50_pwr_t pwr_conf)
{
    uint8_t pwr_1_setting, usr_ctrl_setting, s1_do_setting;

    if (dev->conf.compass_pwr == pwr_conf)
    {
        return 0;
    }

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }

    /* Read current user control configuration */
    read_register(dev, MPU9X50_USER_CTRL_REG, &usr_ctrl_setting, 0);
    /* Prepare power register settings */
    if (pwr_conf == MPU9X50_SENSOR_PWR_ON)
    {
        pwr_1_setting = MPU9X50_PWR_WAKEUP;
        s1_do_setting = MPU9X50_COMP_SINGLE_MEASURE;
        usr_ctrl_setting |= BIT_I2C_MST_EN;
    }
    else
    {
        pwr_1_setting = BIT_PWR_MGMT1_SLEEP;
        s1_do_setting = MPU9X50_COMP_POWER_DOWN;
        usr_ctrl_setting &= ~(BIT_I2C_MST_EN);
    }
    /* Configure power management 1 register if needed */
    if ((dev->conf.gyro_pwr == MPU9X50_SENSOR_PWR_OFF) && (dev->conf.accel_pwr == MPU9X50_SENSOR_PWR_OFF))
    {
        write_register(dev, MPU9X50_PWR_MGMT_1_REG, pwr_1_setting);
    }
    /* Configure mode writing by slave line 1 */
    write_register(dev, MPU9X50_SLAVE1_DATA_OUT_REG, s1_do_setting);
    /* Enable/disable I2C master mode */
    write_register(dev, MPU9X50_USER_CTRL_REG, usr_ctrl_setting);

    /* Release the bus */
    release_bus(dev);

    dev->conf.compass_pwr = pwr_conf;
    xtimer_usleep(MPU9X50_PWR_CHANGE_SLEEP_US);

    return 0;
}

int mpu9x50_read_gyro(const mpu9x50_t *dev, mpu9x50_results_t *output)
{
    uint8_t data[6];
    int16_t temp;
    float fsr;

    switch (dev->conf.gyro_fsr)
    {
    case MPU9X50_GYRO_FSR_250DPS:
        fsr = 250.0;
        break;
    case MPU9X50_GYRO_FSR_500DPS:
        fsr = 500.0;
        break;
    case MPU9X50_GYRO_FSR_1000DPS:
        fsr = 1000.0;
        break;
    case MPU9X50_GYRO_FSR_2000DPS:
        fsr = 2000.0;
        break;
    default:
        return -2;
    }

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }
    /* Read raw data */
    read_registers(dev, MPU9X50_GYRO_START_REG, data, 6);
    /* Release the bus */
    release_bus(dev);

    /* Normalize data according to configured full scale range */
    temp = (data[0] << 8) | data[1];
    output->x_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[2] << 8) | data[3];
    output->y_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[4] << 8) | data[5];
    output->z_axis = (temp * fsr) / MAX_VALUE;

    return 0;
}

void mpu9x50_enable_acc_fifo(const mpu9x50_t *dev)
{
    uint8_t fifo_enable_reg = 0 | BIT_FIFO_EN_ACC;
    write_register(dev, MPU9X50_FIFO_EN_REG, fifo_enable_reg);
    uint8_t usr_ctrl_reg = 0;
    read_register(dev, MPU9X50_USER_CTRL_REG, &usr_ctrl_reg, 0);
    usr_ctrl_reg |= BIT_USR_CTRL_FIFO_EN;
    write_register(dev, MPU9X50_USER_CTRL_REG, usr_ctrl_reg);
}

void mpu9x50_reset_fifo(const mpu9x50_t *dev)
{
    uint8_t usr_ctrl_reg = 0;
    read_register(dev, MPU9X50_USER_CTRL_REG, &usr_ctrl_reg, 0);
    usr_ctrl_reg |= BIT_USR_CTRL_FIFO_RST;
    write_register(dev, MPU9X50_USER_CTRL_REG, usr_ctrl_reg);
}

int mpu9x50_read_accel_from_fifo(const mpu9x50_t *dev, mpu9x50_results_t *buffer, uint16_t len)
{
    uint8_t *rawBuffer = (uint8_t *)buffer;
    uint16_t fifoCnt;
    read_registers(dev, MPU9X50_FIFO_COUNT_H_REG, (uint8_t *)&fifoCnt, 2);
    uint16_t available = (fifoCnt >> 8) | ((fifoCnt & 0x000F) << 8);

    uint16_t lenInBytes = len * sizeof(mpu9x50_results_t);
    uint16_t bytesToRead = (available > lenInBytes) ? lenInBytes : available;
    //printf("A=%d, LiB=%d, btr=%d\n", available, lenInBytes, bytesToRead);
    read_registers(dev, MPU9X50_FIFO_RW_REG, rawBuffer, bytesToRead);

    for (uint16_t i = 0; i < bytesToRead / sizeof(mpu9x50_results_t); i++)
    {
        uint16_t offset = sizeof(mpu9x50_results_t) * i;
        mpu9x50_results_t *results = (mpu9x50_results_t *)(rawBuffer + offset);
        normalize_accel_sample(dev, rawBuffer + offset, results);
    }
    return bytesToRead / sizeof(mpu9x50_results_t);
}

void normalize_accel_sample(const mpu9x50_t *dev, uint8_t *rawBytes, mpu9x50_results_t *result)
{
    float fsr = getFullRange(dev);
    int16_t tempX;
    int16_t tempY;
    int16_t tempZ;
    tempX = (rawBytes[0] << 8) | rawBytes[1];
    tempY = (rawBytes[2] << 8) | rawBytes[3];
    tempZ = (rawBytes[4] << 8) | rawBytes[5];
    result->x_axis = (tempX * fsr) / MAX_VALUE;
    result->y_axis = (tempY * fsr) / MAX_VALUE;
    result->z_axis = (tempZ * fsr) / MAX_VALUE;
}

int mpu9x50_read_accel(const mpu9x50_t *dev, mpu9x50_results_t *output)
{
    uint8_t data[6];

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }
    /* Read raw data */
    read_registers(dev, MPU9X50_ACCEL_START_REG, data, 6);
    /* Release the bus */
    release_bus(dev);

    normalize_accel_sample(dev, data, output);
    return 0;
}

int mpu9x50_read_compass(const mpu9x50_t *dev, mpu9x50_results_t *output)
{
    uint8_t data[6];

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }
    /* Read raw data */
    read_registers(dev, MPU9X50_EXT_SENS_DATA_START_REG, data, 6);
    /* Release the bus */
    release_bus(dev);

    output->x_axis = (data[1] << 8) | data[0];
    output->y_axis = (data[3] << 8) | data[2];
    output->z_axis = (data[5] << 8) | data[4];

    /* Compute sensitivity adjustment */
    output->x_axis = (int16_t)(((float)output->x_axis) *
                               ((((dev->conf.compass_x_adj - 128) * 0.5) / 128.0) + 1));
    output->y_axis = (int16_t)(((float)output->y_axis) *
                               ((((dev->conf.compass_y_adj - 128) * 0.5) / 128.0) + 1));
    output->z_axis = (int16_t)(((float)output->z_axis) *
                               ((((dev->conf.compass_z_adj - 128) * 0.5) / 128.0) + 1));

    /* Normalize data according to full-scale range */
    output->x_axis = output->x_axis * 0.3;
    output->y_axis = output->y_axis * 0.3;
    output->z_axis = output->z_axis * 0.3;

    return 0;
}

int mpu9x50_read_temperature(const mpu9x50_t *dev, int32_t *output)
{
    uint16_t data;

    /* Acquire exclusive access */
    if (acquire_bus(dev))
    {
        return -1;
    }
    /* Read raw temperature value */
    read_registers(dev, MPU9X50_TEMP_START_REG, (void *)&data, 2);
    /* Release the bus */
    release_bus(dev);

    data = htons(data);

    *output = (((int32_t)data * 1000LU) / MPU9X50_TEMP_SENSITIVITY) + (MPU9X50_TEMP_OFFSET * 1000LU);

    return 0;
}

int mpu9x50_set_gyro_fsr(mpu9x50_t *dev, mpu9x50_gyro_ranges_t fsr)
{
    if (dev->conf.gyro_fsr == fsr)
    {
        return 0;
    }

    switch (fsr)
    {
    case MPU9X50_GYRO_FSR_250DPS:
    case MPU9X50_GYRO_FSR_500DPS:
    case MPU9X50_GYRO_FSR_1000DPS:
    case MPU9X50_GYRO_FSR_2000DPS:
        if (acquire_bus(dev))
        {
            return -1;
        }
        write_register(dev, MPU9X50_GYRO_CFG_REG, (fsr << 3));
        release_bus(dev);
        dev->conf.gyro_fsr = fsr;
        break;
    default:
        return -2;
    }

    return 0;
}

int mpu9x50_set_accel_fsr(mpu9x50_t *dev, mpu9x50_accel_ranges_t fsr)
{
    if (dev->conf.accel_fsr == fsr)
    {
        return 0;
    }

    switch (fsr)
    {
    case MPU9X50_ACCEL_FSR_2G:
    case MPU9X50_ACCEL_FSR_4G:
    case MPU9X50_ACCEL_FSR_8G:
    case MPU9X50_ACCEL_FSR_16G:
        if (acquire_bus(dev))
        {
            return -1;
        }
        write_register(dev, MPU9X50_ACCEL_CFG_REG, (fsr << 3));
        release_bus(dev);
        dev->conf.accel_fsr = fsr;
        break;
    default:
        return -2;
    }

    return 0;
}

int mpu9x50_set_sample_rate(mpu9x50_t *dev, uint16_t rate)
{
    uint8_t divider;

    if ((rate < MPU9X50_MIN_SAMPLE_RATE) || (rate > MPU9X50_MAX_SAMPLE_RATE))
    {
        return -2;
    }
    else if (dev->conf.sample_rate == rate)
    {
        return 0;
    }

    /* Compute divider to achieve desired sample rate and write to rate div register */
    divider = (1000 / rate - 1);

    if (acquire_bus(dev))
    {
        return -1;
    }
    write_register(dev, MPU9X50_RATE_DIV_REG, divider);

    uint8_t debug = 0;
    read_register(dev, MPU9X50_RATE_DIV_REG, &debug, 0);
    printf("Divider: expected = %d, real=%d\n", divider, debug);

    /* Store configured sample rate */
    dev->conf.sample_rate = 1000 / (((uint16_t)divider) + 1);

    /* Always set LPF to a maximum of half the configured sampling rate */
    conf_lpf(dev, (dev->conf.sample_rate >> 1));
    release_bus(dev);

    return 0;
}

int mpu9x50_set_compass_sample_rate(mpu9x50_t *dev, uint8_t rate)
{
    uint8_t divider;

    if ((rate < MPU9X50_MIN_COMP_SMPL_RATE) || (rate > MPU9X50_MAX_COMP_SMPL_RATE) || (rate > dev->conf.sample_rate))
    {
        return -2;
    }
    else if (dev->conf.compass_sample_rate == rate)
    {
        return 0;
    }

    /* Compute divider to achieve desired sample rate and write to slave ctrl register */
    divider = (dev->conf.sample_rate / rate - 1);

    if (acquire_bus(dev))
    {
        return -1;
    }
    write_register(dev, MPU9X50_SLAVE4_CTRL_REG, divider);
    release_bus(dev);

    /* Store configured sample rate */
    dev->conf.compass_sample_rate = dev->conf.sample_rate / (((uint16_t)divider) + 1);

    return 0;
}

void mpu9x50_enable_fifo_overflow_interrupt(const mpu9x50_t *dev)
{
    /* Enable the overflow Interrupt */
    uint8_t reg = BIT_INT_EN_FIFO_OVERFLOW;
    read_register(dev, MPU9X50_INT_ENABLE_REG, &reg, 0);
    reg |= BIT_INT_EN_FIFO_OVERFLOW;
    write_register(dev, MPU9X50_INT_ENABLE_REG, reg);

    /*read status to clear interrupt */
    read_register(dev, MPU9X50_INT_STATUS, &reg, 0);
}

void mpu9x50_enable_motion_interrupt(const mpu9x50_t *dev, uint16_t threshold, uint16_t frequency)
{
    /* Enable the wake-up Interrupt */
    uint8_t reg;
    read_register(dev, MPU9X50_INT_ENABLE_REG, &reg, 0);
    reg |= BIT_INT_EN_WOM_EN;
    write_register(dev, MPU9X50_INT_ENABLE_REG, reg);

    /* Enable Accelerometer Intelligence */
    reg = BIT_ACCEL_INTEL_EN | BIT_ACCEL_INTEL_MODE;
    write_register(dev, MPU9X50_ACCEL_INTEL_CTRL, reg);

    /* Set threshold (LSB = 4mg) */
    write_register(dev, MPU9X50_WOM_THRESHOLD, threshold >> 2);

    /* Set Wakeup Frequency */
    write_register(dev, MPU9X50_LP_ACCEL_ODR, frequency);

    /* Enable cycle mode */
    read_register(dev, MPU9X50_PWR_MGMT_1_REG, &reg, 0);
    reg |= BIT_PWR_MGMT1_CYCLE;
    write_register(dev, MPU9X50_PWR_MGMT_1_REG, reg);

    /*read status to clear interrupt */
    read_register(dev, MPU9X50_INT_STATUS, &reg, 0);
}

void mpu9x50_disable_motion_interrupt(const mpu9x50_t *dev)
{
    /* Enable the wake-up Interrupt */
    uint8_t reg = BIT_INT_EN_FIFO_OVERFLOW;
    read_register(dev, MPU9X50_INT_ENABLE_REG, &reg, 0);
    reg &= ~BIT_INT_EN_WOM_EN;
    write_register(dev, MPU9X50_INT_ENABLE_REG, reg);
}

void mpu9x50_disable_fifo_overflow_interrupt(const mpu9x50_t *dev)
{
    /* Disable ONLY the overflow Interrupt */
    uint8_t reg = BIT_INT_EN_FIFO_OVERFLOW;
    read_register(dev, MPU9X50_INT_ENABLE_REG, &reg, 0);
    reg &= ~BIT_INT_EN_FIFO_OVERFLOW;
    write_register(dev, MPU9X50_INT_ENABLE_REG, reg);
}

bool mpu9x50_check_fifo_overflow(const mpu9x50_t *dev)
{
    if (gpio_read(dev->params.int_pin))
    {
        uint8_t status;
        read_register(dev, MPU9X50_INT_STATUS, &status, 0);
        return (status & BIT_INT_STATUS_FIFO_OVERFLOW) > 0;
    }
    return false;
}

bool mpu9x50_check_motion_interrupt(const mpu9x50_t *dev)
{
    if (gpio_read(dev->params.int_pin))
    {
        uint8_t status;
        read_register(dev, MPU9X50_INT_STATUS, &status, 0);
        return (status & BIT_INT_STATUS_WOM_INT) > 0;
    }
    return false;
}

/*------------------------------------------------------------------------------------*/
/*                                Internal functions                                  */
/*------------------------------------------------------------------------------------*/

/**
 * Initialize compass
 * Caution: This internal function does not acquire exclusive access to the I2C bus.
 *          Acquisation and release is supposed to be handled by the calling function.
 */
static int compass_init(mpu9x50_t *dev)
{
    uint8_t data[3];

    /* Enable Bypass Mode to speak to compass directly */
    conf_bypass(dev, 1);

    /* Check whether compass answers correctly */
    read_register(dev, COMPASS_WHOAMI_REG, data, 0);
    if (data[0] != MPU9X50_COMP_WHOAMI_ANSWER)
    {
        DEBUG("[Error] Wrong answer from compass\n");
        return -1;
    }

    /* Configure Power Down mode */
    write_register(dev, COMPASS_CNTL_REG, MPU9X50_COMP_POWER_DOWN);
    xtimer_usleep(MPU9X50_COMP_MODE_SLEEP_US);
    /* Configure Fuse ROM access */
    write_register(dev, COMPASS_CNTL_REG, MPU9X50_COMP_FUSE_ROM);
    xtimer_usleep(MPU9X50_COMP_MODE_SLEEP_US);
    /* Read sensitivity adjustment values from Fuse ROM */
    read_registers(dev, COMPASS_ASAX_REG, data, 3);
    dev->conf.compass_x_adj = data[0];
    dev->conf.compass_y_adj = data[1];
    dev->conf.compass_z_adj = data[2];
    /* Configure Power Down mode again */
    write_register(dev, COMPASS_CNTL_REG, MPU9X50_COMP_POWER_DOWN);
    xtimer_usleep(MPU9X50_COMP_MODE_SLEEP_US);

    /* Disable Bypass Mode to configure MPU as master to the compass */
    conf_bypass(dev, 0);

    /* Configure MPU9X50 for single master mode */
    write_register(dev, MPU9X50_I2C_MST_REG, BIT_WAIT_FOR_ES);

    /* Set up slave line 0 */
    /* Slave line 0 reads the compass data */
    write_register(dev, MPU9X50_SLAVE0_ADDR_REG, (BIT_SLAVE_RW | DEV_COMP_ADDR));
    /* Slave line 0 read starts at compass data register */
    write_register(dev, MPU9X50_SLAVE0_REG_REG, COMPASS_DATA_START_REG);
    /* Enable slave line 0 and configure read length to 6 consecutive registers */
    write_register(dev, MPU9X50_SLAVE0_CTRL_REG, (BIT_SLAVE_EN | 0x06));

    /* Set up slave line 1 */
    /* Slave line 1 writes to the compass */
    write_register(dev, MPU9X50_SLAVE1_ADDR_REG, DEV_COMP_ADDR);
    /* Slave line 1 write starts at compass control register */
    write_register(dev, MPU9X50_SLAVE1_REG_REG, COMPASS_CNTL_REG);
    /* Enable slave line 1 and configure write length to 1 register */
    write_register(dev, MPU9X50_SLAVE1_CTRL_REG, (BIT_SLAVE_EN | 0x01));
    /* Configure data which is written by slave line 1 to compass control */
    write_register(dev, MPU9X50_SLAVE1_DATA_OUT_REG, MPU9X50_COMP_SINGLE_MEASURE);

    /* Slave line 0 and 1 operate at each sample */
    write_register(dev, MPU9X50_I2C_DELAY_CTRL_REG, (BIT_SLV0_DELAY_EN | BIT_SLV1_DELAY_EN));
    /* Set I2C bus to VDD */
    write_register(dev, MPU9X50_YG_OFFS_TC_REG, BIT_I2C_MST_VDDIO);

    return 0;
}

/**
 * Configure bypass mode
 * Caution: This internal function does not acquire exclusive access to the I2C bus.
 *          Acquisation and release is supposed to be handled by the calling function.
 */
static void conf_bypass(const mpu9x50_t *dev, uint8_t bypass_enable)
{
    uint8_t data;
    read_register(dev, MPU9X50_USER_CTRL_REG, &data, 0);

    if (bypass_enable)
    {
        data &= ~(BIT_I2C_MST_EN);
        write_register(dev, MPU9X50_USER_CTRL_REG, data);
        xtimer_usleep(MPU9X50_BYPASS_SLEEP_US);
        write_register(dev, MPU9X50_INT_PIN_CFG_REG, BIT_I2C_BYPASS_EN);
    }
    else
    {
        data |= BIT_I2C_MST_EN;
        write_register(dev, MPU9X50_USER_CTRL_REG, data);
        xtimer_usleep(MPU9X50_BYPASS_SLEEP_US);
        write_register(dev, MPU9X50_INT_PIN_CFG_REG, REG_RESET);
    }
}

/**
 * Configure low pass filter
 * Caution: This internal function does not acquire exclusive access to the I2C bus.
 *          Acquisation and release is supposed to be handled by the calling function.
 */
static void conf_lpf(const mpu9x50_t *dev, uint16_t half_rate)
{
    mpu9x50_lpf_t lpf_setting;

    /* Get target LPF configuration setting */
    if (half_rate >= 188)
    {
        lpf_setting = MPU9X50_FILTER_188HZ;
    }
    else if (half_rate >= 98)
    {
        lpf_setting = MPU9X50_FILTER_98HZ;
    }
    else if (half_rate >= 42)
    {
        lpf_setting = MPU9X50_FILTER_42HZ;
    }
    else if (half_rate >= 20)
    {
        lpf_setting = MPU9X50_FILTER_20HZ;
    }
    else if (half_rate >= 10)
    {
        lpf_setting = MPU9X50_FILTER_10HZ;
    }
    else
    {
        lpf_setting = MPU9X50_FILTER_5HZ;
    }

    /* Write LPF setting to configuration register */
    write_register(dev, MPU9X50_LPF_REG, lpf_setting);
}

int acquire_bus(const mpu9x50_t *dev)
{
    if (dev->params.use_spi <= 0)
    {
        return i2c_acquire(dev->params.i2c);
    }
    return 0;
}

void release_bus(const mpu9x50_t *dev)
{
    if (dev->params.use_spi <= 0)
    {
        i2c_release(dev->params.i2c);
    }
}

void write_register(const mpu9x50_t *dev, uint8_t regAddress, uint8_t data)
{
    if (dev->params.use_spi > 0)
    {
        mpu9x50_params_t params = dev->params;
        regAddress &= ~SPI_READ_WRITE_MASK;
        uint8_t success = spi_acquire(params.spi, params.spi_cs, params.spi_mode, params.spi_clk);
        if (success != SPI_OK)
        {
            printf("SPI Error: %d\n", success);
        }
        else
        {
            spi_transfer_reg(params.spi, params.spi_cs, regAddress, data);
        }
        spi_release(dev->params.spi);
    }
    else
    {
        i2c_write_reg(dev->params.i2c, DEV_ADDR, regAddress, data, 0);
    }
}

void read_register(const mpu9x50_t *dev, uint8_t regAddress, uint8_t *target, uint8_t flags)
{
    if (dev->params.use_spi)
    {
        mpu9x50_params_t params = dev->params;
        regAddress |= SPI_READ_WRITE_MASK;
        uint8_t success = spi_acquire(params.spi, params.spi_cs, params.spi_mode, params.spi_clk);
        if (success != SPI_OK)
        {
            printf("SPI Error: %d\n", success);
        }
        else
        {
            *target = spi_transfer_reg(params.spi, params.spi_cs, regAddress, 0);
        }
        spi_release(dev->params.spi);
    }
    else
    {
        i2c_read_reg(dev->params.i2c, DEV_ADDR, regAddress, target, flags);
    }
}

void read_registers(const mpu9x50_t *dev, uint8_t regAddress, uint8_t *buffer, uint16_t length)
{
    if (dev->params.use_spi > 0)
    {
        mpu9x50_params_t params = dev->params;
        regAddress |= SPI_READ_WRITE_MASK;
        uint8_t success = spi_acquire(params.spi, params.spi_cs, params.spi_mode, params.spi_clk);
        if (success != SPI_OK)
        {
            printf("SPI Error: %d\n", success);
        }
        else
        {
            spi_transfer_regs(params.spi, params.spi_cs, regAddress, NULL, buffer, length);
        }
        spi_release(dev->params.spi);
    }
    else
    {
        i2c_read_regs(dev->params.i2c, DEV_ADDR, regAddress, buffer, length, 0);
    }
}

float getFullRange(const mpu9x50_t *dev)
{
    switch (dev->conf.accel_fsr)
    {
    case MPU9X50_ACCEL_FSR_2G:
        return 2000.0;
    case MPU9X50_ACCEL_FSR_4G:
        return 4000.0;
    case MPU9X50_ACCEL_FSR_8G:
        return 8000.0;
    case MPU9X50_ACCEL_FSR_16G:
        return 16000.0;
    default:
        return -2;
    }
}
