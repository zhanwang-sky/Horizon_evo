/**
  ******************************************************************************
  * @file   icm20648.c
  * @author Ji Chen
  * @brief  ICM20648 driver.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "icm20648.h"
#include "al_stm32l4xx.h"

/* Definitions ---------------------------------------------------------------*/
#define INV_ICM_I2C_ADDRESS     (0x68 << 1)

#define BANK_0                  (0 << 7)
#define BANK_1                  (1 << 7)
#define BANK_2                  (2 << 7)
#define BANK_3                  (3 << 7)

/* register and associated bit definition */
/* bank 0 register map */
#define REG_WHO_AM_I            (BANK_0 | 0x00)

#define REG_USER_CTRL           (BANK_0 | 0x03)
#define BIT_DMP_EN                      0x80
#define BIT_FIFO_EN                     0x40
#define BIT_I2C_MST_EN                  0x20
#define BIT_I2C_IF_DIS                  0x10
#define BIT_DMP_RST                     0x08
#define BIT_SRAM_RST                    0x04
#define BIT_I2C_MST_RST                 0X02

#define REG_LP_CONFIG           (BANK_0 | 0x05)
#define BIT_I2C_MST_CYCLE               0x40
#define BIT_ACCEL_CYCLE                 0x20
#define BIT_GYRO_CYCLE                  0x10

#define REG_PWR_MGMT_1          (BANK_0 | 0x06)
#define BIT_DEVICE_RESET                0x80
#define BIT_SLEEP                       0x40
#define BIT_LP_EN                       0x20
#define BIT_TEMP_DIS                    0x08
#define BITS_CLKSEL                     0x7f

#define REG_PWR_MGMT_2          (BANK_0 | 0x07)
#define SHIFT_DISABLE_ACCEL             3
#define BITS_DISABLE_ACCEL              0x38
#define BITS_DISABLE_GYRO               0x07

#define REG_INT_PIN_CFG         (BANK_0 | 0x0F)
#define BIT_INT1_ACTL                   0x80
#define BIT_INT1_OPEN                   0x40
#define BIT_INT1_LATCH_INT_EN           0x20
#define BIT_INT_ANYRD_2CLEAR            0x10
#define BIT_ACTL_FSYNC                  0x08
#define BIT_FSYNC_INT_MODE_EN           0x04
#define BIT_BYPASS_EN                   0x02

#define REG_INT_ENABLE          (BANK_0 | 0x10)
#define BIT_REG_WOF_EN                  0x80
#define BIT_WOM_INT_EN                  0x08
#define BIT_PLL_RDY_EN                  0X04
#define BIT_DMP_INT1_EN                 0x02
#define BIT_I2C_MST_INT_EN              0x01

#define REG_INT_ENABLE_1        (BANK_0 | 0x11)
#define BIT_RAW_DATA_0_RDY_EN           0x01

#define REG_I2C_MST_STATUS      (BANK_0 | 0x17)
#define BIT_PASS_THROUGH                0x80
#define BIT_I2C_SLV4_DONE               0x40
#define BIT_I2C_LOST_ARB                0x20
#define BIT_I2C_SLV4_NACK               0x10
#define BIT_I2C_SLV3_NACK               0x08
#define BIT_I2C_SLV2_NACK               0x04
#define BIT_I2C_SLV1_NACK               0x02
#define BIT_I2C_SLV0_NACK               0x01

#define REG_INT_STATUS          (BANK_0 | 0x19)
#define BIT_WOM_INT                     0x08
#define BIT_PLL_RDY_INT                 0x04
#define BIT_DMP_INT1                    0x02
#define BIT_I2C_MST_INT                 0x01

#define REG_INT_STATUS_1        (BANK_0 | 0x1a)
#define BIT_RAW_DATA_0_RDY_INT          0x01

#define REG_DELAY_TIMEH         (BANK_0 | 0x28)
#define REG_DELAY_TIMEL         (BANK_0 | 0x29)

#define REG_ACCEL_XOUT_H        (BANK_0 | 0x2d)
#define REG_ACCEL_XOUT_L        (BANK_0 | 0x2e)
#define REG_ACCEL_YOUT_H        (BANK_0 | 0x2f)
#define REG_ACCEL_YOUT_L        (BANK_0 | 0x30)
#define REG_ACCEL_ZOUT_H        (BANK_0 | 0x31)
#define REG_ACCEL_ZOUT_L        (BANK_0 | 0x32)
#define REG_GYRO_XOUT_H         (BANK_0 | 0x33)
#define REG_GYRO_XOUT_L         (BANK_0 | 0x34)
#define REG_GYRO_YOUT_H         (BANK_0 | 0x35)
#define REG_GYRO_YOUT_L         (BANK_0 | 0x36)
#define REG_GYRO_ZOUT_H         (BANK_0 | 0x37)
#define REG_GYRO_ZOUT_L         (BANK_0 | 0x38)
#define REG_TEMP_OUT_H          (BANK_0 | 0x39)
#define REG_TEMP_OUT_L          (BANK_0 | 0x3a)

/* bank 2 register map */
#define REG_GYRO_SMPLRT_DIV     (BANK_2 | 0x00)

#define REG_GYRO_CONFIG_1       (BANK_2 | 0x01)
#define SHIFT_GYRO_DLPFCFG              3
#define BITS_GYRO_DLPFCFG               0x38
#define SHIFT_GYRO_FS_SEL               1
#define BITS_GYRO_FS_SEL                0x06
#define BIT_GYRO_FCHOICE                1

#define REG_GYRO_CONFIG_2       (BANK_2 | 0x02)
#define BIT_XGYRO_CTEN                  0x20
#define BIT_YGYRO_CTEN                  0x10
#define BIT_ZGYRO_CTEN                  0x08
#define BITS_GYRO_AVGCFG                0x07

#define REG_ODR_ALIGN_EN        (BANK_2 | 0x09)
#define BIT_ODR_ALIGN_EN                0x01

#define REG_ACCEL_SMPLRT_DIV_1  (BANK_2 | 0x10)
#define REG_ACCEL_SMPLRT_DIV_2  (BANK_2 | 0x11)

#define REG_ACCEL_CONFIG        (BANK_2 | 0x14)
#define SHIFT_ACCEL_DLPFCFG             3
#define BITS_ACCEL_DLPFCFG              0x38
#define SHIFT_ACCEL_FS_SEL              1
#define BITS_ACCEL_FS_SEL               0x06
#define BIT_ACCEL_FCHOICE               1

#define REG_ACCEL_CONFIG_2      (BANK_2 | 0x15)
#define BIT_AX_ST_EN_REG                0x10
#define BIT_AY_ST_EN_REG                0x08
#define BIT_AZ_ST_EN_REG                0x04
#define BITS_DEC3_CFG                   0x03

/* register for all banks */
#define REG_BANK_SEL            0x7F

#define INV_MAX_SERIAL_READ     16
#define INV_MAX_SERIAL_WRITE    16

/* Macros --------------------------------------------------------------------*/
#ifndef min
#define min(x,y)    (((x) < (y)) ? (x) : (y))
#endif

#ifndef max
#define max(x,y)    (((x) > (y)) ? (x) : (y))
#endif

/* Variables -----------------------------------------------------------------*/
static int g_inv_icm20648_i2c_fd = 0;

/* Functions -----------------------------------------------------------------*/
static int inv_set_bank(unsigned char bank) {
    unsigned char data = (unsigned char) (bank << 4);

    return al_i2c_write(g_inv_icm20648_i2c_fd, INV_ICM_I2C_ADDRESS, REG_BANK_SEL, &data, 1);
}

int
inv_icm20648_write_single_mems_reg(unsigned short reg, const unsigned char data) {
    unsigned char regOnly = (unsigned char) (reg & 0x7F);
    int rc;

    rc = inv_set_bank(reg >> 7);
    if (rc < 0) {
        goto EXIT;
    }

    rc = al_i2c_write(g_inv_icm20648_i2c_fd, INV_ICM_I2C_ADDRESS, regOnly, &data, 1);

EXIT:
    return rc;
}

int
inv_icm20648_write_mems_reg(unsigned short reg, unsigned int length, const unsigned char *data) {
    unsigned char regOnly = (unsigned char) (reg & 0x7F);
    unsigned int bytesWritten;
    unsigned int burstLen;
    int rc;

    rc = inv_set_bank(reg >> 7);
    if (rc < 0) {
        goto EXIT;
    }

    bytesWritten = 0;
    while (bytesWritten < length) {
        burstLen = min(INV_MAX_SERIAL_WRITE, length - bytesWritten);
        rc = al_i2c_write(g_inv_icm20648_i2c_fd, INV_ICM_I2C_ADDRESS, regOnly, data + bytesWritten, burstLen);
        if (rc < 0) {
            goto EXIT;
        }
        bytesWritten += burstLen;
    }

EXIT:
    return rc;
}

int
inv_icm20648_read_single_mems_reg(unsigned short reg, unsigned char *data) {
    unsigned char regOnly = (unsigned char) (reg & 0x7F);
    int rc;

    rc = inv_set_bank(reg >> 7);
    if (rc < 0) {
        goto EXIT;
    }

    rc = al_i2c_read(g_inv_icm20648_i2c_fd, INV_ICM_I2C_ADDRESS, regOnly, data, 1);

EXIT:
    return rc;
}

int
inv_icm20648_read_mems_reg(unsigned short reg, unsigned int length, unsigned char *data) {
    unsigned char regOnly = (unsigned char) (reg & 0x7F);
    unsigned int bytesRead;
    unsigned int burstLen;
    int rc;

    rc = inv_set_bank(reg >> 7);
    if (rc < 0) {
        goto EXIT;
    }

    bytesRead = 0;
    while (bytesRead < length) {
        burstLen = min(INV_MAX_SERIAL_READ, length - bytesRead);
        rc = al_i2c_read(g_inv_icm20648_i2c_fd, INV_ICM_I2C_ADDRESS, regOnly, data + bytesRead, burstLen);
        if (rc < 0) {
            goto EXIT;
        }
        bytesRead += burstLen;
    }

EXIT:
    return rc;
}

// User functions
int inv_icm_soft_reset(void) {
    return inv_icm20648_write_single_mems_reg(REG_PWR_MGMT_1, BIT_DEVICE_RESET);
}

int inv_icm_wakeup(void) {
    unsigned char data;
    int rc;

    rc = inv_icm20648_read_single_mems_reg(REG_PWR_MGMT_1, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data &= ~BIT_SLEEP;
    rc = inv_icm20648_write_single_mems_reg(REG_PWR_MGMT_1, data);

EXIT:
    return rc;
}

int inv_icm_enable_odr_align(void) {
    unsigned char data;
    int rc;

    rc = inv_icm20648_read_single_mems_reg(REG_ODR_ALIGN_EN, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data |= BIT_ODR_ALIGN_EN;
    rc = inv_icm20648_write_single_mems_reg(REG_ODR_ALIGN_EN, data);

EXIT:
    return rc;
}

int inv_icm_set_gyro_smplrt(int rate) {
    unsigned char div, cfg;
    int rc;

    /* sanity check */
    if (rate < 5 || rate > 1125) {
        return -1;
    }

    /* set sample rate */
    div = (1125 / rate) - 1;
    rc = inv_icm20648_write_single_mems_reg(REG_GYRO_SMPLRT_DIV, div);
    if (rc < 0) {
        goto EXIT;
    }

    /* set low pass filter */
    rc = inv_icm20648_read_single_mems_reg(REG_GYRO_CONFIG_1, &cfg);
    if (rc < 0) {
        goto EXIT;
    }
    cfg &= ~BITS_GYRO_DLPFCFG;
    if (div == 0) {
        // 1125 Hz
        cfg |= 7 << SHIFT_GYRO_DLPFCFG; // 361.4
    } else if (div == 1) {
        // 562.5 Hz
        cfg |= 0; // 196.6
    } else if (div == 2) {
        // 375 Hz
        cfg |= 1 << SHIFT_GYRO_DLPFCFG; // 151.8
    } else if (div == 3) {
        // 281.25 Hz
        cfg |= 2 << SHIFT_GYRO_DLPFCFG; // 119.5
    } else if (div <= 10) {
        // 225 ~ 102.3 Hz
        cfg |= 3 << SHIFT_GYRO_DLPFCFG; // 51.2
    } else if (div <= 22) {
        // 93.75 ~ 48.9 Hz
        cfg |= 4 << SHIFT_GYRO_DLPFCFG; // 23.9
    } else if (div <= 47) {
        // 46.875 ~ 23.4 Hz
        cfg |= 5 << SHIFT_GYRO_DLPFCFG; // 11.6
    } else {
        // < 23.0 Hz
        cfg |= 6 << SHIFT_GYRO_DLPFCFG; // 5.7
    }
    cfg |= BIT_GYRO_FCHOICE;
    rc = inv_icm20648_write_single_mems_reg(REG_GYRO_CONFIG_1, cfg);

EXIT:
    return rc;
}

int inv_icm_set_gfs(mpu_gyro_fs_t gfs) {
    unsigned char data;
    int rc;

    /* sanity check */
    if (gfs >= NUM_MPU_GFS) {
        return -1;
    }

    rc = inv_icm20648_read_single_mems_reg(REG_GYRO_CONFIG_1, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data &= ~BITS_GYRO_FS_SEL;
    data |= gfs << SHIFT_GYRO_FS_SEL;
    rc = inv_icm20648_write_single_mems_reg(REG_GYRO_CONFIG_1, data);

EXIT:
    return rc;
}

int inv_icm_set_accel_smplrt(int rate) {
    unsigned short div;
    unsigned char data;
    int rc;

    /* sanity check */
    if (rate < 1 || rate > 1125) {
        return -1;
    }

    /* set sample rate */
    div = (1125 / rate) - 1;
    data = div >> 8;
    data &= 0x0f;
    rc = inv_icm20648_write_single_mems_reg(REG_ACCEL_SMPLRT_DIV_1, data);
    if (rc < 0) {
        goto EXIT;
    }
    data = div & 0xff;
    rc = inv_icm20648_write_single_mems_reg(REG_ACCEL_SMPLRT_DIV_2, data);
    if (rc < 0) {
        goto EXIT;
    }

    /* set LPF */
    rc = inv_icm20648_read_single_mems_reg(REG_ACCEL_CONFIG, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data &= ~BITS_ACCEL_DLPFCFG;
    if (div == 0) {
        // 1125 Hz
        data |= 7 << SHIFT_ACCEL_DLPFCFG; // 473
    } else if (div == 1) {
        // 562.5 Hz
        data |= 0; // 246
    } else if (div <= 4) {
        // 375 ~ 225 Hz
        data |= 2 << SHIFT_ACCEL_DLPFCFG; // 111.4
    } else if (div <= 10) {
        // 187.5 ~ 102.3 Hz
        data |= 3 << SHIFT_ACCEL_DLPFCFG; // 50.4
    } else if (div <= 22) {
        // 93.75 ~ 48.9 Hz
        data |= 4 << SHIFT_ACCEL_DLPFCFG; // 23.9
    } else if (div <= 47) {
        // 46.875 ~ 23.4 Hz
        data |= 5 << SHIFT_ACCEL_DLPFCFG; // 11.5
    } else {
        // < 23.0 Hz
        data |= 6 << SHIFT_ACCEL_DLPFCFG; // 5.7
    }
    data |= BIT_ACCEL_FCHOICE;
    rc = inv_icm20648_write_single_mems_reg(REG_ACCEL_CONFIG, data);

EXIT:
    return rc;
}

int inv_icm_set_afs(mpu_accel_fs_t afs) {
    unsigned char data;
    int rc;

    /* sanity check */
    if (afs >= NUM_MPU_AFS) {
        return -1;
    }

    rc = inv_icm20648_read_single_mems_reg(REG_ACCEL_CONFIG, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data &= ~BITS_ACCEL_FS_SEL;
    data |= afs << SHIFT_ACCEL_FS_SEL;
    rc = inv_icm20648_write_single_mems_reg(REG_ACCEL_CONFIG, data);

EXIT:
    return rc;
}

int inv_icm_set_lp_mode(mpu_lp_mode_t lp_mode) {
    unsigned char data;
    int rc;

    /* sanity check */
    if (lp_mode >= NUM_MPU_LP) {
        return -1;
    }

    rc = inv_icm20648_read_single_mems_reg(REG_LP_CONFIG, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data |= BIT_I2C_MST_CYCLE;
    if (lp_mode == MPU_LP_DUTY_CYCLE) {
        data |= BIT_ACCEL_CYCLE | BIT_GYRO_CYCLE;
    } else {
        data &= ~(BIT_ACCEL_CYCLE | BIT_GYRO_CYCLE);
    }
    rc = inv_icm20648_write_single_mems_reg(REG_LP_CONFIG, data);

EXIT:
    return rc;
}

int inv_icm_enable_int(void) {
    unsigned char data;
    int rc;

    /* enable raw data interrupt */
    rc = inv_icm20648_read_single_mems_reg(REG_INT_ENABLE_1, &data);
    if (rc < 0) {
        goto EXIT;
    }
    data |= BIT_RAW_DATA_0_RDY_EN;
    rc = inv_icm20648_write_single_mems_reg(REG_INT_ENABLE_1, data);

EXIT:
    return rc;
}

/******************************** END OF FILE *********************************/
