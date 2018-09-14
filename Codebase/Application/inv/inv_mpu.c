/**
  ******************************************************************************
  * @file    inv_mpu.c
  * @author  Ji Chen
  * @brief   MPUxxxx sensor dirver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef HORIZON_MINI_L4
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

#include "main.h"

/* Private define ------------------------------------------------------------*/

/* MPU device address */
#define INV_MPU_DEV_ADDR              (0xD0)

/* MPU registers */
#define INV_MPU_REG_SELF_TEST_X_GYRO  (0x00)
#define INV_MPU_REG_SELF_TEST_Y_GYRO  (0x01)
#define INV_MPU_REG_SELF_TEST_Z_GYRO  (0x02)

#define INV_MPU_REG_SELF_TEST_X_ACCEL (0x0D)
#define INV_MPU_REG_SELF_TEST_Y_ACCEL (0x0E)
#define INV_MPU_REG_SELF_TEST_Z_ACCEL (0x0F)

#define INV_MPU_REG_XG_OFFSET_H       (0x13)
#define INV_MPU_REG_XG_OFFSET_L       (0x14)
#define INV_MPU_REG_YG_OFFSET_H       (0x15)
#define INV_MPU_REG_YG_OFFSET_L       (0x16)
#define INV_MPU_REG_ZG_OFFSET_H       (0x17)
#define INV_MPU_REG_ZG_OFFSET_L       (0x18)
#define INV_MPU_REG_SMPLRT_DIV        (0x19)
#define INV_MPU_REG_CONFIG            (0x1A)
#define INV_MPU_REG_GYRO_CONFIG       (0x1B)
#define INV_MPU_REG_ACCEL_CONFIG      (0x1C)
#define INV_MPU_REG_ACCEL_CONFIG2     (0x1D)
#define INV_MPU_REG_LP_ACCEL_ODR      (0x1E)
#define INV_MPU_REG_WOM_THR           (0x1F)

#define INV_MPU_REG_FIFO_EN           (0x23)

#define INV_MPU_REG_INT_PIN_CFG       (0x37)
#define INV_MPU_REG_INT_ENABLE        (0x38)

#define INV_MPU_REG_INT_STATUS        (0x3A)
#define INV_MPU_REG_ACCEL_XOUT_H      (0x3B)
#define INV_MPU_REG_ACCEL_XOUT_L      (0x3C)
#define INV_MPU_REG_ACCEL_YOUT_H      (0x3D)
#define INV_MPU_REG_ACCEL_YOUT_L      (0x3E)
#define INV_MPU_REG_ACCEL_ZOUT_H      (0x3F)
#define INV_MPU_REG_ACCEL_ZOUT_L      (0x40)
#define INV_MPU_REG_TEMP_OUT_H        (0x41)
#define INV_MPU_REG_TEMP_OUT_L        (0x42)
#define INV_MPU_REG_GYRO_XOUT_H       (0x43)
#define INV_MPU_REG_GYRO_XOUT_L       (0x44)
#define INV_MPU_REG_GYRO_YOUT_H       (0x45)
#define INV_MPU_REG_GYRO_YOUT_L       (0x46)
#define INV_MPU_REG_GYRO_ZOUT_H       (0x47)
#define INV_MPU_REG_GYRO_ZOUT_L       (0x48)

#define INV_MPU_REG_SIGNAL_PATH_RESET (0x68)
#define INV_MPU_REG_MOT_DETECT_CTRL   (0x69)
#define INV_MPU_REG_USER_CTRL         (0x6A)
#define INV_MPU_REG_PWR_MGMT_1        (0x6B)
#define INV_MPU_REG_PWR_MGMT_2        (0x6C)

#define INV_MPU_REG_FIFO_COUNTH       (0x72)
#define INV_MPU_REG_FIFO_COUNTL       (0x73)
#define INV_MPU_REG_FIFO_R_W          (0x74)
#define INV_MPU_REG_WHO_AM_I          (0x75)

#define INV_MPU_REG_XA_OFFSET_H       (0x77)
#define INV_MPU_REG_XA_OFFSET_L       (0x78)
#define INV_MPU_REG_YA_OFFSET_H       (0x7A)
#define INV_MPU_REG_YA_OFFSET_L       (0x7B)
#define INV_MPU_REG_ZA_OFFSET_H       (0x7D)
#define INV_MPU_REG_ZA_OFFSET_L       (0x7E)

/* bits defined in CONFIG */
#define INV_MPU_BIT_FIFO_MODE         (0x40)
#define INV_MPU_MASK_EXT_SYNC_SET     (0x38)
#define INV_MPU_MASK_DLPF_CFG         (0x07)

/* bits defined in GYRO_CONFIG */
#define INV_MPU_BIT_XGYRO_Cten        (0x80)
#define INV_MPU_BIT_YGYRO_Cten        (0x40)
#define INV_MPU_BIT_ZGYRO_Cten        (0x20)
#define INV_MPU_MASK_GYRO_FS_SEL      (0x18)
#define INV_MPU_MASK_Fchoice_b        (0x03)

/* bits defined in ACCEL_CONFIG */
#define INV_MPU_BIT_ax_st_en          (0x80)
#define INV_MPU_BIT_ay_st_en          (0x40)
#define INV_MPU_BIT_az_st_en          (0x20)
#define INV_MPU_MASK_ACCEL_FS_SEL     (0x18)

/* bits defined in ACCEL_CONFIG2 */
#define INV_MPU_BIT_accel_fchoice_b   (0x08)
#define INV_MPU_MASK_A_DLPFCFG        (0x07)

/* bits defined in INT_PIN_CFG */
#define INV_MPU_BIT_INT_ANYRD_2CLEAR  (0x10)
#define INV_MPU_BIT_BYPASS_EN         (0x02)

/* bits defined in INT_ENABLE */
#define INV_MPU_BIT_RAW_RDY_EN        (0x01)

/* bits defined in USER_CTRL */
#define INV_MPU_BIT_FIFO_EN           (0x40)
#define INV_MPU_BIT_I2C_MST_EN        (0x20)
#define INV_MPU_BIT_I2C_IF_DIS        (0x10)
#define INV_MPU_BIT_FIFO_RST          (0x04)
#define INV_MPU_BIT_I2C_MST_RST       (0x02)
#define INV_MPU_BIT_SIG_COND_RST      (0X01)

/* bits defined in PWR_MGMT_1 */
#define INV_MPU_BIT_H_RESET           (0x80)
#define INV_MPU_BIT_SLEEP             (0x40)
#define INV_MPU_BIT_CYCLE             (0x20)
#define INV_MPU_BIT_GYRO_STANDBY      (0x10)
#define INV_MPU_BIT_PD_PTAT           (0x08)
#define INV_MPU_MASK_CLKSEL           (0x07)

/* bits defined in PWR_MGMT_2 */
#define INV_MPU_BIT_DISABLE_XA        (0x20)
#define INV_MPU_BIT_DISABLE_YA        (0x10)
#define INV_MPU_BIT_DISABLE_ZA        (0x08)
#define INV_MPU_BIT_DISABLE_XG        (0x04)
#define INV_MPU_BIT_DISABLE_YG        (0x02)
#define INV_MPU_BIT_DISABLE_ZG        (0x01)

/******************************************************************************/

/* Gyro full scale ranges. */
#define INV_MPU_GYRO_FSR_250DPS       (0)
#define INV_MPU_GYRO_FSR_500DPS       (1)
#define INV_MPU_GYRO_FSR_1000DPS      (2)
#define INV_MPU_GYRO_FSR_2000DPS      (3)

/* Accel full scale ranges. */
#define INV_MPU_ACCEL_FSR_2G          (0)
#define INV_MPU_ACCEL_FSR_4G          (1)
#define INV_MPU_ACCEL_FSR_8G          (2)
#define INV_MPU_ACCEL_FSR_16G         (3)

/* Filter configurations. */
#define INV_MPU_LPF_184HZ             (1)
#define INV_MPU_LPF_92HZ              (2)
#define INV_MPU_LPF_41HZ              (3)
#define INV_MPU_LPF_20HZ              (4)
#define INV_MPU_LPF_10HZ              (5)
#define INV_MPU_LPF_5HZ               (6)

/* Private variables ---------------------------------------------------------*/
static float inv_mpu_gyroScale = 0.00762939453125f;
static float inv_mpu_accelScale = 0.00006103515625f;

/* Functions -----------------------------------------------------------------*/
int mpu_read_data(uint8_t rawData[14], float data[6]) {
    if (NULL == rawData || NULL == data) {
        return -1;
    }

    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_ACCEL_XOUT_H, rawData, 14);

    for (int i = 0; i < 3; i++) {
        data[i] = (float) ((int16_t) (rawData[2 * (i + 4)] << 8 | rawData[(2 * (i + 4)) + 1]))
                  * inv_mpu_gyroScale;
    }
    for (int i = 3; i < 6; i++) {
        data[i] = (float) ((int16_t) (rawData[2 * (i - 3)] << 8 | rawData[(2 * (i - 3)) + 1]))
                  * inv_mpu_accelScale;
    }

    return 0;
}

int mpu_set_int(int enable) {
    uint8_t data[2];

    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_INT_PIN_CFG, &data[0], 1);
    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_INT_ENABLE, &data[1], 1);
    if (enable != 0) {
        data[0] |= INV_MPU_BIT_INT_ANYRD_2CLEAR;
        data[1] |= INV_MPU_BIT_RAW_RDY_EN;
    } else {
        data[0] &= ~INV_MPU_BIT_INT_ANYRD_2CLEAR;
        data[1] &= ~INV_MPU_BIT_RAW_RDY_EN;
    }
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_INT_PIN_CFG, &data[0], 1);
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_INT_ENABLE, &data[1], 1);

    return 0;
}

/**
  * @brief  Set sampling rate.
  *         Sampling rate must be between 4Hz and 1kHz.
  * @param  rate Desired sampling rate (Hz).
  * @retval 0 if successful.
  */
int mpu_set_sample_rate(int rate) {
    uint8_t data;

    if (rate < 4)
        rate = 4;
    else if (rate > 1000)
        rate = 1000;

    data = 1000 / rate - 1;

    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_SMPLRT_DIV, &data, 1);

    return 0;
}

/**
  * @brief  Set digital low pass filter.
  *         The following LPF settings are supported: 184, 92, 41, 20, 10, 5.
  * @param  lpf Desired LPF setting.
  * @retval 0 if successful.
  */
int mpu_set_lpf(int lpf) {
    uint8_t cfg_bits;
    uint8_t data;

    if (lpf >= 184) {
        cfg_bits = INV_MPU_LPF_184HZ;
    } else if (lpf >= 92) {
        cfg_bits = INV_MPU_LPF_92HZ;
    } else if (lpf >= 41) {
        cfg_bits = INV_MPU_LPF_41HZ;
    } else if (lpf >= 20) {
        cfg_bits = INV_MPU_LPF_20HZ;
    } else if (lpf >= 10) {
        cfg_bits = INV_MPU_LPF_10HZ;
    } else {
        cfg_bits = INV_MPU_LPF_5HZ;
    }

    /* GYRO */
    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_CONFIG, &data, 1);
    data &= ~INV_MPU_MASK_DLPF_CFG;
    data |= cfg_bits;
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_CONFIG, &data, 1);
    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_GYRO_CONFIG, &data, 1);
    data &= ~INV_MPU_MASK_Fchoice_b;
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_GYRO_CONFIG, &data, 1);

    /* ACCEL */
    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_ACCEL_CONFIG2, &data, 1);
    data &= ~INV_MPU_MASK_A_DLPFCFG;
    data &= ~INV_MPU_BIT_accel_fchoice_b;
    data |= cfg_bits;
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_ACCEL_CONFIG2, &data, 1);

    return 0;
}

/**
  * @brief  Set the accel full-scale range.
  * @param  fsr Desired full-scale range.
  * @retval 0 if successful.
  */
int mpu_set_accel_fsr(int fsr) {
    uint8_t cfg_bits;
    uint8_t data;

    switch (fsr) {
    case 2:
        cfg_bits = INV_MPU_ACCEL_FSR_2G << 3;
        inv_mpu_accelScale = 0.00006103515625f;
        break;
    case 4:
        cfg_bits = INV_MPU_ACCEL_FSR_4G << 3;
        inv_mpu_accelScale = 0.0001220703125f;
        break;
    case 8:
        cfg_bits = INV_MPU_ACCEL_FSR_8G << 3;
        inv_mpu_accelScale = 0.000244140625f;
        break;
    case 16:
        cfg_bits = INV_MPU_ACCEL_FSR_16G << 3;
        inv_mpu_accelScale = 0.00048828125f;
        break;
    default:
        return -1;
    }

    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_ACCEL_CONFIG, &data, 1);
    data &= ~INV_MPU_MASK_ACCEL_FS_SEL;
    data |= cfg_bits;
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_ACCEL_CONFIG, &data, 1);

    return 0;
}

/**
  * @brief  Set the gyro full-scale range.
  * @param  fsr Desired full-scale range.
  * @retval 0 if successful.
  */
int mpu_set_gyro_fsr(int fsr) {
    uint8_t cfg_bits;
    uint8_t data;

    switch (fsr) {
    case 250:
        cfg_bits = INV_MPU_GYRO_FSR_250DPS << 3;
        inv_mpu_gyroScale = 0.00762939453125f;
        break;
    case 500:
        cfg_bits = INV_MPU_GYRO_FSR_500DPS << 3;
        inv_mpu_gyroScale = 0.0152587890625f;
        break;
    case 1000:
        cfg_bits = INV_MPU_GYRO_FSR_1000DPS << 3;
        inv_mpu_gyroScale = 0.030517578125f;
        break;
    case 2000:
        cfg_bits = INV_MPU_GYRO_FSR_2000DPS << 3;
        inv_mpu_gyroScale = 0.06103515625f;
        break;
    default:
        return -1;
    }

    al_i2c_read(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_GYRO_CONFIG, &data, 1);
    data &= ~INV_MPU_MASK_GYRO_FS_SEL;
    data |= cfg_bits;
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_GYRO_CONFIG, &data, 1);

    return 0;
}

/**
  * @brief  Reset device.
  * @param  None.
  * @retval 0 if successful.
  */
int mpu_reset(void) {
    uint8_t data;

    data = INV_MPU_BIT_H_RESET;
    al_i2c_write(INV_MPU_I2C_FD, INV_MPU_DEV_ADDR, INV_MPU_REG_PWR_MGMT_1, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    return 0;
}

/******************************** END OF FILE *********************************/
