/**
  ******************************************************************************
  * @file   icm20648.h
  * @author Ji Chen
  * @brief  ICM20648 driver.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef _ICM20648_H
#define _ICM20648_H

/* Definitions ---------------------------------------------------------------*/
typedef enum mpu_lp_mode {
    MPU_LP_DUTY_CYCLE = 0,
    MPU_LP_LOW_NOISE,
    NUM_MPU_LP
} mpu_lp_mode_t;

typedef enum mpu_accel_fs {
    MPU_FS_2G = 0,
    MPU_FS_4G,
    MPU_FS_8G,
    MPU_FS_16G,
    NUM_MPU_AFS
} mpu_accel_fs_t;

typedef enum mpu_gyro_fs {
    MPU_FS_250DPS = 0,
    MPU_FS_500DPS,
    MPU_FS_1000DPS,
    MPU_FS_2000DPS,
    NUM_MPU_GFS
} mpu_gyro_fs_t;

/* Functions -----------------------------------------------------------------*/
int inv_icm_soft_reset(void);
int inv_icm_wakeup(void);
int inv_icm_enable_odr_align(void);
int inv_icm_set_gyro_smplrt(int);
int inv_icm_set_gfs(mpu_gyro_fs_t);
int inv_icm_set_accel_smplrt(int);
int inv_icm_set_afs(mpu_accel_fs_t);
int inv_icm_set_lp_mode(mpu_lp_mode_t);
int inv_icm_enable_int(void);

#endif /* _ICM20648_H */

/******************************** END OF FILE *********************************/
