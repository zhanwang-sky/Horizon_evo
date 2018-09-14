/**
  ******************************************************************************
  * @file    inv_mpu.h
  * @author  Ji Chen
  * @brief   MPUxxxx sensor driver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __INV_MPU_H
#define __INV_MPU_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Function prototypes -------------------------------------------------------*/
int mpu_reset(void);
int mpu_set_gyro_fsr(int fsr);
int mpu_set_accel_fsr(int fsr);
int mpu_set_lpf(int lpf);
int mpu_set_sample_rate(int rate);
int mpu_set_int(int enable);
int mpu_read_data(uint8_t rawData[14], float data[6]);

#endif /* __INV_MPU_H */

/******************************** END OF FILE *********************************/
