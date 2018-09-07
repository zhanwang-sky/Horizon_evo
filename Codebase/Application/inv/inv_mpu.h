/**
  ******************************************************************************
  * @file    inv_mpu.h
  * @author  Ji Chen
  * @brief   MPUxxxx sensor dirver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __INV_MPU_H
#define __INV_MPU_H

/* Function prototypes -------------------------------------------------------*/
int mpu_reset(void);
int mpu_set_gyro_fsr(int fsr);
int mpu_set_accel_fsr(int fsr);
int mpu_set_lpf(int lpf);
int mpu_set_sample_rate(int rate);
int mpu_set_int(int enable);

#endif /* __INV_MPU_H */

/******************************** END OF FILE *********************************/
