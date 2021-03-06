/**
  ******************************************************************************
  * @file    pni_rm3100.h
  * @author  Ji Chen
  * @brief   RM3100 sensor dirver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __PNI_RM3100_H
#define __PNI_RM3100_H

/* Function prototypes -------------------------------------------------------*/
int pni_init(int cycle_count);
int pni_measurement_once(void);
int pni_read_data(int mag[3]);

#endif /* __PNI_RM3100_H */

/******************************** END OF FILE *********************************/
