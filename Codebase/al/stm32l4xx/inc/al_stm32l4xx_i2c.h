/**
  ******************************************************************************
  * @file   al_stm32l4xx_i2c.h
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with I2C.
  ******************************************************************************
  * @attention
  * I. aio APIs should be used in task context.
  * II. however, handlers (callbacks) will be executed in interrupt context.
  ******************************************************************************
  */

#ifndef _AL_STM32L4XX_I2C_H
#define _AL_STM32L4XX_I2C_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "al_stm32l4xx.h"

/* Typedef -------------------------------------------------------------------*/
struct al_i2c_aiocb {
    int             aio_fildes;
    unsigned short  aio_devadd;
    unsigned short  aio_memadd;
    volatile void  *aio_buf;
    size_t          aio_nbytes;
    int             aio_flag;
    struct sigevent aio_sigevent;
};

/* Function prototypes -------------------------------------------------------*/
int al_i2c_init(void);
int al_i2c_aio_write(struct al_i2c_aiocb *aiocb);
int al_i2c_aio_read(struct al_i2c_aiocb *aiocb);

#endif /* _AL_STM32L4XX_I2C_H */

/******************************** END OF FILE *********************************/
