/**
  ******************************************************************************
  * @file   al_stm32l4xx_uart.h
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with UART.
  ******************************************************************************
  * @attention
  * I. aio APIs should be used in task context.
  * II. however, handlers (callbacks) will be executed in interrupt context.
  ******************************************************************************
  */

#ifndef _AL_STM32L4XX_UART_H
#define _AL_STM32L4XX_UART_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include "al_stm32l4xx.h"

/* Typedef -------------------------------------------------------------------*/
struct al_uart_aiocb {
    int             aio_fildes;
    volatile void  *aio_buf;
    size_t          aio_nbytes;
    int             aio_flag;
    struct sigevent aio_sigevent;
};

/* Prototypes ----------------------------------------------------------------*/
int al_uart_init(void);
int al_uart_aio_write(struct al_uart_aiocb *aiocb);
int al_uart_async_read_one(int fd, int (*notifier)(unsigned short, int));

#endif /* _AL_STM32L4XX_UART_H */

/******************************** END OF FILE *********************************/
