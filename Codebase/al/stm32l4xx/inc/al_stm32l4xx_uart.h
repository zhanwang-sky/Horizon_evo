/**
  ******************************************************************************
  * @file   al_stm32l4xx_uart.h
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with UART.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __AL_STM32L4XX_UART_H
#define __AL_STM32L4XX_UART_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

/* Typedef -------------------------------------------------------------------*/
struct al_uart_aiocb {
    int             aio_fildes;
    volatile void  *aio_buf;
    size_t          aio_nbytes;
    int             aio_flag;
    int (*handler)(int rc);
};

/* Prototypes ----------------------------------------------------------------*/
int al_uart_init(void);
int al_uart_aio_write(struct al_uart_aiocb *aiocb);
int al_uart_async_read_one(int fd, int (*handler)(unsigned short, int));

#endif /* __AL_STM32L4XX_UART_H */

/******************************** END OF FILE *********************************/
