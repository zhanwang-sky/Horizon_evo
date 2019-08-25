/**
  ******************************************************************************
  * @file   al_stm32l4xx.h
  * @author Ji Chen
  * @brief  Adaptation Layer header file.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __AL_STM32L4XX_H
#define __AL_STM32L4XX_H

/* Function prototypes -------------------------------------------------------*/
/* gpio */
int al_gpio_write_pin(int fd, int state);
int al_gpio_read_pin(int fd, int *pState);
int al_gpio_toggle_pin(int fd);
void al_exti_0_callback(void);
void al_exti_1_callback(void);
/* uart */
int al_uart_init(void);
int al_uart_write(int fd, const void *buf, unsigned int nbytes);
int al_uart_start_receiving(int fd);
void al_uart_0_recv_callback(unsigned char c, int ec, int *brk);
void al_uart_1_recv_callback(unsigned char c, int ec, int *brk);
/* i2c */
int al_i2c_init(void);
int al_i2c_write(int fd, char dev_addr, char reg_addr, const void *buf, unsigned int nbytes);
int al_i2c_read(int fd, char dev_addr, char reg_addr, void *buf, unsigned int nbytes);
/* spi */
int al_spi_init(void);
int al_spi_write(int fd, int subfd, const void *buf, unsigned int nbytes);
int al_spi_read(int fd, int subfd, void *buf, unsigned int nbytes);
int al_spi_write_read(int fd, int subfd, const void *txbuf, void *rxbuf, unsigned int nbytes);
/* timer */
int al_tim_dshot_init(void);
int al_tim_dshot_set(int fd, unsigned int value);
int al_tim_dshot_set4(unsigned int values[4]);

#endif /* __AL_STM32L4XX_H */

/******************************** END OF FILE *********************************/
