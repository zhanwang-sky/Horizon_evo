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
/* uart */
int al_uart_init(void);
int al_uart_write(int fd, const void *buf, unsigned int nbytes);
/* i2c */
int al_i2c_init(void);
int al_i2c_write(int fd, char dev_addr, char reg_addr, const void *buf, unsigned int nbytes);
int al_i2c_read(int fd, char dev_addr, char reg_addr, void *buf, unsigned int nbytes);

#endif /* __AL_STM32L4XX_H */

/******************************** END OF FILE *********************************/
