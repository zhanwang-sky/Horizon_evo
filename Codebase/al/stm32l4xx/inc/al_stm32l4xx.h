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

/* Common definitions --------------------------------------------------------*/
/* aio */
#define AIO_FLAGNONE (0)
#define AIO_NONBLOCK (1 << 0)
/* errno */
#define EINTR   (4)
#define EIO     (5)
#define ENXIO   (6)
#define EBADF   (9)
#define ENOMEM  (12)
#define EFAULT  (14)
#define EBUSY   (16)
#define EINVAL  (22)
#define EAGAIN  (35)
#define EPWROFF (82)
#define EDEVERR (83)

/* Includes ------------------------------------------------------------------*/

/* Typedef -------------------------------------------------------------------*/
union sigval {
    int   sival_int;
    void *sival_ptr;
};

struct sigevent {
    union sigval sigev_value;
    int (*sigev_notify_function)(union sigval, int);
};

/* Function prototypes -------------------------------------------------------*/
/* gpio */
int al_gpio_write_pin(int fd, int state);
int al_gpio_read_pin(int fd, int *pState);
int al_gpio_toggle_pin(int fd);
void al_exti_0_callback(void);
void al_exti_1_callback(void);
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
