/**
  ******************************************************************************
  * @file   nucleo_l432kc_bsp_config.h
  * @author Ji Chen
  * @brief  NUCLEO-L432KC BSP configuration file.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __NUCLEO_L432KC_BSP_CONFIG_H
#define __NUCLEO_L432KC_BSP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Definitions ---------------------------------------------------------------*/
#define BSP_NR_GPIOs (1)
#define BSP_NR_EXTIs (1)
#define BSP_NR_UARTs (1)
#define BSP_NR_I2Cs (1)
#define BSP_NR_SPIs (1)
#define BSP_NR_SPI_NSS (1)

#define BSP_SYSTEM_CORE_CLOCK (80000000U)
#define BSP_UART_BAUD_RATE (9600U)
#define BSP_I2C_TIMING (0x00702991)
#define BSP_SPI_PRESCALER SPI_BAUDRATEPRESCALER_16
#define BSP_DSHORT_TARGET_CNT_CLK (2000U)
#define BSP_DSHORT_TIMER_PRESCALER (BSP_SYSTEM_CORE_CLOCK / BSP_DSHORT_TARGET_CNT_CLK - 1U)
#define BSP_DSHORT_TARGET_FRQ (1U)
#define BSP_DSHORT_TIMER_PERIOD (BSP_DSHORT_TARGET_CNT_CLK / BSP_DSHORT_TARGET_FRQ)

/* Macros --------------------------------------------------------------------*/
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
    /* PB3(LD3 [Green]) */ \
    (PORT) = GPIOB; \
    (PIN) = GPIO_PIN_3; \
} while (0)

#define BSP_EXTI_PIN2IDX(PIN, INDEX) \
do { \
    switch (PIN) { \
    case GPIO_PIN_1: \
        /* PB1(MPU_INT) */ \
        (INDEX) = 0; \
        break; \
    default: \
        (INDEX) = -1; \
    } \
} while (0)

#define BSP_UART_FD2IDXHDL(FD, INDEX, HUART) \
do { \
    (INDEX) = 0; \
    (HUART) = &huart2; \
} while (0)

#define BSP_UART_HDL2IDX(HUART, INDEX) \
do { \
    (INDEX) = 0; \
} while (0)

#define BSP_I2C_FD2IDXHDL(FD, INDEX, HI2C) \
do { \
    (INDEX) = 0; \
    (HI2C) = &hi2c1; \
} while (0)

#define BSP_I2C_HDL2IDX(HI2C, INDEX) \
do { \
    (INDEX) = 0; \
} while (0)

#define BSP_SPI_FD2IDXHDL(FD, INDEX, HSPI) \
do { \
    (INDEX) = 0; \
    (HSPI) = &hspi1; \
} while (0)

#define BSP_SPI_SUBFD2PORTPIN(SUBFD, PORT, PIN) \
do { \
    (PORT) = GPIOA; \
    (PIN) = GPIO_PIN_4; \
} while (0)

#define BSP_SPI_HDL2IDX(HSPI, INDEX) \
do { \
    (INDEX) = 0; \
} while (0)

#define BSP_SPI_HDL2PORTPIN(HSPI, PORT, PIN) \
do { \
    (PORT) = GPIOA; \
    (PIN) = GPIO_PIN_4; \
} while (0)

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

#endif /* __NUCLEO_L432KC_BSP_CONFIG_H */

/******************************** END OF FILE *********************************/
