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
#define BSP_NR_I2Cs (1)
#define BSP_NR_UARTs (1)

/* Macros --------------------------------------------------------------------*/
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
    (PORT) = GPIOB; \
    (PIN) = GPIO_PIN_3; \
} while (0)

#define BSP_EXTI_PIN2IDX(PIN, INDEX) \
do { \
    switch (PIN) { \
    case GPIO_PIN_1: \
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
} while(0)

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

#endif /* __NUCLEO_L432KC_BSP_CONFIG_H */

/******************************** END OF FILE *********************************/
