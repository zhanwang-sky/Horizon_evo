/**
  ******************************************************************************
  * @file   nucleo_l476rg_bsp_config.h
  * @author Ji Chen
  * @brief  NUCLEO-L476RG BSP configuration file.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __NUCLEO_L476RG_BSP_CONFIG_H
#define __NUCLEO_L476RG_BSP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Definitions ---------------------------------------------------------------*/
#define BSP_NR_GPIOs (2)
#define BSP_NR_EXTIs (3)
#define BSP_NR_I2Cs (1)
#define BSP_NR_UARTs (1)
#define BSP_NR_SPIs (1)
#define BSP_NR_SPI_NSS (1)

/* Macros --------------------------------------------------------------------*/
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
    if (0 == (FD)) { \
        /* PA5(LD2 [green Led]) */ \
        (PORT) = GPIOA; \
        (PIN) = GPIO_PIN_5; \
    } else { \
        /* PB10(nRF_CE) */ \
        (PORT) = GPIOB; \
        (PIN) = GPIO_PIN_10; \
    } \
} while (0)

#define BSP_EXTI_PIN2IDX(PIN, INDEX) \
do { \
    switch (PIN) { \
    case GPIO_PIN_9: \
        /* PC9(MPU_INT) */ \
        (INDEX) = 0; \
        break; \
    case GPIO_PIN_4: \
        /* PB4(nRF_IRQ) */ \
        (INDEX) = 1; \
        break; \
    case GPIO_PIN_13: \
        /* PC13(B1 [Blue PushButton]) */ \
        (INDEX) = 2; \
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

#define BSP_SPI_FD2IDXHDL(FD, INDEX, HSPI) \
do { \
    (INDEX) = 0; \
    (HSPI) = &hspi2; \
} while(0)

#define BSP_SPI_SUBFD2PORTPIN(SUBFD, PORT, PIN) \
do { \
    (PORT) = GPIOB; \
    (PIN) = GPIO_PIN_5; \
} while(0)

#define BSP_SPI_HDL2IDX(HSPI, INDEX) \
do { \
    (INDEX) = 0; \
} while(0)

#define BSP_SPI_HDL2PORTPIN(HSPI, PORT, PIN) \
do { \
    (PORT) = GPIOB; \
    (PIN) = GPIO_PIN_5; \
} while (0)

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;

#endif /* __NUCLEO_L476RG_BSP_CONFIG_H */

/******************************** END OF FILE *********************************/
