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
#define BSP_NR_GPIOs (2)
#define BSP_NR_EXTIs (2)
#define BSP_NR_UARTs (2)
#define BSP_NR_I2Cs (0)
#define BSP_NR_SPIs (0)
#define BSP_NR_SPI_NSS (2)
#define BSP_NR_DSHOT_CHANNELs (4)
#define BSP_NR_DSHOT_TIMERs (2)

#define BSP_TARGET_SYSCLK (72000000)
#define BSP_SYSCLK_PLLM (1)
#define BSP_SYSCLK_PLLN (18)
#define BSP_SYSCLK_PLLP RCC_PLLP_DIV7
#define BSP_SYSCLK_PLLQ RCC_PLLQ_DIV2
#define BSP_SYSCLK_PLLR RCC_PLLR_DIV2

#define BSP_UART_BAUD_RATE (9600)
#define BSP_I2C_TIMING (0x00702681)
#define BSP_SPI_PRESCALER SPI_BAUDRATEPRESCALER_128
#define BSP_TIM_DSHOT_CNT_CLK (1500000)
#define BSP_TIM_DSHOT_PRESCALER (BSP_TARGET_SYSCLK / BSP_TIM_DSHOT_CNT_CLK - 1)
#define BSP_TIM_DSHOT_BAUD_RATE (150000)
#define BSP_TIM_DSHOT_PERIOD (BSP_TIM_DSHOT_CNT_CLK / BSP_TIM_DSHOT_BAUD_RATE)
#define BSP_TIM_DSHOT_CODE0 (BSP_TIM_DSHOT_PERIOD * 3 / 10)
#define BSP_TIM_DSHOT_CODE1 (BSP_TIM_DSHOT_PERIOD * 7 / 10)

/* Macros --------------------------------------------------------------------*/
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
    if ((FD) == 0) { \
        /* PB3(D0) */ \
        PORT = GPIOB; \
        PIN = GPIO_PIN_3; \
    } else { \
        /* PB1(D5\A2) */ \
        PORT = GPIOB; \
        PIN = GPIO_PIN_1; \
    } \
} while (0)

#define BSP_EXTI_PIN2IDX(PIN, INDEX) \
do { \
    switch (PIN) { \
    case GPIO_PIN_14: \
        /* PC14 (INT0\D1) */ \
        INDEX = 0; \
        break; \
    case GPIO_PIN_0: \
        /* PB0(INT1\D4\A1) */ \
        INDEX = 1; \
        break; \
    default: \
        INDEX = -1; \
    } \
} while (0)

#define BSP_UART_FD2IDXHDL(FD, INDEX, HUART) \
do { \
    if ((FD) == 0) { \
        INDEX = 0; \
        HUART = &huart1; \
    } else { \
        INDEX = 1; \
        HUART = &huart2; \
    } \
} while (0)

#define BSP_UART_HDL2IDX(HUART, INDEX) \
do { \
    if (USART1 == HUART->Instance) { \
        INDEX = 0; \
    } else { \
        INDEX = 1; \
    } \
} while (0)

#ifdef _REVISE_N_OPTIMIZE_
#define BSP_I2C_FD2IDXHDL(FD, INDEX, HI2C) \
do { \
    INDEX = 0; \
    HI2C = &hi2c1; \
} while (0)

#define BSP_I2C_HDL2IDX(HI2C, INDEX) \
do { \
    INDEX = 0; \
} while (0)

#define BSP_SPI_IS_VALID_FD(FD, SUBFD) \
    ((FD) >= 0 && (FD) < BSP_NR_SPIs && (SUBFD) >= 0 && (SUBFD) < BSP_NR_SPI_NSS)

#define BSP_SPI_FD2IDXHDL(FD, INDEX, HSPI) \
do { \
    INDEX = 0; \
    HSPI = &hspi1; \
} while (0)

#define BSP_SPI_SUBFD2PORTPIN(SUBFD, PORT, PIN) \
do { \
    if ((SUBFD) == 0) { \
        PORT = GPIOC; \
        PIN = GPIO_PIN_15; \
    } else { \
        PORT = GPIOA; \
        PIN = GPIO_PIN_4; \
    } \
} while (0)

#define BSP_SPI_HDL2IDX(HSPI, INDEX) \
do { \
    INDEX = 0; \
} while (0)
#endif // _REVISE_N_OPTIMIZE_

#define BSP_TIM_IS_DSHOT_TIMER(HTIM) \
    ((TIM1 == HTIM->Instance || TIM15 == HTIM->Instance) ? 1 : 0)

#define BSP_TIM_DSHOT_HDL2DMAPARAMS(HTIM, TIMID, BASE, OFFSET, LEN) \
do { \
    if (TIM1 == HTIM->Instance) { \
        TIMID = 0; \
        BASE = TIM_DMABASE_CCR1; \
        OFFSET = 0; \
        LEN = TIM_DMABURSTLENGTH_3TRANSFERS; \
    } else { \
        TIMID = 1; \
        BASE = TIM_DMABASE_CCR2; \
        OFFSET = 18 * 3; \
        LEN = TIM_DMABURSTLENGTH_1TRANSFER; \
    } \
} while (0)

#define BSP_TIM_DSHOT_TIMID2DMAPARAMS(INDEX, HTIM, BASE, OFFSET, LEN) \
do { \
    if (0 == (INDEX)) { \
        HTIM = &htim1; \
        BASE = TIM_DMABASE_CCR1; \
        OFFSET = 0; \
        LEN = TIM_DMABURSTLENGTH_3TRANSFERS; \
    } else { \
        HTIM = &htim15; \
        BASE = TIM_DMABASE_CCR2; \
        OFFSET = 18 * 3; \
        LEN = TIM_DMABURSTLENGTH_1TRANSFER; \
    } \
} while (0)

#define BSP_TIM_DSHOT_CHID2TIMID(CHID, TIMID) \
do { \
    if ((CHID) < 3) { \
        TIMID = 0; \
    } else { \
        TIMID = 1; \
    } \
} while (0)

#define BSP_TIM_DSHOT_CHID2IDINTIM(CHID, IDINTIM) \
do { \
    if ((CHID) < 3) { \
        IDINTIM = (CHID); \
    } else { \
        IDINTIM = (CHID) - 3; \
    } \
} while (0)

#define BSP_TIM_DSHOT_CHID2NRCHS(CHID, NRCHS) \
do { \
    if ((CHID) < 3) { \
        NRCHS = 3; \
    } else { \
        NRCHS = 1; \
    } \
} while (0)

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#ifdef _REVISE_N_OPTIMIZE_
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
#endif // _REVISE_N_OPTIMIZE_
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;

#endif /* __NUCLEO_L432KC_BSP_CONFIG_H */

/******************************** END OF FILE *********************************/
