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

/* Definitions ---------------------------------------------------------------*/
#define BSP_NR_GPIOs (1)
#define BSP_NR_EXINTs (0)
#define BSP_NR_I2Cs (0)
#define BSP_NR_UARTs (0)

/* Macros --------------------------------------------------------------------*/
#define BSP_GPIO_AL2HAL(FD, PORT, PIN) \
do { \
    (PORT) = GPIOB; \
    (PIN) = GPIO_PIN_3; \
} while (0)

#endif /* __NUCLEO_L432KC_BSP_CONFIG_H */

/******************************** END OF FILE *********************************/
