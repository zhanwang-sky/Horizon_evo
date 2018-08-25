/**
  ******************************************************************************
  * @file   al_stm32l4xx_util.c
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with low level peripherals.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#ifdef HORIZON_MINI_L4
#include "nucleo_l432kc_bsp_config.h"
#else
#error please specify a target board
#endif

#include "FreeRTOS.h"
#include "task.h"

/* Function prototypes -------------------------------------------------------*/
extern void xPortSysTickHandler( void );

/* Functions -----------------------------------------------------------------*/
int al_gpio_write_pin(int fd, int state) {
    GPIO_TypeDef *pPort;
    uint16_t pin;

    if (fd >= BSP_NR_GPIOs) {
        return -1;
    }

    BSP_GPIO_AL2HAL(fd, pPort, pin);

    HAL_GPIO_WritePin(pPort, pin, (0 == state) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    return 0;
}

int al_gpio_read_pin(int fd, int *pState) {
    GPIO_TypeDef *pPort;
    uint16_t pin;

    if (fd >= BSP_NR_GPIOs) {
        return -1;
    }


    if (NULL == pState) {
        return -1;
    }

    BSP_GPIO_AL2HAL(fd, pPort, pin);

    *pState = HAL_GPIO_ReadPin(pPort, pin);

    return 0;
}

int al_gpio_toggle_pin(int fd) {
    GPIO_TypeDef *pPort;
    uint16_t pin;

    if (fd >= BSP_NR_GPIOs) {
        return -1;
    }

    BSP_GPIO_AL2HAL(fd, pPort, pin);

    HAL_GPIO_TogglePin(pPort, pin);

    return 0;
}

#ifdef configASSERT
inline void os_assert_failed(void) {
    HAL_Assert_Failed();
}
#endif /* end of configASSERT */

/* Interrupt service routines ------------------------------------------------*/
/**
  * @brief  SYSTICK callback.
  * @param  None
  * @retval None
  */
void HAL_SYSTICK_Callback(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/******************************** END OF FILE *********************************/
