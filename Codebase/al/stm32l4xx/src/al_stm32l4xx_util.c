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
#if defined(HORIZON_MINI_L4)
#include "nucleo_l432kc_bsp_config.h"
#elif defined(HORIZON_STD_L4) || defined(HORIZON_GS_STD_L4)
#include "nucleo_l476rg_bsp_config.h"
#else
#error please specify a target board
#endif

#include "FreeRTOS.h"
#include "task.h"

/* Function prototypes -------------------------------------------------------*/
extern void xPortSysTickHandler( void );

/* Functions -----------------------------------------------------------------*/
int al_gpio_write_pin(int fd, int state) {
    GPIO_TypeDef *port;
    uint16_t pin;

    if (fd < 0 || fd >= BSP_NR_GPIOs) {
        return -1;
    }

    BSP_GPIO_FD2PORTPIN(fd, port, pin);

    HAL_GPIO_WritePin(port, pin, (0 == state) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    return 0;
}

int al_gpio_read_pin(int fd, int *pState) {
    GPIO_TypeDef *port;
    uint16_t pin;

    if (fd < 0 || fd >= BSP_NR_GPIOs || NULL == pState) {
        return -1;
    }

    BSP_GPIO_FD2PORTPIN(fd, port, pin);

    *pState = (GPIO_PIN_RESET == HAL_GPIO_ReadPin(port, pin)) ? 0 : 1;

    return 0;
}

int al_gpio_toggle_pin(int fd) {
    GPIO_TypeDef *port;
    uint16_t pin;

    if (fd < 0 || fd >= BSP_NR_GPIOs) {
        return -1;
    }

    BSP_GPIO_FD2PORTPIN(fd, port, pin);

    HAL_GPIO_TogglePin(port, pin);

    return 0;
}

__weak void al_exti_0_callback(void) {
    return;
}

__weak void al_exti_1_callback(void) {
    return;
}

__weak void al_exti_2_callback(void) {
    return;
}

#ifdef configASSERT
inline void os_assert_failed(void) {
    HAL_Assert_Failed();
}
#endif /* configASSERT */

/* ISR callbacks -------------------------------------------------------------*/
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

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    int index;

    BSP_EXTI_PIN2IDX(GPIO_Pin, index);

    if (0 == index) {
        al_exti_0_callback();
    } else if (1 == index) {
        al_exti_1_callback();
    } else if (2 == index) {
        al_exti_2_callback();
    }
}

/******************************** END OF FILE *********************************/
