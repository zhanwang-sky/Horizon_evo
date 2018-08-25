/**
  ******************************************************************************
  * @file   main.c
  * @author Ji Chen
  * @brief  Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#ifdef HORIZON_MINI_L4
#include "nucleo_l432kc_bsp.h"
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blink;

/* Functions -----------------------------------------------------------------*/
void blinkLED(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

int main(void) {
    /* Initialize MCU */
    BSP_MCU_Init();

    al_gpio_write_pin(0, 1);

    /* Create a FreeRTOS timer */
    xTimer_blink = xTimerCreate("blinkLED",
                                pdMS_TO_TICKS(5000),
                                pdTRUE,
                                NULL,
                                blinkLED);
    xTimerStart(xTimer_blink, 0);

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
       will never be reached.
       If the following line does execute, then there was insufficient FreeRTOS
       heap memory available for the idle and/or timer tasks to be created.
       See the memory management section on the FreeRTOS web site for more
       details. */
    while(1);
}

/******************************** END OF FILE *********************************/
