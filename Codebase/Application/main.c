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
#include <string.h>

#ifdef HORIZON_MINI_L4
#include "stm32l4xx_hal.h"
#include "nucleo_l432kc_bsp.h"
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

extern UART_HandleTypeDef huart2;

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blink;

/* Functions -----------------------------------------------------------------*/
void blinkLED(TimerHandle_t xTimer) {
    static uint32_t count = 0;
    char uartTxBuf[128] = { '\0' };

    if (0 == count) {
        snprintf(uartTxBuf, sizeof(uartTxBuf), "\033c");
        HAL_UART_Transmit(&huart2, (uint8_t *) uartTxBuf, strlen(uartTxBuf), 100);
    }
    count++;
    al_gpio_toggle_pin(0);
    snprintf(uartTxBuf, sizeof(uartTxBuf), "%4u: hello world!\r\n", count);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) uartTxBuf, strlen(uartTxBuf));
}

int main(void) {
    /* Initialize MCU */
    BSP_MCU_Init();

    al_gpio_write_pin(0, 1);

    /* Create a FreeRTOS timer */
    xTimer_blink = xTimerCreate("blinkLED",
                                pdMS_TO_TICKS(1000),
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
