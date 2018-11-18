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
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#if defined(HORIZON_MINI_L4)
#include "nucleo_l432kc_bsp.h"
#include "al_stm32l4xx.h"
#elif defined(HORIZON_GS_STD_L4)
#include "nucleo_l476rg_bsp.h"
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

#include "nrf24l01.h"

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinker;
char uartTxBuf[81] = { '\0' };

/* Functions -----------------------------------------------------------------*/
/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tPrintHello(void *pvParameters) {
    int len;
    TickType_t xLastWakeTime;
    int n = 0;

    /* initialize... */
    len = snprintf(uartTxBuf, sizeof(uartTxBuf), "\033c\033[2Jinitializing...\r\n");
    al_uart_write(0, (uint8_t *) uartTxBuf, len);
    vTaskDelay(pdMS_TO_TICKS(100));

    nrf_power_up(0);
    nrf_power_up(1);

    len = snprintf(uartTxBuf, sizeof(uartTxBuf), "hello nRF24L01!\r\n");
    al_uart_write(0, (uint8_t *) uartTxBuf, len);

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        len = snprintf(uartTxBuf, sizeof(uartTxBuf), "%4d\r\n", ++n);
        al_uart_write(0, (uint8_t *) uartTxBuf, len);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    /* BSP Init */
    BSP_MCU_Init();

    /* AL Init */
    al_uart_init();
    al_i2c_init();
    al_spi_init();

    /* Create software timers */
    xTimer_blinker = xTimerCreate("tBlinker",
                                  pdMS_TO_TICKS(1000),
                                  pdTRUE,
                                  NULL,
                                  tBlinker);
    /* Activate timers */
    xTimerStart(xTimer_blinker, 0);

    /* Create tasks */
    xTaskCreate(tPrintHello,
                "tPrintHello",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2U,
                NULL);

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

#if defined(HORIZON_GS_STD_L4)
void al_exti_2(void) {
    static uint32_t period = 1000;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        period = (period == 1000) ? 100 : 1000;
        xTimerChangePeriodFromISR(xTimer_blinker, pdMS_TO_TICKS(period), NULL);
    }
}
#endif

/******************************** END OF FILE *********************************/
