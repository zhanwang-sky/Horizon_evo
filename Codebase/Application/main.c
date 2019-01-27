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

/* Definitions ---------------------------------------------------------------*/
#define INTERACT_UART_FD 0
#define INTERACT_UART_TXBUF_LEN 81

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinker;
char uartTxBuf[INTERACT_UART_TXBUF_LEN] = { '\0' };

/* Functions -----------------------------------------------------------------*/
/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tInteraction(void *pvParameters) {
    TickType_t xLastWakeTime;
    int len;
    uint16_t n = 0;
    // I2C test
    int rc;
    uint8_t data = 0x88;

    /* initialize... */
    len = snprintf(uartTxBuf, sizeof(uartTxBuf), "\033c\033[2JInitializing...\r\n");
    al_uart_write(INTERACT_UART_FD, (uint8_t *) uartTxBuf, len);
    vTaskDelay(pdMS_TO_TICKS(100));

    // I2C test
    rc = al_i2c_read(0, 0xD0, 0x75, &data, 1);
    if (rc != 0) {
        len = snprintf(uartTxBuf, sizeof(uartTxBuf), "I2C bus error!\r\n");
        al_uart_write(INTERACT_UART_FD, (uint8_t *) uartTxBuf, len);
    }

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        len = snprintf(uartTxBuf, sizeof(uartTxBuf), "%5u: data = 0x%X\r\n", n++, data);
        al_uart_write(INTERACT_UART_FD, (uint8_t *) uartTxBuf, len);
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
    xTaskCreate(tInteraction,
                "tInteraction",
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
void al_exti_2_callback(void) {
    static uint32_t period = 1000;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        period = (period == 1000) ? 100 : 1000;
        xTimerChangePeriodFromISR(xTimer_blinker, pdMS_TO_TICKS(period), NULL);
    }
}
#endif

/******************************** END OF FILE *********************************/
