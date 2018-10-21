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

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinker;
char cTxBuf[80] = { '\0' };

/* Functions -----------------------------------------------------------------*/
/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tPrintHello(void *pvParameters) {
    int len;
    TickType_t xLastWakeTime;
    // nRF24L01
    uint8_t uTxData = 0xFF, uRxData1 = 0xE1, uRxData2 = 0xE2;

    /* initialize... */
    len = snprintf(cTxBuf, sizeof(cTxBuf), "\033c\033[2Jinitializing...\r\n");
    al_uart_write(0, (uint8_t *) cTxBuf, len);
    vTaskDelay(pdMS_TO_TICKS(100));

    len = snprintf(cTxBuf, sizeof(cTxBuf), "hello world!\r\n");
    al_uart_write(0, (uint8_t *) cTxBuf, len);

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // nRF24L01
        al_spi_write_read(0, 0, &uTxData, &uRxData1, 1);
        al_spi_write_read(0, 0, &uTxData, &uRxData2, 1);
        len = snprintf(cTxBuf, sizeof(cTxBuf),
            "data1 = %02X, data2 = %02X\r\n", uRxData1, uRxData2);
        al_uart_write(0, (uint8_t *) cTxBuf, len);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));
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

/******************************** END OF FILE *********************************/
