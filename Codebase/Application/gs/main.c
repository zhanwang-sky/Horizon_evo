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
#elif defined(HORIZON_STD_L4) || defined(HORIZON_GS_STD_L4)
#include "nucleo_l476rg_bsp.h"
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinkLED;
char TxBuf[80] = { '\0' };

/* Functions -----------------------------------------------------------------*/
void tBlinkLED(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tPrintHello(void *pvParameters) {
    uint32_t n = 0;
    uint8_t data[2];
    int len;
    TickType_t xLastWakeTime;

    len = snprintf(TxBuf, sizeof(TxBuf), "\033cFreeRTOS\r\n");
    al_uart_write(0, (uint8_t *) TxBuf, len);
    al_i2c_read(0, 0xD0, 0x6B, data, 1);
    al_i2c_read(0, 0xD0, 0x75, data + 1, 1);

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        len = snprintf(TxBuf, sizeof(TxBuf), "n: %04u, PWR: %02x, WHOAMI: %02x\r\n", n++, data[0], data[1]);
        al_uart_write(0, TxBuf, len);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    BSP_MCU_Init();

    al_uart_init();
    al_i2c_init();

    /* Create software timers */
    xTimer_blinkLED = xTimerCreate("tBlinkLED",
                                   pdMS_TO_TICKS(1000),
                                   pdTRUE,
                                   NULL,
                                   tBlinkLED);
    /* Activate timers */
    xTimerStart(xTimer_blinkLED, 0);

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

/* ISR callbacks -------------------------------------------------------------*/
void al_exti_2(void) {
    static uint16_t period = 1000;

    period = (period == 1000) ? 100 : 1000;
    xTimerChangePeriodFromISR(xTimer_blinkLED, pdMS_TO_TICKS(period), NULL);
}

/******************************** END OF FILE *********************************/
