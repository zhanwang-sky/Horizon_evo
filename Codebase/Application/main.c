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

#include "inv_mpu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blink;
char uartTxBuf[128] = { '\0' };

/* Functions -----------------------------------------------------------------*/
void blinkLED(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void printHello(void *pvParameters) {
    uint32_t count = 0;
    TickType_t xLastWakeTime;
    uint8_t data = 0;

    snprintf(uartTxBuf, sizeof(uartTxBuf), "\033c");
    al_uart_write(0, uartTxBuf, strlen(uartTxBuf));

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        count += 1;
        snprintf(uartTxBuf, sizeof(uartTxBuf), "%4u: hello world! (DMA)\r\n", count);
        al_uart_write(0, uartTxBuf, strlen(uartTxBuf));
        al_i2c_read(0, 0xD0, 0x75, &data, 1);
        snprintf(uartTxBuf, sizeof(uartTxBuf), "---whoami? -0x%x\r\n\n", data);
        al_uart_write(0, uartTxBuf, strlen(uartTxBuf));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
    }
}

int main(void) {
    /* Initialize MCU */
    BSP_MCU_Init();

    /* Adaptation Layer Initialization */
    al_uart_init();
    al_i2c_init();

    al_gpio_write_pin(0, 1);

    /* Create a FreeRTOS timer */
    xTimer_blink = xTimerCreate("blinkLED",
                                pdMS_TO_TICKS(1000),
                                pdTRUE,
                                NULL,
                                blinkLED);
    xTimerStart(xTimer_blink, 0);

    /* Create tasks */
    xTaskCreate(printHello,
                "printHello",
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
