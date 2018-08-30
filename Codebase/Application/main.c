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
#include "semphr.h"

#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blink;
SemaphoreHandle_t xSem_medium;
SemaphoreHandle_t xSem_high;
char uartTxBuf[128] = { '\0' };

/* Functions -----------------------------------------------------------------*/
void blinkLED(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void printHello(void *pvParameters) {
    uint32_t count = 0;
    TickType_t xLastWakeTime;

    snprintf(uartTxBuf, sizeof(uartTxBuf), "\033c");
    al_uart_write(0, uartTxBuf, strlen(uartTxBuf));

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        count += 1;
        snprintf(uartTxBuf, sizeof(uartTxBuf), "%4u: hello world! (DMA)\r\n", count);
        // wake up medium level task
        xSemaphoreGive(xSem_medium);
        al_uart_write(0, uartTxBuf, strlen(uartTxBuf));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2500));
    }
}

void mediumLevelTask(void *pvParameters) {
    static char mediumBuf[] = "@@@medium level task@@@\r\n";

    while (1) {
        xSemaphoreTake(xSem_medium, portMAX_DELAY);
        // wakeup high level task
        xSemaphoreGive(xSem_high);
        vTaskDelay(pdMS_TO_TICKS(1));
        al_uart_write(0, mediumBuf, strlen(mediumBuf));
    }
}

void highLevelTask(void *pvParameters) {
    static char highBuf[] = "!!!high level task!!!\r\n";

    while (1) {
        xSemaphoreTake(xSem_high, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(3));
        al_uart_write(0, highBuf, strlen(highBuf));
    }
}

int main(void) {
    /* Initialize MCU */
    BSP_MCU_Init();

    /* Adaptation Layer Initialization */
    al_uart_init();

    al_gpio_write_pin(0, 1);

    /* Initialize semaphores */
    xSem_medium = xSemaphoreCreateBinary();
    xSem_high = xSemaphoreCreateBinary();

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

    xTaskCreate(mediumLevelTask,
                "mediumLevelTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3U,
                NULL);

    xTaskCreate(highLevelTask,
                "highLevelTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 4U,
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
