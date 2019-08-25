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

/* Global variables ----------------------------------------------------------*/
TimerHandle_t xTimerBlinker;
char msgBuf[121];

/* Functions -----------------------------------------------------------------*/
/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0); // non-blocking API
}

void tPrintHello(void *pvParameters) {
    static int count = 0;
    TickType_t xLastWakeTime;
    int len;
    // DShot test
    char dshotSet = 0;

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // sleep 1s
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        // print hello
        len = snprintf(msgBuf, sizeof(msgBuf), "%04d hello world!\r\n", ++count);
        al_uart_write(1, msgBuf, len);
        // start motor
        if (!dshotSet && count == 10) {
            dshotSet = 1;
            al_tim_dshot_set(0, 200); // 10%
            xTimerChangePeriod(xTimerBlinker, pdMS_TO_TICKS(100), 0); // 10HZ
        }
    }
}

int main(void) {
    /* BSP Init */
    BSP_MCU_Init();

    /* AL Init */
    al_uart_init();
    al_i2c_init();
    al_tim_dshot_init();

    /* Create semaphores */

    /* Create tasks */
    xTaskCreate(tPrintHello,
                "thread tPrintHello",
                configMINIMAL_STACK_SIZE,
                NULL,
                configTIMER_TASK_PRIORITY + 1,
                NULL);

    /* Create software timers */
    xTimerBlinker = xTimerCreate("timer tBlinker",
                                 pdMS_TO_TICKS(1000),
                                 pdTRUE,
                                 NULL,
                                 tBlinker);
    /* Activate timers */
    xTimerStart(xTimerBlinker, 0);

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

#if 0
void al_uart_0_recv_callback(unsigned char c, int ec, int *brk) {
}

void al_uart_1_recv_callback(unsigned char c, int ec, int *brk) {
}
#endif

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
