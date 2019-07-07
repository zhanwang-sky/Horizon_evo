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
#define TMP_BUF_LEN 5

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinker;
TimerHandle_t xTimer_DShot_Start;

SemaphoreHandle_t xSem_com0_rx;
SemaphoreHandle_t xSem_com1_rx;

uint8_t tmp_buf[TMP_BUF_LEN] = { 0 };

/* Functions -----------------------------------------------------------------*/
/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tDShot_Start(TimerHandle_t xTimer) {
    al_tim_dshot_set(0, 200);
    al_tim_dshot_set(3, 1024);
    xTimerChangePeriod(xTimer_blinker, pdMS_TO_TICKS(100), 0);
}

void tCom0(void *pvParameters) {
    al_uart_start_receiving(0);
    while (1) {
        xSemaphoreTake(xSem_com1_rx, portMAX_DELAY);
        al_uart_write(0, tmp_buf, TMP_BUF_LEN);
    }
}

void tCom1(void *pvParameters) {
    do {
        al_uart_start_receiving(1);
        xSemaphoreTake(xSem_com0_rx, portMAX_DELAY);
        al_uart_write(1, tmp_buf, TMP_BUF_LEN);
    } while (1);
}

int main(void) {
    /* BSP Init */
    BSP_MCU_Init();

    /* AL Init */
    al_uart_init();
    al_tim_dshot_init();

    /* Create software timers */
    xTimer_blinker = xTimerCreate("tBlinker",
                                  pdMS_TO_TICKS(1000),
                                  pdTRUE,
                                  NULL,
                                  tBlinker);
    xTimer_DShot_Start = xTimerCreate("tDShot_Start",
                                  pdMS_TO_TICKS(10000),
                                  pdFALSE,
                                  NULL,
                                  tDShot_Start);
    /* Activate timers */
    xTimerStart(xTimer_blinker, 0);
    xTimerStart(xTimer_DShot_Start, 0);

    /* Create semaphores */
    xSem_com0_rx = xSemaphoreCreateBinary();
    xSem_com1_rx = xSemaphoreCreateBinary();

    /* Create tasks */
    xTaskCreate(tCom0,
                "tCom0",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(tCom1,
                "tCom1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
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

int al_uart_0_recv_callback(unsigned char c) {
    static int pos = 0;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        tmp_buf[pos] = c;
        if (++pos == TMP_BUF_LEN) {
            pos = 0;
            xSemaphoreGiveFromISR(xSem_com0_rx, NULL);
        }
    }

    return 0;
}

int al_uart_1_recv_callback(unsigned char c) {
    static int pos = TMP_BUF_LEN - 1;
    int brk = 0;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        tmp_buf[pos] = c;
        if (--pos < 0) {
            pos = TMP_BUF_LEN - 1;
            xSemaphoreGiveFromISR(xSem_com1_rx, NULL);
            brk = 1; // stop receiving
        }
    }

    return brk;
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
