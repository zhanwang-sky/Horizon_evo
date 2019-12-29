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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if defined(HORIZON_MINI_L4)
#include "al_stm32l4xx.h"
#include "nucleo_l432kc_bsp.h"
#elif defined(HORIZON_GS_STD_L4)
#include "al_stm32l4xx.h"
#include "nucleo_l476rg_bsp.h"
#else
#error please specify a target board
#endif

/* Definitions ---------------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
SemaphoreHandle_t gSem_printBlob;
unsigned int g_count = 0;
const char g_strBlob[] =
    "~~~~~~~~~~\r\n"
    "magnet:?xt=urn:btih:70992b776f4c0f37dcaf3c90e6556bd309fb29f1\r\n"
    "magnet:?xt=urn:btih:1314d891b36decc52739fce38913ed25749684b7\r\n"
    "magnet:?xt=urn:btih:b73f37e438d2486ef2d0fc994c9e7331f55dcb69\r\n"
    "magnet:?xt=urn:btih:43f2baf7b99fe014979c54a3c4ec8f5d4f5c0d2f\r\n"
    "magnet:?xt=urn:btih:e6968f0d0057cb359b7f33f03f7db7b8f9055c55\r\n"
    "magnet:?xt=urn:btih:76044c274fb699214be6c7027ad0cba4d8d1dfdd\r\n"
    "magnet:?xt=urn:btih:45dc0af919f63018d9391ba07118a93af58c9df7\r\n"
    "magnet:?xt=urn:btih:bd05eef248405ecb5ed7b61804010747a90e7921\r\n"
    "~~~~~~~~~~\r\n";

/* Functions -----------------------------------------------------------------*/
int onRecv(unsigned short data, int rc) {
    static int i = 0;
    BaseType_t higherPriorityTaskWoken;

    if (rc > 0) {
        if (data == 'a') {
            i = 1;
        } else {
            if (i == 1 && data == 'Z') {
                xSemaphoreGiveFromISR(gSem_printBlob, &higherPriorityTaskWoken);
                portYIELD_FROM_ISR(higherPriorityTaskWoken);
            }
            i = 0;
        }
    } else {
        i = 0;
    }

    return 0;
}

int onTxBlobDone(int rc) {
    if (rc > 0) {
        BSP_SYSLED_Set_Normal();
    } else {
        BSP_SYSLED_Set_Fault();
    }
    return 0;
}

int onTxHelloDone(int rc) {
    if (rc > 0) {
        if ((g_count % 10) == 0) {
            BSP_SYSLED_Set_UnderInit();
        }
    } else {
        BSP_SYSLED_Set_Fault();
    }
    return 0;
}

/* Threads */
void thr_printBlob(void *pvParameters) {
    struct al_uart_aiocb aiocb = {
        1,
        (void*) g_strBlob,
        sizeof(g_strBlob),
        0,
        onTxBlobDone,
    };

    while (1) {
        xSemaphoreTake(gSem_printBlob, portMAX_DELAY);
        al_uart_aio_write(&aiocb);
    }
}

void thr_main(void *pvParameters) {
    static char msgBuf[81];
    TickType_t lastWakeTime;
    struct al_uart_aiocb aiocb = { 1, msgBuf, 0, AIO_NONBLOCK, onTxHelloDone };

    al_uart_async_read_one(1, onRecv);

    lastWakeTime = xTaskGetTickCount();
    while (1) {
        aiocb.aio_nbytes = snprintf(msgBuf, sizeof(msgBuf),
            "[%u] hello, aio!\r\n", g_count++);
        al_uart_aio_write(&aiocb);
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
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
    gSem_printBlob = xSemaphoreCreateBinary();

    /* Create events */

    /* Create tasks */
    xTaskCreate(thr_main,
                "the main thread",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(thr_printBlob,
                "thread to print strBlob",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
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
