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
#define BMX055_ACC_DEV_ADDR (0x18 << 1)
#define BMX055_ACC_REG_ID   (0)
#define BMX055_GYR_DEV_ADDR (0X68 << 1)
#define BMX055_GYR_REG_ID   (0)
#define BMX055_MAG_DEV_ADDR (0x10 << 1)
#define BMX055_MAG_REG_ID   (0x40)
#define BMX055_MAG_REG_PWR  (0x4B)
#define BME280_DEV_ADDR     (0x76 << 1)
#define BME280_REG_ID       (0xD0)

/* Global variables ----------------------------------------------------------*/
SemaphoreHandle_t gSem_patternRecv;
SemaphoreHandle_t gSem_bmxXferCplt;
int g_bmeXferEc;
unsigned int g_seq = 0;
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
int uart1_dataRecv(unsigned short data, int rc) {
    static unsigned short lastData = 0;
    BaseType_t higherPriorityTaskWoken;

    if (rc > 0) {
        if (lastData == 'a' && data == 'Z') {
            higherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(gSem_patternRecv, &higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
        lastData = data;
    } else {
        lastData = 0;
    }

    return 0;
}

int printBlobDone(int rc) {
    if (rc > 0) {
        BSP_SYSLED_Set_Normal();
    } else {
        BSP_SYSLED_Set_Fault();
    }
    return 0;
}

int printMsgDone(int rc) {
    if (rc > 0) {
        if ((g_seq % 10) == 0) {
            BSP_SYSLED_Set_UnderInit();
        }
    } else {
        BSP_SYSLED_Set_Fault();
    }
    return 0;
}

int bmxXferCplt(int ec) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    g_bmeXferEc = ec;
    xSemaphoreGiveFromISR(gSem_bmxXferCplt, &higherPriorityTaskWoken);
    return (higherPriorityTaskWoken == pdTRUE) ? 1 : 0;
}

/* Threads */
void thr_printBlob(void *pvParameters) {
    struct al_uart_aiocb aiocb = { 1, (void*) g_strBlob, sizeof(g_strBlob), AIO_FLAGNONE, printBlobDone };

    while (1) {
        xSemaphoreTake(gSem_patternRecv, portMAX_DELAY);
        al_uart_aio_write(&aiocb);
    }
}

void thr_main(void *pvParameters) {
    static const unsigned char bmxDevs[4] = { BMX055_ACC_DEV_ADDR, BMX055_GYR_DEV_ADDR, BMX055_MAG_DEV_ADDR, BME280_DEV_ADDR };
    static const unsigned char bmxRegs[4] = { BMX055_ACC_REG_ID, BMX055_GYR_REG_ID, BMX055_MAG_REG_ID, BME280_REG_ID };
    static volatile char bmxBuf[1];
    static char msgBuf[81];
    unsigned int seq;
    int msgLen;
    int rc;
    TickType_t lastWakeTime;
    struct al_uart_aiocb uart1_aiocb = { 1, msgBuf, 0, AIO_NONBLOCK, NULL };
    struct al_i2c_aiocb i2c0_aiocb = { 0, BME280_DEV_ADDR, BME280_REG_ID, bmxBuf, 1, AIO_NONBLOCK, NULL };

    /* prestage */
    vTaskDelay(pdMS_TO_TICKS(100));

    uart1_aiocb.aio_nbytes = snprintf(msgBuf, sizeof(msgBuf),
        "write 0x01 to BMX055 PWR register\r\n");
    al_uart_aio_write(&uart1_aiocb);

    bmxBuf[0] = 1;
    i2c0_aiocb.aio_devadd = BMX055_MAG_DEV_ADDR;
    i2c0_aiocb.aio_memadd = BMX055_MAG_REG_PWR;
    al_i2c_aio_write(&i2c0_aiocb);

    vTaskDelay(pdMS_TO_TICKS(100));

    /* prepare */
    // setup uart aio_handler
    uart1_aiocb.aio_handler = printMsgDone;
    // setup i2c aio_handler
    i2c0_aiocb.aio_handler = bmxXferCplt;
    // start UART reading
    al_uart_async_read_one(1, uart1_dataRecv);

    /* start loop */
    lastWakeTime = xTaskGetTickCount();
    while (1) {
        seq = g_seq++;

        i2c0_aiocb.aio_devadd = bmxDevs[seq % 4];
        i2c0_aiocb.aio_memadd = bmxRegs[seq % 4];
        rc = al_i2c_aio_read(&i2c0_aiocb);

        msgLen = snprintf(msgBuf, sizeof(msgBuf), "[%d] ", seq);
        if (rc) {
            msgLen += snprintf(msgBuf + msgLen, sizeof(msgBuf) - msgLen,
                "aio error(%d)\r\n", rc);
        } else {
            if (xSemaphoreTake(gSem_bmxXferCplt, pdMS_TO_TICKS(10)) != pdTRUE) {
                msgLen += snprintf(msgBuf + msgLen, sizeof(msgBuf) - msgLen,
                    "timeout\r\n");
            } else {
                if (g_bmeXferEc) {
                    msgLen += snprintf(msgBuf + msgLen, sizeof(msgBuf) - msgLen,
                        "aio post error(%d)\r\n", g_bmeXferEc);
                } else {
                    msgLen += snprintf(msgBuf + msgLen, sizeof(msgBuf) - msgLen,
                        "1 byte from DEV 0x%02X, REG 0x%02X: 0x%02X\r\n",
                        i2c0_aiocb.aio_devadd, i2c0_aiocb.aio_memadd, bmxBuf[0]);
                }
            }
        }
        uart1_aiocb.aio_nbytes = msgLen;
        al_uart_aio_write(&uart1_aiocb);
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
    gSem_patternRecv = xSemaphoreCreateBinary();
    gSem_bmxXferCplt = xSemaphoreCreateBinary();

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
