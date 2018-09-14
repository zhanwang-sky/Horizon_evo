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

#ifdef HORIZON_MINI_L4
#include "nucleo_l432kc_bsp.h"
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

#include "inv_mpu.h"

/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t xSem_mpu;
SemaphoreHandle_t xSem_msgReady;
SemaphoreHandle_t xSem_msgCplt;
TimerHandle_t xTimer_blink;
uint8_t mpu_rawData[14];
float mpu_data[6];
char uartTxBuf[128] = { '\0' };

/* Functions -----------------------------------------------------------------*/
void init_mpu(void) {
    mpu_reset();
    mpu_set_gyro_fsr(1000);
    mpu_set_accel_fsr(8);
    mpu_set_lpf(92);
    mpu_set_sample_rate(200);
    vTaskDelay(pdMS_TO_TICKS(50));
    mpu_set_int(1);

    return;
}

void tBlinkLED(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tLog(void *pvParameters) {
    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        xSemaphoreGive(xSem_msgCplt);
        xSemaphoreTake(xSem_msgReady, portMAX_DELAY);
        al_uart_write(0, uartTxBuf, strlen(uartTxBuf));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}

void tAHRS(void *pvParameters) {
    int n;

    init_mpu();

    while (1) {
        xSemaphoreTake(xSem_mpu, portMAX_DELAY);
        mpu_read_data(mpu_rawData, mpu_data);
        if (xSemaphoreTake(xSem_msgCplt, 0)) {
            mpu_data[0] -= 0.13f;
            mpu_data[1] -= 0.38f;
            mpu_data[2] -= 0.5f;
            n = snprintf(uartTxBuf, sizeof(uartTxBuf),
                         "\033cgyro: %f, %f, %f\r\n",
                         mpu_data[0], mpu_data[1], mpu_data[2]);
            n += snprintf(uartTxBuf + n, sizeof(uartTxBuf) - n,
                          "accel: %f, %f, %f\r\n",
                          mpu_data[3], mpu_data[4], mpu_data[5]);
            xSemaphoreGive(xSem_msgReady);
        }
    }
}

int main(void) {
    /* Initialize MCU */
    BSP_MCU_Init();

    /* Adaptation Layer Initialization */
    al_uart_init();
    al_i2c_init();

    al_gpio_write_pin(0, 1);

    /* Initialize semaphores */
    xSem_mpu = xSemaphoreCreateBinary();
    xSem_msgReady = xSemaphoreCreateBinary();
    xSem_msgCplt = xSemaphoreCreateBinary();

    /* Create a FreeRTOS timer */
    xTimer_blink = xTimerCreate("tBlinkLED",
                                pdMS_TO_TICKS(1000),
                                pdTRUE,
                                NULL,
                                tBlinkLED);
    xTimerStart(xTimer_blink, 0);

    /* Create tasks */
    xTaskCreate(tLog,
                "tLog",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1U,
                NULL);

    /* Create tasks */
    xTaskCreate(tAHRS,
                "tAHRS",
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
void al_exti_0(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreGiveFromISR(xSem_mpu, NULL);
    }

    return;
}

/******************************** END OF FILE *********************************/
