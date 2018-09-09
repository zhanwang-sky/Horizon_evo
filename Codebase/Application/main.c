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
TimerHandle_t xTimer_blink;
char uartTxBuf[128] = { '\0' };

/* Functions -----------------------------------------------------------------*/
void init_mpu(void) {
    mpu_reset();
    mpu_set_gyro_fsr(2000);
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

void tPrintHello(void *pvParameters) {
    uint32_t count = 0;

    init_mpu();

    snprintf(uartTxBuf, sizeof(uartTxBuf), "\033c");
    al_uart_write(0, uartTxBuf, strlen(uartTxBuf));

    while (1) {
        xSemaphoreTake(xSem_mpu, portMAX_DELAY);
        count++;
        snprintf(uartTxBuf, sizeof(uartTxBuf), "%4u: Hello world!\r\n", count);
        al_uart_write(0, uartTxBuf, strlen(uartTxBuf));
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

    /* Create a FreeRTOS timer */
    xTimer_blink = xTimerCreate("tBlinkLED",
                                pdMS_TO_TICKS(1000),
                                pdTRUE,
                                NULL,
                                tBlinkLED);
    xTimerStart(xTimer_blink, 0);

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
void al_exti_0(void) {
    static uint32_t count = 0;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        count += 1;
        if (count >= 200) {
            count = 0;
            xSemaphoreGiveFromISR(xSem_mpu, NULL);
        }
    }

    return;
}

/******************************** END OF FILE *********************************/
