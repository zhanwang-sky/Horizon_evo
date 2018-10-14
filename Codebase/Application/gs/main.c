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

#include "bmp280.h"

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinkLED;
char TxBuf[80] = { '\0' };
// test
#define N (150)
int32_t temp32[N];
uint32_t pres32[N];
int64_t temp_sum;
uint64_t pres_sum;

/* Functions -----------------------------------------------------------------*/
/* Adapter */
int8_t bmp_i2c_read(uint8_t dev_id, uint8_t reg_addr,
        uint8_t *data, uint16_t len) {
    return (int8_t) al_i2c_read(0, dev_id << 1, reg_addr, data, len);
}

int8_t bmp_i2c_write(uint8_t dev_id, uint8_t reg_addr,
        uint8_t *data, uint16_t len) {
    return (int8_t) al_i2c_write(0, dev_id << 1, reg_addr, data, len);
}

void bmp_delay_ms(uint32_t period) {
    vTaskDelay(pdMS_TO_TICKS(period));
    return;
}

/* Threads */
void tBlinkLED(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tPrintHello(void *pvParameters) {
    int len;
    TickType_t xLastWakeTime;
    // bmp280
    int8_t rslt;
    struct bmp280_dev bmp;
    struct bmp280_config conf;
    struct bmp280_uncomp_data ucomp_data;
    uint8_t meas_dur;
    int i, j;
    char of = 0;

    /*  */
    len = snprintf(TxBuf, sizeof(TxBuf), "\033c\033[2Jinitializing...\r\n");
    al_uart_write(0, (uint8_t *) TxBuf, len);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Initialize BMP280 */
    do {
        /* Sensor interface over I2C with secondary slave address */
        bmp.dev_id = BMP280_I2C_ADDR_SEC;
        bmp.intf = BMP280_I2C_INTF;
        bmp.read = bmp_i2c_read;
        bmp.write = bmp_i2c_write;
        bmp.delay_ms = bmp_delay_ms;
        rslt = bmp280_init(&bmp);
        configASSERT(0 == rslt);

        /* Always read the current settings before writing, especially when
         * all the configuration is not modified.
         */
        rslt = bmp280_get_config(&conf, &bmp);
        configASSERT(0 == rslt);

        /* Overwrite the desired settings */
        conf.os_pres = BMP280_OS_16X;
        conf.os_temp = BMP280_OS_2X;
        conf.filter = BMP280_FILTER_COEFF_16;
        conf.odr = BMP280_ODR_0_5_MS;
        rslt = bmp280_set_config(&conf, &bmp);
        configASSERT(0 == rslt);

        /* Always set the power mode after setting the configuration */
        rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
        configASSERT(0 == rslt);

        meas_dur = bmp280_compute_meas_time(&bmp);
        vTaskDelay(pdMS_TO_TICKS(meas_dur));
    } while (0);

    i = 0;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        bmp280_get_uncomp_data(&ucomp_data, &bmp);
        temp32[i] = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
        pres32[i] = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp);
        i++;
        if (i >= N) {
            i = 0;
            if (of == 0) {
                of = 1;
                xTimerChangePeriod(xTimer_blinkLED, pdMS_TO_TICKS(50), NULL);
            }
        }
        if (i % 10 == 0) {
            for (temp_sum = 0, pres_sum = 0, j = 0; j < N; j++) {
                temp_sum += temp32[j];
                pres_sum += pres32[j];
            }
            if (of != 0) {
                temp_sum /= N;
                pres_sum /= N;
            } else {
                temp_sum /= i;
                pres_sum /= i;
            }
            len = snprintf(TxBuf, sizeof(TxBuf),
                "\rtemp: %lld, pres: %llu", temp_sum, pres_sum);
            al_uart_write(0, TxBuf, len);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
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

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        period = (period == 1000) ? 100 : 1000;
        xTimerChangePeriodFromISR(xTimer_blinkLED, pdMS_TO_TICKS(period), NULL);
    }
}

/******************************** END OF FILE *********************************/
