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

#include "icm20648.h"

/* Definitions ---------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/
#define LOG(format, ...) \
do { \
    extern char msgBuf[]; \
    int len; \
    len = snprintf(msgBuf, sizeof(msgBuf), format, ##__VA_ARGS__); \
    al_uart_write(1, msgBuf, len); \
} while (0)

/* Global variables ----------------------------------------------------------*/
char msgBuf[121];
TimerHandle_t xTimerBlinker;
volatile unsigned int initialized = 0;
volatile unsigned int hits = 0;

/* Functions -----------------------------------------------------------------*/
int inv_icm_initializer(void) {
    int rc;

    /* Initialize ICM20x48 */
    // 0. wait to device power-up
    vTaskDelay(pdMS_TO_TICKS(100));
    // 1. reset and wakeup
    rc |= inv_icm_soft_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    rc |= inv_icm_wakeup();
    vTaskDelay(pdMS_TO_TICKS(100));
    // 2. enable ODR align
    rc |= inv_icm_enable_odr_align();
    // 3. set gyro sample rate & full scale
    rc |= inv_icm_set_gyro_smplrt(562);
    rc |= inv_icm_set_gfs(MPU_FS_2000DPS);
    // 4. set accelerator sample rate & full scale
    rc |= inv_icm_set_accel_smplrt(562);
    rc |= inv_icm_set_afs(MPU_FS_16G);
    // 5. enter duty-cycle mode
    rc |= inv_icm_set_lp_mode(MPU_LP_DUTY_CYCLE);
    // since enabled LPF, needs to wait the old data flushed out
    vTaskDelay(pdMS_TO_TICKS(100));
    rc |= inv_icm_enable_int();

    return rc;
}

/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    static int count = 0;
    unsigned int values[4];

    al_gpio_toggle_pin(0); // non-blocking API

    if (count < 15) {
        count++;
        if (count == 10) {
            xTimerChangePeriod(xTimerBlinker, pdMS_TO_TICKS(100), 0);
        } else if (count > 10) {
            for (int i = 0; i < 4; i++) {
                values[i] = (count - 10) * 200;
            }
            al_tim_dshot_set4(values); // non-block API
        }
    }
}

void tComm(void *pvParameters) {
    TickType_t xLastWakeTime;
    int sec = 0;
    int rc;

    LOG("initializing icm20648...\r\n");
    rc = inv_icm_initializer();
    LOG("rc = %d\r\n", rc);

    initialized = 1;
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        LOG("sec = %d, hits = %d\r\n", ++sec, hits);
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
    xTaskCreate(tComm,
                "thread communication",
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

void al_exti_0_callback(void) {
    if (initialized) {
        hits++;
    }
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
