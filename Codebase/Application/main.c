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

/* Global variables ----------------------------------------------------------*/
SemaphoreHandle_t xSemDRDY;
SemaphoreHandle_t xSemInitialized;
SemaphoreHandle_t xMutRaw6;
TimerHandle_t xTimerBlinker;
volatile int g_icm_initialized;
short g_raw6[6];
unsigned g_hits;

/* Functions -----------------------------------------------------------------*/
int inv_icm_init(const short gyro_offs[3]) {
    int rc;

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
    // 5. set gyro offset
    rc |= inv_icm_set_gyro_offs(gyro_offs);
    // 6. enable interrupt
    vTaskDelay(pdMS_TO_TICKS(100));
    rc |= inv_icm_enable_int();

    return rc;
}

/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    static int count = 0;
    unsigned int values[4];

    if (count < 15) {
        count++;
        if (count == 10) {
            BSP_SYSLED_Set_Normal();
        } else if (count > 10) {
            for (int i = 0; i < 4; i++) {
                values[i] = (count - 10) * 200;
            }
            al_tim_dshot_set4(values); // non-block API
        }
    }
}

void tComm(void *pvParameters) {
    static char msgBuf[121];
    static int len;
#define LOG(format, ...) \
    do { \
        len = snprintf(msgBuf, sizeof(msgBuf), format, ##__VA_ARGS__); \
        al_uart_write(1, msgBuf, len); \
    } while (0)

    TickType_t xLastWakeTime;
    short raw6[6];
    unsigned hits;
    unsigned count = 0;

    // waiting for ICM20648 initialization
    xSemaphoreTake(xSemInitialized, portMAX_DELAY);
    // start 1HZ loop
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        xSemaphoreTake(xMutRaw6, portMAX_DELAY);
        memcpy(raw6, g_raw6, sizeof(raw6));
        memset(g_raw6, 0, sizeof(g_raw6));
        hits = g_hits;
        xSemaphoreGive(xMutRaw6);
        LOG("count: %u, hits %u, gyro_sum: %d, %d, %d\r\n",
            ++count, hits, raw6[3], raw6[4], raw6[5]);
    }

#undef LOG
}

void tIMUCollector(void *pvParameters) {
    static const short gyro_offset[3] = { -47, 8, 6 };
    static short raw6[6];
    int rc;

    // waiting for IMC20648 powered up
    vTaskDelay(pdMS_TO_TICKS(100));

    rc = inv_icm_init(gyro_offset);
    if (rc < 0) {
        return;
    }

    g_icm_initialized = 1;
    xSemaphoreGive(xSemInitialized);

    while (xSemaphoreTake(xSemDRDY, pdMS_TO_TICKS(3)) == pdTRUE) {
        inv_icm_read_raw6(raw6);
        xSemaphoreTake(xMutRaw6, portMAX_DELAY);
        for (int i = 0; i < 3; i++) {
            g_raw6[i + 3] += raw6[i + 3];
        }
        g_hits++;
        xSemaphoreGive(xMutRaw6);
    }

    return;
}

int main(void) {
    /* BSP Init */
    BSP_MCU_Init();

    /* AL Init */
    al_uart_init();
    al_i2c_init();
    al_tim_dshot_init();

    /* Create semaphores */
    xSemDRDY = xSemaphoreCreateBinary();
    xSemInitialized = xSemaphoreCreateBinary();
    xMutRaw6 = xSemaphoreCreateMutex();

    /* Create tasks */
    xTaskCreate(tComm,
                "thread communication",
                configMINIMAL_STACK_SIZE,
                NULL,
                configTIMER_TASK_PRIORITY + 1,
                NULL);

    xTaskCreate(tIMUCollector,
                "thread IMU collector",
                configMINIMAL_STACK_SIZE,
                NULL,
                configTIMER_TASK_PRIORITY + 2,
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
    if (g_icm_initialized) {
        xSemaphoreGiveFromISR(xSemDRDY, NULL);
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
