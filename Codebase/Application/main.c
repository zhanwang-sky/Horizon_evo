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
#define PSEUDO_SHELL_UART_FD 0
#define PSEUDO_SHELL_BUFF_LEN 123 // \r\n\0

/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimer_blinker;
char pseudoShellBuff[PSEUDO_SHELL_BUFF_LEN];
volatile char pseudoShellCmdBuff[2] = { '\0', '\n' };
SemaphoreHandle_t pseudoShellLineReady;

/* Functions -----------------------------------------------------------------*/
/* Threads */
void tBlinker(TimerHandle_t xTimer) {
    al_gpio_toggle_pin(0);
}

void tPseudoShell(void *pvParameters) {
    int len;
    // I2C test
    uint8_t data;

    /* initialize... */
    len = snprintf(pseudoShellBuff, sizeof(pseudoShellBuff), "\033c\033[2JInitializing...\r\n");
    al_uart_write(PSEUDO_SHELL_UART_FD, (uint8_t *) pseudoShellBuff, len);
    vTaskDelay(pdMS_TO_TICKS(100));

    // I2C test
    if (al_i2c_read(0, 0xD0, 0x75, &data, 1) < 0) {
        len = snprintf(pseudoShellBuff, sizeof(pseudoShellBuff), "I2C bus error!\r\n");
    } else {
        len = snprintf(pseudoShellBuff, sizeof(pseudoShellBuff), "data = 0x%X\r\n", data);
    }
    al_uart_write(PSEUDO_SHELL_UART_FD, (uint8_t *) pseudoShellBuff, len);

#if defined(HORIZON_MINI_L4)
    // dshort test
    len = snprintf(pseudoShellBuff, sizeof(pseudoShellBuff), "motors will start in 10s, get ready!\r\n");
    al_uart_write(PSEUDO_SHELL_UART_FD, (uint8_t *) pseudoShellBuff, len);
    vTaskDelay(pdMS_TO_TICKS(10000));
    al_tim_dshot_set(0, 200);
#endif

    al_uart_start_receiving(PSEUDO_SHELL_UART_FD);
    while (1) {
        xSemaphoreTake(pseudoShellLineReady, portMAX_DELAY);
        if (pseudoShellCmdBuff[0] == '\n') {
            // ignore LF
            continue;
        } else if (pseudoShellCmdBuff[0] == '\r') {
            al_uart_write(PSEUDO_SHELL_UART_FD, (void *) pseudoShellCmdBuff, 2);
        } else {
            al_uart_write(PSEUDO_SHELL_UART_FD, (void *) pseudoShellCmdBuff, 1);
        }
    }
}

int main(void) {
    /* BSP Init */
    BSP_MCU_Init();

    /* AL Init */
    al_uart_init();
    al_i2c_init();
    al_spi_init();
#if defined(HORIZON_MINI_L4)
    al_tim_dshot_init();
#endif

    /* Create semaphores */
    pseudoShellLineReady = xSemaphoreCreateBinary();

    /* Create software timers */
    xTimer_blinker = xTimerCreate("tBlinker",
                                  pdMS_TO_TICKS(1000),
                                  pdTRUE,
                                  NULL,
                                  tBlinker);
    /* Activate timers */
    xTimerStart(xTimer_blinker, 0);

    /* Create tasks */
    xTaskCreate(tPseudoShell,
                "tPseudoShell",
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

void al_uart_0_callback(int state, unsigned char data, int *brk) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        if (!state) {
            pseudoShellCmdBuff[0] = data;
            xSemaphoreGiveFromISR(pseudoShellLineReady, NULL);
        }
    }

    return;
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
