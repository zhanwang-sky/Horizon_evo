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

#include "stm32l4xx_hal.h"
#include "nucleo_l476rg_bsp.h"

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
char TxBuf[80] = { '\0' };

/* Functions -----------------------------------------------------------------*/
int main(void) {
    uint32_t n = 0;
    int len;

    BSP_MCU_Init();

    len = snprintf(TxBuf, sizeof(TxBuf), "\033c");
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) TxBuf, len);
    HAL_Delay(100);

    /* If all is well, the scheduler will now be running, and the following line
       will never be reached.
       If the following line does execute, then there was insufficient FreeRTOS
       heap memory available for the idle and/or timer tasks to be created.
       See the memory management section on the FreeRTOS web site for more
       details. */
    while(1) {
        len = snprintf(TxBuf, sizeof(TxBuf), "n = %04u\r\n", n++);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *) TxBuf, len);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(1000);
    }
}

/* ISR callbacks -------------------------------------------------------------*/

/******************************** END OF FILE *********************************/
