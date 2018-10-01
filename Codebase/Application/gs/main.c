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
extern I2C_HandleTypeDef hi2c1;
char TxBuf[80] = { '\0' };

/* Functions -----------------------------------------------------------------*/
int main(void) {
    uint32_t n = 0;
    uint8_t data[2];
    int len;

    BSP_MCU_Init();

    len = snprintf(TxBuf, sizeof(TxBuf), "\033c");
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) TxBuf, len);
    HAL_I2C_Mem_Read_DMA(&hi2c1, 0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT, data, 1);
    HAL_Delay(10);
    HAL_I2C_Mem_Read_DMA(&hi2c1, 0xD0, 0x75, I2C_MEMADD_SIZE_8BIT, data + 1, 1);
    HAL_Delay(90);

    /* If all is well, the scheduler will now be running, and the following line
       will never be reached.
       If the following line does execute, then there was insufficient FreeRTOS
       heap memory available for the idle and/or timer tasks to be created.
       See the memory management section on the FreeRTOS web site for more
       details. */
    while(1) {
        len = snprintf(TxBuf, sizeof(TxBuf), "n: %04u, PWR: %02x, WHOAMI: %02x\r\n", n++, data[0], data[1]);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *) TxBuf, len);
        HAL_Delay(1000);
    }
}

/* ISR callbacks -------------------------------------------------------------*/
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_PIN_13 == GPIO_Pin) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

/******************************** END OF FILE *********************************/
