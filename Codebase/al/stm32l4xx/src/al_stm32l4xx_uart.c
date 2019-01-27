/**
  ******************************************************************************
  * @file   al_stm32l4xx_uart.c
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with UART.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#if defined(HORIZON_MINI_L4)
#include "nucleo_l432kc_bsp_config.h"
#elif defined(HORIZON_STD_L4) || defined(HORIZON_GS_STD_L4)
#include "nucleo_l476rg_bsp_config.h"
#else
#error please specify a target board
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t _al_uart_txMutex[BSP_NR_UARTs];
SemaphoreHandle_t _al_uart_txCpltSem[BSP_NR_UARTs];

/* Functions -----------------------------------------------------------------*/
int al_uart_init(void) {
    for (int i = 0; i < BSP_NR_UARTs; i++) {
        if (NULL == (_al_uart_txMutex[i] = xSemaphoreCreateMutex())) {
            return -1;
        }
        if (NULL == (_al_uart_txCpltSem[i] = xSemaphoreCreateBinary())) {
            return -1;
        }
    }

    return 0;
}

int al_uart_write(int fd, const void *buf, unsigned int nbytes) {
    UART_HandleTypeDef *huart;
    int index;
    int rc = nbytes;

    if (fd < 0 || fd >= BSP_NR_UARTs || NULL == buf || nbytes > 0xFFFF) {
        return -1;
    }

    BSP_UART_FD2IDXHDL(fd, index, huart);

    xSemaphoreTake(_al_uart_txMutex[index], portMAX_DELAY);

    if (HAL_UART_Transmit_DMA(huart, (uint8_t *) buf, (uint16_t) nbytes) != HAL_OK) {
        rc = -1;
        goto EXIT;
    }

    xSemaphoreTake(_al_uart_txCpltSem[index], portMAX_DELAY);

    if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        rc = -1;
    }

EXIT:
    xSemaphoreGive(_al_uart_txMutex[index]);

    return rc;
}

/* ISR callbacks -------------------------------------------------------------*/
/**
  * @brief  UART Tx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    int index;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_UART_HDL2IDX(huart, index);
        xSemaphoreGiveFromISR(_al_uart_txCpltSem[index], NULL);
    }
}

/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    int index;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_UART_HDL2IDX(huart, index);
        xSemaphoreGiveFromISR(_al_uart_txCpltSem[index], NULL);
    }
}

/******************************** END OF FILE *********************************/
