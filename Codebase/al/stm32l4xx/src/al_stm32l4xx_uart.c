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
volatile int8_t _al_uart_txErr[BSP_NR_UARTs];
uint8_t _al_uart_rxBuf[BSP_NR_UARTs];

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

    _al_uart_txErr[index] = 0;
    if (HAL_UART_Transmit_DMA(huart, (uint8_t *) buf, (uint16_t) nbytes) != HAL_OK) {
        rc = -1;
        goto EXIT;
    }

    xSemaphoreTake(_al_uart_txCpltSem[index], portMAX_DELAY);

    if (_al_uart_txErr[index] != 0) {
        rc = -1;
    }

EXIT:
    xSemaphoreGive(_al_uart_txMutex[index]);

    return rc;
}

int al_uart_start_receiving(int fd) {
    UART_HandleTypeDef *huart;
    int index;
    HAL_StatusTypeDef rc;

    if (fd < 0 || fd >= BSP_NR_UARTs) {
        return -1;
    }

    BSP_UART_FD2IDXHDL(fd, index, huart);

    // TODO: Mutex

    rc = HAL_UART_Receive_IT(huart, &_al_uart_rxBuf[index], 1);
    /* It's okay if receiving is already in progress. */
    if (rc != HAL_OK && rc != HAL_BUSY) {
        return -1;
    }

    return 0;
}

__weak void al_uart_0_recv_callback(unsigned char data, int err, int *brk) {
    return;
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
  * @brief  UART Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    int index;
    int brk = 0;

    BSP_UART_HDL2IDX(huart, index);
    if (0 == index) {
        al_uart_0_recv_callback(_al_uart_rxBuf[index], 0, &brk);
    }
    if (!brk) {
        HAL_UART_Receive_IT(huart, &_al_uart_rxBuf[index], 1);
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
        /* Only DMA error may cause Tx fail */
        if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0) {
            huart->ErrorCode = HAL_UART_ERROR_NONE; // TODO: is it necessary?
            _al_uart_txErr[index] = 1;
            xSemaphoreGiveFromISR(_al_uart_txCpltSem[index], NULL);
        } else {
            /* DMA error and Rx error should not occur simultaneously,
               so, if not a DMA error, it must be an Rx error. */
            if (0 == index) {
                al_uart_0_recv_callback(0, 1, NULL);
            }
            // TODO: abort receiving?
        }
    }
}

/******************************** END OF FILE *********************************/
