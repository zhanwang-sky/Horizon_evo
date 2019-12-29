/**
  ******************************************************************************
  * @file   al_stm32l4xx_uart.c
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with UART.
  ******************************************************************************
  * @attention
  * I. aio APIs should be used in task context.
  * II. however, handlers (callbacks) will be executed in interrupt context.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "al_stm32l4xx.h"

#if defined(HORIZON_MINI_L4)
#include "nucleo_l432kc_bsp_config.h"
#elif defined(HORIZON_STD_L4) || defined(HORIZON_GS_STD_L4)
#include "nucleo_l476rg_bsp_config.h"
#else
#error please specify a target board
#endif

/* Private variables ---------------------------------------------------------*/
SemaphoreHandle_t __al_uart_txBusLock[BSP_NR_UARTs];
volatile unsigned short __al_uart_rxBuf[BSP_NR_UARTs];
int (*__al_uart_tx_handler[BSP_NR_UARTs])(int);
int (*__al_uart_rx_handler[BSP_NR_UARTs])(unsigned short, int);

/* Functions -----------------------------------------------------------------*/
int al_uart_init(void) {
    for (int i = 0; i < BSP_NR_UARTs; i++) {
        if ((NULL == (__al_uart_txBusLock[i] = xSemaphoreCreateBinary()))
            // XXX release a semaphore before starting scheduler?
            || (xSemaphoreGive(__al_uart_txBusLock[i]) != pdTRUE)) {
            return -1;
        }
    }

    return 0;
}

int al_uart_aio_write(struct al_uart_aiocb *aiocb) {
    int fd = aiocb->aio_fildes;
    size_t size = aiocb->aio_nbytes;
    UART_HandleTypeDef *huart;

    /* sanity check */
    if (fd < 0 || fd >= BSP_NR_UARTs) {
        return -EBADF;
    }
    if (!aiocb->aio_buf) {
        return -EFAULT;
    }
    if (size == 0) {
        return 0;
    } else if (size > 0xFFFF) {
        size = 0xFFFF;
    }

    BSP_UART_FD2HDL(fd, huart);

    if (xSemaphoreTake(__al_uart_txBusLock[fd], (aiocb->aio_flag & AIO_NONBLOCK) ? 0 : portMAX_DELAY) != pdTRUE) {
        return -EAGAIN;
    }

    __al_uart_tx_handler[fd] = aiocb->handler;

    if (HAL_UART_Transmit_DMA(huart, (uint8_t*) aiocb->aio_buf, (uint16_t) size) != HAL_OK) {
        goto ERR_HAL;
    }

    return 0;

ERR_HAL:
    xSemaphoreGive(__al_uart_txBusLock[fd]);
    return -EBUSY;
}

int al_uart_async_read_one(int fd, int (*handler)(unsigned short, int)) {
    UART_HandleTypeDef *huart;

    /* sanity check */
    if (fd < 0 || fd >= BSP_NR_UARTs) {
        return -EBADF;
    }

    BSP_UART_FD2HDL(fd, huart);

    __al_uart_rx_handler[fd] = handler;

    if (HAL_UART_Receive_IT(huart, (uint8_t*) &__al_uart_rxBuf[fd], 1) != HAL_OK) {
        return -EBUSY;
    }

    return 0;
}

/* ISR callbacks -------------------------------------------------------------*/
/**
  * @brief  UART Tx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    int fd;
    BaseType_t xHigherPriorityTaskWoken;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_UART_HDL2FD(huart, fd);
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(__al_uart_txBusLock[fd], &xHigherPriorityTaskWoken);
        if (__al_uart_tx_handler[fd] && __al_uart_tx_handler[fd](huart->TxXferSize)) {
            xHigherPriorityTaskWoken = pdTRUE;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  UART Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    int fd;
    int rc;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_UART_HDL2FD(huart, fd);
        rc = (huart->ErrorCode != HAL_UART_ERROR_NONE) ? -EIO : 1;
        if (__al_uart_rx_handler[fd]
            && (!__al_uart_rx_handler[fd](__al_uart_rxBuf[fd], rc))) {
            HAL_UART_Receive_IT(huart, (uint8_t*) &__al_uart_rxBuf[fd], 1);
        }
    }
}

/**
  * @brief  UART error callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    int fd;
    BaseType_t xHigherPriorityTaskWoken;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* Only consider Tx error, Rx error is processed in RxCpltCallback */
        if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0) {
            /* Only DMA error may cause Tx fail */
            BSP_UART_HDL2FD(huart, fd);
            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(__al_uart_txBusLock[fd], &xHigherPriorityTaskWoken);
            if (__al_uart_tx_handler[fd] && __al_uart_tx_handler[fd](-EIO)) {
                xHigherPriorityTaskWoken = pdTRUE;
            }
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    huart->ErrorCode = HAL_UART_ERROR_NONE;
}

/******************************** END OF FILE *********************************/
