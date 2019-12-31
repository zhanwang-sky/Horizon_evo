/**
  ******************************************************************************
  * @file   al_stm32l4xx_i2c.c
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with I2C.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

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

/* Private typedef -----------------------------------------------------------*/
typedef HAL_StatusTypeDef __al_i2c_xFunc_t(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t);

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t __al_i2c_busLock[BSP_NR_I2Cs];
static int (*__al_i2c_handler[BSP_NR_I2Cs])(int);

/* Private functions ---------------------------------------------------------*/
static int __al_i2c_xfer(struct al_i2c_aiocb *aiocb, __al_i2c_xFunc_t *xFunc) {
    int fd = aiocb->aio_fildes;
    size_t size = aiocb->aio_nbytes;
    I2C_HandleTypeDef *hi2c;
    HAL_StatusTypeDef hal_rc;

    /* sanity check */
    if (fd < 0 || fd >= BSP_NR_I2Cs) {
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

    BSP_I2C_FD2HDL(fd, hi2c);

    if (xSemaphoreTake(__al_i2c_busLock[fd],
        (aiocb->aio_flag & AIO_NONBLOCK) ? 0 : portMAX_DELAY) != pdTRUE) {
        return -EAGAIN;
    }

    __al_i2c_handler[fd] = aiocb->aio_handler;

    hal_rc = xFunc(hi2c,
        aiocb->aio_devadd, aiocb->aio_memadd,
        (aiocb->aio_memadd > 0xFF) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT,
        (uint8_t*) aiocb->aio_buf,
        size);
    if (hal_rc != HAL_OK) {
        goto ERR_HAL;
    }

    return 0;

ERR_HAL:
    xSemaphoreGive(__al_i2c_busLock[fd]);

    return (hal_rc == HAL_BUSY) ? -EBUSY : -EIO;

}

static void __al_i2c_xfer_handler(I2C_HandleTypeDef *hi2c, int ec) {
    int fd;
    BaseType_t xHigherPriorityTaskWoken;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_I2C_HDL2FD(hi2c, fd);
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(__al_i2c_busLock[fd], &xHigherPriorityTaskWoken);
        if (__al_i2c_handler[fd] && __al_i2c_handler[fd](ec)) {
            xHigherPriorityTaskWoken = pdTRUE;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Functions -----------------------------------------------------------------*/
int al_i2c_init(void) {
    for (int i = 0; i < BSP_NR_I2Cs; i++) {
        if ((NULL == (__al_i2c_busLock[i] = xSemaphoreCreateBinary()))
            // XXX release a semaphore before starting scheduler?
            || (xSemaphoreGive(__al_i2c_busLock[i]) != pdTRUE)) {
            return -1;
        }
    }

    return 0;
}

int al_i2c_aio_write(struct al_i2c_aiocb *aiocb) {
    return __al_i2c_xfer(aiocb, HAL_I2C_Mem_Write_DMA);
}

int al_i2c_aio_read(struct al_i2c_aiocb *aiocb) {
    return __al_i2c_xfer(aiocb, HAL_I2C_Mem_Read_DMA);
}

/* ISR callbacks -------------------------------------------------------------*/
/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    __al_i2c_xfer_handler(hi2c, 0);
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    __al_i2c_xfer_handler(hi2c, 0);
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    __al_i2c_xfer_handler(hi2c, -EIO);
}

/**
  * @brief  I2C abort callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c) {
    __al_i2c_xfer_handler(hi2c, -EIO);
}

/******************************** END OF FILE *********************************/
