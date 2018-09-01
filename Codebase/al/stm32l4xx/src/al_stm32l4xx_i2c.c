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
#ifdef HORIZON_MINI_L4
#include "nucleo_l432kc_bsp_config.h"
#else
#error please specify a target board
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
typedef HAL_StatusTypeDef(*_HAL_I2C_Mem_Xfer_DMA_t) \
    (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t xSem_i2c[BSP_NR_I2Cs] = { NULL };
static SemaphoreHandle_t xSem_i2cCplt[BSP_NR_I2Cs] = { NULL };
//volatile static unsigned int uI2CBytesLeft[BSP_NR_I2Cs] = { 0 };
//volatile static _HAL_I2C_Mem_Xfer_DMA_t xI2CXfer = NULL;

/* Functions -----------------------------------------------------------------*/
static int _al_i2c_xfer(int index, I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t nBytes, _HAL_I2C_Mem_Xfer_DMA_t xfer) {
    int rc;

    xSemaphoreTake(xSem_i2c[index], portMAX_DELAY);
    //xI2CXfer = xfer;

    if (xfer(hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, buf, nBytes) != HAL_OK) {
        rc = -1;
        goto EXIT;
    }

    xSemaphoreTake(xSem_i2cCplt[index], portMAX_DELAY);

    if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
        rc = -1;
    }

    /* return the number of bytes already sent/received instead of -1 */
    //rc = nBytes - uI2CBytesLeft[index];

EXIT:
    //xI2CXfer = NULL;
    xSemaphoreGive(xSem_i2c[index]);

    return rc;
}

int al_i2c_init(void) {
    for (int i = 0; i < BSP_NR_I2Cs; i++) {
        if (NULL == (xSem_i2c[i] = xSemaphoreCreateMutex())) {
            return -1;
        }
        if (NULL == (xSem_i2cCplt[i] = xSemaphoreCreateBinary())) {
            return -1;
        }
    }

    return 0;
}

int al_i2c_write(int fd, char dev_addr, char reg_addr, const void *buf, unsigned int nbytes) {
    I2C_HandleTypeDef *hi2c;
    int index;

    if (fd < 0 || fd >= BSP_NR_I2Cs || NULL == buf || nbytes > 0xFFFF) {
        return -1;
    }

    BSP_I2C_FD2IDXHDL(fd, index, hi2c);

    return _al_i2c_xfer(index, hi2c, dev_addr, reg_addr, (uint8_t *) buf, (uint16_t) nbytes, HAL_I2C_Mem_Write_DMA);
}

int al_i2c_read(int fd, char dev_addr, char reg_addr, void *buf, unsigned int nbytes) {
    I2C_HandleTypeDef *hi2c;
    int index;

    if (fd < 0 || fd >= BSP_NR_I2Cs || NULL == buf || nbytes > 0xFFFF) {
        return -1;
    }

    BSP_I2C_FD2IDXHDL(fd, index, hi2c);

    return _al_i2c_xfer(index, hi2c, dev_addr, reg_addr, (uint8_t *) buf, (uint16_t) nbytes, HAL_I2C_Mem_Read_DMA);
}

/* ISR callbacks -------------------------------------------------------------*/
static void _al_i2c_xferCpltCallback(I2C_HandleTypeDef *hi2c) {
    int index;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_I2C_HDL2IDX(hi2c, index);
        /*
        if (xI2CXfer != NULL) {
            if (HAL_I2C_Mem_Write_DMA == xI2CXfer) {
                uI2CBytesLeft[index] = (unsigned int) hi2c->hdmatx->Instance->CNDTR;
            } else if (HAL_I2C_Mem_Read_DMA == xI2CXfer) {
                uI2CBytesLeft[index] = (unsigned int) hi2c->hdmarx->Instance->CNDTR;
            } else {
                uI2CBytesLeft[index] = 0;
            }
        }
        */
        xSemaphoreGiveFromISR(xSem_i2cCplt[index], NULL);
    }
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    _al_i2c_xferCpltCallback(hi2c);
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    _al_i2c_xferCpltCallback(hi2c);
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    _al_i2c_xferCpltCallback(hi2c);
}

/******************************** END OF FILE *********************************/
