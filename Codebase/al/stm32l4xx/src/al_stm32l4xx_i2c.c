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

#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef HAL_StatusTypeDef(*_HAL_I2C_Mem_Xfer_DMA_t) \
    (I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, \
     uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

typedef struct _al_i2c_params {
    int index;
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint8_t *buf;
    uint16_t nbytes;
    _HAL_I2C_Mem_Xfer_DMA_t xfer;
} _al_i2c_params_t;

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t _al_i2c_mutex[BSP_NR_I2Cs] = { NULL };
static SemaphoreHandle_t _al_i2c_cpltSem[BSP_NR_I2Cs] = { NULL };

/* Functions -----------------------------------------------------------------*/
int al_i2c_init(void) {
    for (int i = 0; i < BSP_NR_I2Cs; i++) {
        if (NULL == (_al_i2c_mutex[i] = xSemaphoreCreateMutex())) {
            return -1;
        }
        if (NULL == (_al_i2c_cpltSem[i] = xSemaphoreCreateBinary())) {
            return -1;
        }
    }

    return 0;
}

int _al_i2c_xfer(_al_i2c_params_t *params) {
    int rc = 0;

    xSemaphoreTake(_al_i2c_mutex[params->index], portMAX_DELAY);

    if (params->xfer(params->hi2c, params->dev_addr, params->reg_addr,
        I2C_MEMADD_SIZE_8BIT, params->buf, params->nbytes) != HAL_OK) {
        rc = -1;
        goto EXIT;
    }

    xSemaphoreTake(_al_i2c_cpltSem[params->index], portMAX_DELAY);

    if (params->hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
        rc = -1;
    }

EXIT:
    xSemaphoreGive(_al_i2c_mutex[params->index]);

    return rc;
}

int al_i2c_write(int fd, char dev_addr, char reg_addr, const void *buf, unsigned int nbytes) {
    _al_i2c_params_t params;

    if ((fd < 0 || fd >= BSP_NR_I2Cs)
        || (NULL == buf)
        || (0 == nbytes || nbytes > 0xFFFF)) {
        return -1;
    }

    memset(&params, 0, sizeof(params));
    BSP_I2C_FD2IDXHDL(fd, params.index, params.hi2c);
    params.dev_addr = dev_addr;
    params.reg_addr = reg_addr;
    params.buf = (uint8_t *) buf;
    params.nbytes = nbytes;
    params.xfer = HAL_I2C_Mem_Write_DMA;

    return _al_i2c_xfer(&params);
}

int al_i2c_read(int fd, char dev_addr, char reg_addr, void *buf, unsigned int nbytes) {
    _al_i2c_params_t params;

    if ((fd < 0 || fd >= BSP_NR_I2Cs)
        || (NULL == buf)
        || (0 == nbytes || nbytes > 0xFFFF)) {
        return -1;
    }

    memset(&params, 0, sizeof(params));
    BSP_I2C_FD2IDXHDL(fd, params.index, params.hi2c);
    params.dev_addr = dev_addr;
    params.reg_addr = reg_addr;
    params.buf = buf;
    params.nbytes = nbytes;
    params.xfer = HAL_I2C_Mem_Read_DMA;

    return _al_i2c_xfer(&params);
}

/* ISR callbacks -------------------------------------------------------------*/
static void _al_i2c_xferCpltCallback(I2C_HandleTypeDef *hi2c) {
    int index;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_I2C_HDL2IDX(hi2c, index);
        xSemaphoreGiveFromISR(_al_i2c_cpltSem[index], NULL);
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
