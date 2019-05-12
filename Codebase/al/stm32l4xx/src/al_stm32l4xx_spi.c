/**
  ******************************************************************************
  * @file   al_stm32l4xx_spi.c
  * @author Ji Chen
  * @brief  Provide a set of APIs to interface with SPI.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if defined(HORIZON_MINI_L4)
#include "nucleo_l432kc_bsp_config.h"
#elif defined(HORIZON_STD_L4) || defined(HORIZON_GS_STD_L4)
#include "nucleo_l476rg_bsp_config.h"
#else
#error please specify a target board
#endif

/* Private typedef -----------------------------------------------------------*/
typedef struct _al_spi_nss {
    GPIO_TypeDef *port;
    uint16_t pin;
} _al_spi_nss_t;

typedef struct _al_spi_params {
    int index;
    _al_spi_nss_t nss;
    SPI_HandleTypeDef *hspi;
    uint8_t *txbuf;
    uint8_t *rxbuf;
    uint16_t nbytes;
} _al_spi_params_t;

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t _al_spi_mutex[BSP_NR_SPIs] = { NULL };
static SemaphoreHandle_t _al_spi_cpltSem[BSP_NR_SPIs] = { NULL };
static volatile _al_spi_nss_t _al_spi_nss_info[BSP_NR_SPIs];

/* Functions -----------------------------------------------------------------*/
int al_spi_init(void) {
    for (int i = 0; i < BSP_NR_SPIs; i++) {
        if (NULL == (_al_spi_mutex[i] = xSemaphoreCreateMutex())) {
            return -1;
        }
        if (NULL == (_al_spi_cpltSem[i] = xSemaphoreCreateBinary())) {
            return -1;
        }
    }

    return 0;
}

int _al_spi_xfer(_al_spi_params_t *params) {
    int rc = 0;
    HAL_StatusTypeDef hal_rc;

    xSemaphoreTake(_al_spi_mutex[params->index], portMAX_DELAY);

    _al_spi_nss_info[params->index].port = params->nss.port;
    _al_spi_nss_info[params->index].pin = params->nss.pin;

    /* ---------- enter critical area ---------- */
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(params->nss.port, params->nss.pin, GPIO_PIN_RESET);
    if (params->txbuf != NULL && params->rxbuf != NULL) {
        hal_rc = HAL_SPI_TransmitReceive_DMA(params->hspi,
                                             params->txbuf,
                                             params->rxbuf,
                                             params->nbytes);
    } else if (params->txbuf != NULL) {
        hal_rc = HAL_SPI_Transmit_DMA(params->hspi,
                                      params->txbuf,
                                      params->nbytes);
    } else {
        hal_rc = HAL_SPI_Receive_DMA(params->hspi,
                                     params->rxbuf,
                                     params->nbytes);
    }
    if (hal_rc != HAL_OK) {
        HAL_GPIO_WritePin(params->nss.port, params->nss.pin, GPIO_PIN_SET);
        rc = -1;
    }
    taskEXIT_CRITICAL();
    /* ---------- exit critical area ---------- */

    if (rc < 0) {
        goto EXIT;
    }

    xSemaphoreTake(_al_spi_cpltSem[params->index], portMAX_DELAY);

    if (params->hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
        rc = -1;
    }

EXIT:
    xSemaphoreGive(_al_spi_mutex[params->index]);
    return rc;
}

int al_spi_write(int fd, int subfd, const void *buf, unsigned int nbytes) {
    _al_spi_params_t params;

    if (!BSP_SPI_IS_VALID_FD(fd, subfd)
        || (NULL == buf)
        || (0 == nbytes || nbytes > 0xFFFF)) {
        return -1;
    }

    memset(&params, 0, sizeof(params));
    BSP_SPI_FD2IDXHDL(fd, params.index, params.hspi);
    BSP_SPI_SUBFD2PORTPIN(subfd, params.nss.port, params.nss.pin);
    params.txbuf = (uint8_t *) buf;
    params.rxbuf = NULL;
    params.nbytes = nbytes;

    return _al_spi_xfer(&params);
}

int al_spi_read(int fd, int subfd, void *buf, unsigned int nbytes) {
    _al_spi_params_t params;

    if (!BSP_SPI_IS_VALID_FD(fd, subfd)
        || (NULL == buf)
        || (0 == nbytes || nbytes > 0xFFFF)) {
        return -1;
    }

    memset(&params, 0, sizeof(params));
    BSP_SPI_FD2IDXHDL(fd, params.index, params.hspi);
    BSP_SPI_SUBFD2PORTPIN(subfd, params.nss.port, params.nss.pin);
    params.txbuf = NULL;
    params.rxbuf = (uint8_t *) buf;
    params.nbytes = nbytes;

    return _al_spi_xfer(&params);
}

int al_spi_write_read(int fd, int subfd, const void *txbuf, void *rxbuf, unsigned int nbytes) {
    _al_spi_params_t params;

    if (!BSP_SPI_IS_VALID_FD(fd, subfd)
        || (NULL == txbuf)
        || (NULL == rxbuf)
        || (0 == nbytes || nbytes > 0xFFFF)) {
        return -1;
    }

    memset(&params, 0, sizeof(params));
    BSP_SPI_FD2IDXHDL(fd, params.index, params.hspi);
    BSP_SPI_SUBFD2PORTPIN(subfd, params.nss.port, params.nss.pin);
    params.txbuf = (uint8_t *) txbuf;
    params.rxbuf = (uint8_t *) rxbuf;
    params.nbytes = nbytes;

    return _al_spi_xfer(&params);
}

/* ISR callbacks -------------------------------------------------------------*/
void _al_spi_xferCpltCallback(SPI_HandleTypeDef *hspi) {
    int index;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BSP_SPI_HDL2IDX(hspi, index);
        HAL_GPIO_WritePin(_al_spi_nss_info[index].port, _al_spi_nss_info[index].pin, GPIO_PIN_SET);
        xSemaphoreGiveFromISR(_al_spi_cpltSem[index], NULL);
    }
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    _al_spi_xferCpltCallback(hspi);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    _al_spi_xferCpltCallback(hspi);
}

/**
  * @brief  Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    _al_spi_xferCpltCallback(hspi);
}

/**
  * @brief  SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    _al_spi_xferCpltCallback(hspi);
}

/******************************** END OF FILE *********************************/
