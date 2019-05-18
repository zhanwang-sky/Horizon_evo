/**
  ******************************************************************************
  * @file    pni_rm3100.c
  * @author  Ji Chen
  * @brief   RM3100 sensor dirver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "al_stm32l4xx.h"
#include "main.h"
#include "pni_rm3100.h"

/* Private define ------------------------------------------------------------*/
#define PNI_RM3100_READ_PREFIX (0x80)
#define PNI_RM3100_REG_POLL   (0x00)
#define PNI_RM3100_REG_CMM    (0x01)
#define PNI_RM3100_REG_CCX    (0x04)
#define PNI_RM3100_REG_CCY    (0x06)
#define PNI_RM3100_REG_CCZ    (0x08)
#define PNI_RM3100_REG_TMRC   (0x0B)
#define PNI_RM3100_REG_MX     (0x24)
#define PNI_RM3100_REG_MY     (0x27)
#define PNI_RM3100_REG_MZ     (0x2A)
#define PNI_RM3100_REG_BIST   (0x33)
#define PNI_RM3100_REG_STATUS (0x34)
#define PNI_RM3100_REG_HSHAKE (0x35)
#define PNI_RM3100_REG_REVID  (0x36)
#define PNI_RM3100_MAX_BUFF_LEN (11)
// REG |  CMD   | xxx
// --- | STATUS | DATA

/* Private variables ---------------------------------------------------------*/
uint8_t pni_spi_buffer[PNI_RM3100_MAX_BUFF_LEN] = { 0 };

/* Functions -----------------------------------------------------------------*/
int pni_init(int cycle_count) {
    int rc;

    if (cycle_count < 0 || cycle_count > 65535) {
        return -1;
    }

    // 1. reset CCM (stop continuous mode)
    pni_spi_buffer[0] = PNI_RM3100_REG_CMM;
    pni_spi_buffer[1] = 0;
    if ((rc = al_spi_write(PNI_RM3100_SPI_FD, PNI_RM3100_SPI_SUB_FD,
                pni_spi_buffer, 2)) < 0) {
        return rc;
    }

    // 2. configure the cycle count
    pni_spi_buffer[0] = PNI_RM3100_REG_CCX;
    for (int i = 0; i < 3; i++) {
        pni_spi_buffer[(2 * i) + 1] = (cycle_count >> 8) & 0xFF;
        pni_spi_buffer[(2 * i) + 2] = cycle_count & 0xFF;
    }
    if ((rc = al_spi_write(PNI_RM3100_SPI_FD, PNI_RM3100_SPI_SUB_FD,
                pni_spi_buffer, 7)) < 0) {
        return rc;
    }

    // 3. reset TMRC
    pni_spi_buffer[0] = PNI_RM3100_REG_TMRC;
    pni_spi_buffer[1] = 0x96;
    if ((rc = al_spi_write(PNI_RM3100_SPI_FD, PNI_RM3100_SPI_SUB_FD,
                pni_spi_buffer, 2)) < 0) {
        return rc;
    }

    return 0;
}

int pni_measurement_once(void) {
    pni_spi_buffer[0] = PNI_RM3100_REG_POLL;
    pni_spi_buffer[1] = 0x70;
    return al_spi_write(PNI_RM3100_SPI_FD, PNI_RM3100_SPI_SUB_FD,
            pni_spi_buffer, 2);
}

int pni_read_data(int mag[3]) {
    int rc;

    pni_spi_buffer[0] = PNI_RM3100_READ_PREFIX | PNI_RM3100_REG_MX;
    if ((rc = al_spi_write_read(PNI_RM3100_SPI_FD, PNI_RM3100_SPI_SUB_FD,
                pni_spi_buffer, pni_spi_buffer + 1, 10)) < 0) {
        return rc;
    }

    for (int i = 0; i < 3; i++) {
        mag[i] = pni_spi_buffer[(3 * i) + 2] << 16
                 | pni_spi_buffer[(3 * i) + 3] << 8
                 | pni_spi_buffer[(3 * i) + 4];
        if (mag[i] & 0x800000) {
            mag[i] |= 0xFF000000;
        }
    }

    return 0;
}

/******************************** END OF FILE *********************************/
