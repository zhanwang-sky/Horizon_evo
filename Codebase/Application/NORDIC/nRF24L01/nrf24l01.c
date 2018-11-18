/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @author  Ji Chen
  * @brief   nRF24L01 dirver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#if defined(HORIZON_MINI_L4) || defined(HORIZON_STD_L4) || defined(HORIZON_GS_STD_L4)
#include "al_stm32l4xx.h"
#else
#error please specify a target board
#endif

/* Private define ------------------------------------------------------------*/
#define NRF_SPI_FD (0)
#define NRF_SPI_SUBFD (0)

#define NRF_CE_FD (1)

/* nRF registers */
#define NRF_REG_CONFIG      (0x00)
#define NRF_REG_EN_AA       (0x01)
#define NRF_REG_EN_RXADDR   (0x02)
#define NRF_REG_SETUP_AW    (0x03)
#define NRF_REG_SETUP_RETR  (0x04)
#define NRF_REG_RF_CH       (0x05)
#define NRF_REG_RF_SETUP    (0x06)
#define NRF_REG_STATUS      (0x07)
#define NRF_REG_OBSERVE_TX  (0x08)
#define NRF_REG_RPD         (0x09)
#define NRF_REG_RX_ADDR_P0  (0x0A)
#define NRF_REG_RX_ADDR_P1  (0x0B)
#define NRF_REG_RX_ADDR_P2  (0x0C)
#define NRF_REG_RX_ADDR_P3  (0x0D)
#define NRF_REG_RX_ADDR_P4  (0x0E)
#define NRF_REG_RX_ADDR_P5  (0x0F)
#define NRF_REG_TX_ADDR     (0x10)
#define NRF_REG_RX_PW_P0    (0x11)
#define NRF_REG_RX_PW_P1    (0x12)
#define NRF_REG_RX_PW_P2    (0x13)
#define NRF_REG_RX_PW_P3    (0x14)
#define NRF_REG_RX_PW_P4    (0x15)
#define NRF_REG_RX_PW_P5    (0x16)
#define NRF_REG_FIFO_STATUS (0x17)
#define NRF_REG_DYNPD       (0x1C)
#define NRF_REG_FEATURE     (0x1D)

/* bits defined in reg CONFIG */
#define NRF_BIT_MASK_RX_DR  (0x40)
#define NRF_BIT_MASK_TX_DS  (0x20)
#define NRF_BIT_MASK_MAX_RT (0x10)
#define NRF_BIT_EN_CRC      (0x08)
#define NRF_BIT_CRCO        (0x04)
#define NRF_BIT_PWR_UP      (0x02)
#define NRF_BIT_PRIM_RX     (0x01)

/* bits defined in reg EN_AA */
#define NRF_BIT_ENAA_P5     (0x20)
#define NRF_BIT_ENAA_P4     (0x10)
#define NRF_BIT_ENAA_P3     (0x08)
#define NRF_BIT_ENAA_P2     (0x04)
#define NRF_BIT_ENAA_P1     (0x02)
#define NRF_BIT_ENAA_P0     (0x01)

/* bits defined in reg EN_RXADDR */
#define NRF_BIT_ERX_P5      (0x20)
#define NRF_BIT_ERX_P4      (0x10)
#define NRF_BIT_ERX_P3      (0x08)
#define NRF_BIT_ERX_P2      (0x04)
#define NRF_BIT_ERX_P1      (0x02)
#define NRF_BIT_ERX_P0      (0x01)

/* bits defined in reg SETUP_AW */
#define NRF_MSK_AW          (0x03)

/* bits defined in reg SETUP_RETR */
#define NRF_MSK_ARD         (0xF0)
#define NRF_MSK_ARC         (0x0F)

/* bits defined in reg RF_CH */
#define NRF_MSK_RF_CH       (0x7F)

/* bits defined in reg RF_SETUP */
#define NRF_BIT_CONT_WAVE   (0x80)
#define NRF_BIT_RF_DR_LOW   (0x20)
#define NRF_BIT_PLL_LOCK    (0x10)
#define NRF_BIT_RF_DR_HIGH  (0x08)
#define NRF_MSK_RF_PWR      (0x06)

/* bits defined in reg STATUS */
#define NRF_BIT_RX_DR       (0x40)
#define NRF_BIT_TX_DS       (0x20)
#define NRF_BIT_MAX_RT      (0x10)
#define NRF_MSK_RX_P_NO     (0x0E)
#define NRF_BIT_TX_FULL     (0x01)

/* bits defined in reg OBSERVE_TX */
#define NRF_MSK_PLOS_CNT    (0xF0)
#define NRF_MSK_ARC_CNT     (0x0F)

/* bits defined in reg RPD */
#define NRF_BIT_RPD         (0x01)

/* bits defined in reg RX_PW_P0 */
#define NRF_MSK_RX_PW_P0    (0x3F)

/* bits defined in reg RX_PW_P1 */
#define NRF_MSK_RX_PW_P1    (0x3F)

/* bits defined in reg RX_PW_P2 */
#define NRF_MSK_RX_PW_P2    (0x3F)

/* bits defined in reg RX_PW_P3 */
#define NRF_MSK_RX_PW_P3    (0x3F)

/* bits defined in reg RX_PW_P4 */
#define NRF_MSK_RX_PW_P4    (0x3F)

/* bits defined in reg RX_PW_P5 */
#define NRF_MSK_RX_PW_P5    (0x3F)

/* bits defined in reg FIFO_STATUS */
#define NRF_BIT_TX_REUSE    (0x40)
#define NRF_BIT_TX_EMPTY    (0x10)
#define NRF_BIT_RX_FULL     (0x02)
#define NRF_BIT_RX_EMPTY    (0x01)

/* bits defined in reg DYNPD */
#define NRF_BIT_DPL_P5      (0x20)
#define NRF_BIT_DPL_P4      (0x10)
#define NRF_BIT_DPL_P3      (0x08)
#define NRF_BIT_DPL_P2      (0x04)
#define NRF_BIT_DPL_P1      (0x02)
#define NRF_BIT_DPL_P0      (0x01)

/* bits defined in reg FEATURE */
#define NRF_BIT_EN_DPL      (0x04)
#define NRF_BIT_EN_ACK_PAY  (0x02)
#define NRF_BIT_EN_DYN_ACK  (0x01)

/* Private variables ---------------------------------------------------------*/
uint8_t _nrf_rbuf[33];
uint8_t _nrf_wbuf[33];

/* Functions -----------------------------------------------------------------*/
uint8_t _nrf_r_register(uint8_t addr, uint8_t *cmd, uint8_t len) {
    _nrf_wbuf[0] = 0x1F & addr;
    for (int i = 1; i <= len; i++) {
        _nrf_wbuf[i] = 0xFF;
    }
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, len + 1);
    memcpy(cmd, _nrf_rbuf + 1, len);
    return _nrf_rbuf[0];
}

uint8_t _nrf_w_register(uint8_t addr, const uint8_t *cmd, uint8_t len) {
    _nrf_wbuf[0] = 0x20 | (0x1F & addr);
    memcpy(_nrf_wbuf + 1, cmd, len);
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, len + 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_r_rx_payload(uint8_t *buf, uint8_t nbytes) {
    _nrf_wbuf[0] = 0x61;
    for (int i = 1; i <= nbytes; i++) {
        _nrf_wbuf[i] = 0xFF;
    }
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, nbytes + 1);
    memcpy(buf, _nrf_rbuf + 1, nbytes);
    return _nrf_rbuf[0];
}

uint8_t _nrf_w_tx_payload(const uint8_t *buf, uint8_t nbytes) {
    _nrf_wbuf[0] = 0xA0;
    memcpy(_nrf_wbuf + 1, buf, nbytes);
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, nbytes + 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_flush_tx(void) {
    _nrf_wbuf[0] = 0xE1;
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_flush_rx(void) {
    _nrf_wbuf[0] = 0xE2;
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_reuse_tx_pl(void) {
    _nrf_wbuf[0] = 0xE3;
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_r_rx_pl_wid(uint8_t *width) {
    _nrf_wbuf[0] = 0x60;
    _nrf_wbuf[1] = 0xFF;
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, 2);
    *width = _nrf_rbuf[1];
    return _nrf_rbuf[0];
}

uint8_t _nrf_w_ack_payload(uint8_t ppp, const uint8_t *buf, uint8_t nbytes) {
    _nrf_wbuf[0] = 0xA8 | (0x07 & ppp);
    memcpy(_nrf_wbuf + 1, buf, nbytes);
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, nbytes + 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_w_tx_payload_noack(const uint8_t *buf, uint8_t nbytes) {
    _nrf_wbuf[0] = 0xB0;
    memcpy(_nrf_wbuf + 1, buf, nbytes);
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, nbytes + 1);
    return _nrf_rbuf[0];
}

uint8_t _nrf_nop(void) {
    _nrf_wbuf[0] = 0xFF;
    al_spi_write_read(NRF_SPI_FD, NRF_SPI_SUBFD, _nrf_wbuf, _nrf_rbuf, 1);
    return _nrf_rbuf[0];
}

int nrf_power_up(int up) {
    uint8_t cmd;

    _nrf_r_register(NRF_REG_CONFIG, &cmd, 1);
    if (up) {
        cmd |= NRF_BIT_PWR_UP;
    } else {
        cmd &= ~NRF_BIT_PWR_UP;
    }
    _nrf_w_register(NRF_REG_CONFIG, &cmd, 1);

    return 0;
}

/******************************** END OF FILE *********************************/
