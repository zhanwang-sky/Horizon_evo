/**
  ******************************************************************************
  * @file   al_stm32l4xx_tim_dshot.c
  * @author Ji Chen
  * @brief  Provide a set of APIs that implement PWM or D-Shot protocol.
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

#include <string.h>

/* Private define ------------------------------------------------------------*/
#define _AL_TIM_DSHOT_MIN_THR (48)
#define _AL_TIM_DSHOT_MAX_THR (2047)
#define _AL_TIM_DSHOT_INIT_PATTERN (0x606)

/* Private variables ---------------------------------------------------------*/
static uint32_t _al_tim_dshot_burst_buffer[3][BSP_NR_DSHOT_CHANNELs * 18] = { 0 };
static volatile int _al_tim_dshot_task_buff_id[BSP_NR_DSHOT_TIMERs] = { 0 };
static volatile int _al_tim_dshot_isr_buff_id[BSP_NR_DSHOT_TIMERs] = { 0 };

/* Functions -----------------------------------------------------------------*/
int _al_tim_dshot_cal_new_buff_id(int task_buff_id, int isr_buff_id) {
    int new_buff_id;

    if (task_buff_id == isr_buff_id) {
        new_buff_id = (task_buff_id + 1) % 3;
    } else {
        new_buff_id = 3 - (task_buff_id + isr_buff_id);
    }

    return new_buff_id;
}

uint16_t _al_tim_dshot_make_pattern(unsigned int value) {
    unsigned crc = 0;
    unsigned crc_data = 0;

    value += _AL_TIM_DSHOT_MIN_THR;
    value <<= 1;
    crc_data = value;
    for (int i = 0; i < 3; i++ ) {
        crc ^= crc_data;
        crc_data >>= 4;
    }

    return (uint16_t) ((value << 4) | (crc & 0xf));
}

void _al_tim_dshot_update_buffer(int buff_id, int ch_id, uint16_t pattern) {
    int id_in_tim;
    int nr_chs;

    BSP_TIM_DSHOT_CHID2IDINTIM(ch_id, id_in_tim);
    BSP_TIM_DSHOT_CHID2NRCHS(ch_id, nr_chs);
    for (int bit = 0; bit < 16; bit++) {
        _al_tim_dshot_burst_buffer[buff_id][(ch_id - id_in_tim) * 18 + bit * nr_chs + id_in_tim]
            = ((pattern << bit) & 0x8000) ? BSP_TIM_DSHOT_CODE1 : BSP_TIM_DSHOT_CODE0;
    }

    return;
}

void _al_tim_dshot_copy_buffer(int ch_id, int from_buff, int to_buff) {
    int id_in_tim;
    int nr_chs;
    size_t offset;
    size_t length;

    BSP_TIM_DSHOT_CHID2IDINTIM(ch_id, id_in_tim);
    BSP_TIM_DSHOT_CHID2NRCHS(ch_id, nr_chs);
    offset = (ch_id - id_in_tim) * 18;
    length = nr_chs * 18;
    memcpy(_al_tim_dshot_burst_buffer[to_buff] + offset, _al_tim_dshot_burst_buffer[from_buff] + offset, length * sizeof(uint32_t));

    return;
}

int al_tim_dshot_init(void) {
    TIM_HandleTypeDef *htim;
    uint32_t burstBaseAddress;
    uint32_t offset;
    uint32_t burstLength;

    for (int ch_id = 0; ch_id < BSP_NR_DSHOT_CHANNELs; ch_id++) {
        _al_tim_dshot_update_buffer(0, ch_id, _AL_TIM_DSHOT_INIT_PATTERN);
    }

    for (int tim_id = 0; tim_id < BSP_NR_DSHOT_TIMERs; tim_id++) {
        BSP_TIM_DSHOT_TIMID2DMAPARAMS(tim_id, htim, burstBaseAddress, offset, burstLength);
        HAL_TIM_DMABurst_WriteStart(htim, burstBaseAddress, TIM_DMA_UPDATE, _al_tim_dshot_burst_buffer[0] + offset, burstLength, 18);
    }

    return 0;
}

int al_tim_dshot_set(int fd, unsigned int value) {
    uint16_t pattern;
    int tim_id;
    int new_buff_id;

    if ((fd < 0 || fd >= BSP_NR_DSHOT_CHANNELs)
        || (value > _AL_TIM_DSHOT_MAX_THR - _AL_TIM_DSHOT_MIN_THR)) {
        return -1;
    }

    // make pattern
    pattern = _al_tim_dshot_make_pattern(value);

    // TODO: Mutex

    BSP_TIM_DSHOT_CHID2TIMID(fd, tim_id);
    new_buff_id = _al_tim_dshot_cal_new_buff_id(_al_tim_dshot_task_buff_id[tim_id], _al_tim_dshot_isr_buff_id[tim_id]);
    _al_tim_dshot_copy_buffer(fd, _al_tim_dshot_task_buff_id[tim_id], new_buff_id);
    _al_tim_dshot_update_buffer(new_buff_id, fd, pattern);
    _al_tim_dshot_task_buff_id[tim_id] = new_buff_id;

    return 0;
}

/* ISR callbacks -------------------------------------------------------------*/
/**
  * @brief  Period elapsed callback in non-blocking mode.
  * @note   This function is called when TIM7 interrupt took place inside
  *         HAL_TIM_IRQHandler(), or TIM1/TIM2 DMA complete callback.
  *         In the case of TIM7 interrupt took place, it makes a direct call to
  *         HAL_IncTick() to increment a global variable "uwTick" used as
  *         application time base.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    int tim_id;
    uint32_t burstBaseAddress;
    uint32_t offset;
    uint32_t burstLength;

    if (BSP_TIM_IS_DSHOT_TIMER(htim)) {
        BSP_TIM_DSHOT_HDL2DMAPARAMS(htim, tim_id, burstBaseAddress, offset, burstLength);
        HAL_TIM_DMABurst_WriteStart(htim, burstBaseAddress, TIM_DMA_UPDATE, _al_tim_dshot_burst_buffer[_al_tim_dshot_task_buff_id[tim_id]] + offset, burstLength, 18);
        _al_tim_dshot_isr_buff_id[tim_id] = _al_tim_dshot_task_buff_id[tim_id];
    }
}

/******************************** END OF FILE *********************************/
