/**
  ******************************************************************************
  * @file   stm32l4xx_hal_msp.c
  * @author MCD Application Team
  * @brief  HAL MSP module.
  *         This file template is located in the HAL folder and should be copied
  *         to the user folder.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Global variables ----------------------------------------------------------*/
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_i2c3_tx;
DMA_HandleTypeDef hdma_i2c3_rx;
#ifdef _REVISE_N_OPTIMIZE_
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
#endif // _REVISE_N_OPTIMIZE_
DMA_HandleTypeDef hdma_tim1_up;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle) {
    GPIO_InitTypeDef GPIO_InitStruct;

    if (USART1 == uartHandle->Instance) {
        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        /* USART1 GPIO Configuration
           PB6 -> USART1_TX
           PB7 -> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART1 DMA Init */
        /* USART1_TX Init */
        hdma_usart1_tx.Instance = DMA2_Channel6;
        hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
        hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart1_tx.Init.Mode = DMA_NORMAL;
        hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else {
        /* USART2 clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        /* USART2 GPIO Configuration
           PA2 -> USART2_TX
           PA15 (JTDI) -> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 DMA Init */
        /* USART2_TX Init */
        hdma_usart2_tx.Instance = DMA1_Channel7;
        hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart2_tx);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle) {
    GPIO_InitTypeDef GPIO_InitStruct;

    if (I2C3 == i2cHandle->Instance) {
        /* I2C3 clock enable */
        __HAL_RCC_I2C3_CLK_ENABLE();

        /* I2C3 GPIO Configuration
           PA7 -> I2C3_SCL
           PB4 (NJTRST) -> I2C3_SDA
        */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2C3 DMA Init */
        /* I2C3_TX Init */
        hdma_i2c3_tx.Instance = DMA1_Channel2;
        hdma_i2c3_tx.Init.Request = DMA_REQUEST_3;
        hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
        hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(i2cHandle, hdmatx, hdma_i2c3_tx);

        /* I2C3_RX Init */
        hdma_i2c3_rx.Instance = DMA1_Channel3;
        hdma_i2c3_rx.Init.Request = DMA_REQUEST_3;
        hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
        hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(i2cHandle, hdmarx, hdma_i2c3_rx);

        /* I2C3 interrupt Init */
        HAL_NVIC_SetPriority(I2C3_EV_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
        HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
        HAL_NVIC_SetPriority(I2C3_ER_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
        HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
    }
}

#ifdef _REVISE_N_OPTIMIZE_
void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle) {
    GPIO_InitTypeDef GPIO_InitStruct;

    if (SPI1 == spiHandle->Instance) {
        /* SPI1 clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        /* SPI1 GPIO Configuration
           PA5 -> SPI1_SCK
           PA6 -> SPI1_MISO
           PB5 -> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI1 DMA Init */
        /* SPI1_RX Init */
        hdma_spi1_rx.Instance = DMA2_Channel3;
        hdma_spi1_rx.Init.Request = DMA_REQUEST_4;
        hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_rx.Init.Mode = DMA_NORMAL;
        hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi1_rx);

        /* SPI1_TX Init */
        hdma_spi1_tx.Instance = DMA1_Channel3;
        hdma_spi1_tx.Init.Request = DMA_REQUEST_1;
        hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx.Init.Mode = DMA_NORMAL;
        hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx);

        /* SPI1 interrupt Init */
        HAL_NVIC_SetPriority(SPI1_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
    }
}
#endif // _REVISE_N_OPTIMIZE_

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
    GPIO_InitTypeDef GPIO_InitStruct;

    if (TIM1 == htim->Instance) {
        /* TIM1 Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        /* TIM1 GPIO Configuration
           PA8  -> TIM1_CH1
           PA9  -> TIM1_CH2
           PA10 -> TIM1_CH3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* TIM1 DMA Init */
        /* TIM1_UP Init */
        hdma_tim1_up.Instance = DMA1_Channel6;
        hdma_tim1_up.Init.Request = DMA_REQUEST_7;
        hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_tim1_up.Init.Mode = DMA_NORMAL;
        hdma_tim1_up.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_tim1_up) != HAL_OK) {
            while(1);
        }
        __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_UPDATE], hdma_tim1_up);
    } else if (TIM15 == htim->Instance) {
        /* TIM15 Peripheral clock enable */
        __HAL_RCC_TIM15_CLK_ENABLE();

        /* TIM15 GPIO Configuration
           PA3 -> TIM15_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* TIM2 DMA Init */
        /* TIM2_UP Init */
        hdma_tim15_ch1_up_trig_com.Instance = DMA1_Channel5;
        hdma_tim15_ch1_up_trig_com.Init.Request = DMA_REQUEST_7;
        hdma_tim15_ch1_up_trig_com.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tim15_ch1_up_trig_com.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tim15_ch1_up_trig_com.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tim15_ch1_up_trig_com.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_tim15_ch1_up_trig_com.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_tim15_ch1_up_trig_com.Init.Mode = DMA_NORMAL;
        hdma_tim15_ch1_up_trig_com.Init.Priority = DMA_PRIORITY_LOW;
        if (HAL_DMA_Init(&hdma_tim15_ch1_up_trig_com) != HAL_OK) {
            while(1);
        }
        /* Several peripheral DMA handle pointers point to the same DMA handle.
        Be aware that there is only one channel to perform all the requested DMAs. */
        //__HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], hdma_tim15_ch1_up_trig_com);
        __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_UPDATE], hdma_tim15_ch1_up_trig_com);
        //__HAL_LINKDMA(htim, hdma[TIM_DMA_ID_TRIGGER], hdma_tim15_ch1_up_trig_com);
        //__HAL_LINKDMA(htim, hdma[TIM_DMA_ID_COMMUTATION], hdma_tim15_ch1_up_trig_com);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
