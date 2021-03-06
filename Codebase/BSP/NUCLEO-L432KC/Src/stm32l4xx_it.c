/**
  ******************************************************************************
  * @file   stm32l4xx_it.c
  * @author MCD Application Team
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
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

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_i2c3_tx; // DMA1_CH2
extern DMA_HandleTypeDef hdma_i2c3_rx; // DMA1_CH3
extern DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com; // DMA1_CH5
extern DMA_HandleTypeDef hdma_tim1_up; // DMA1_CH6
extern DMA_HandleTypeDef hdma_usart2_tx; // DMA1_CH7
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_tx; // DMA2_CH6
extern I2C_HandleTypeDef hi2c3;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
    return;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while(1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while(1);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while(1);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while(1);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/*void SVC_Handler(void) {
    return;
}*/

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
    return;
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*void PendSV_Handler(void) {
    return;
}*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void) {
    HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xxxx.s).                                             */
/******************************************************************************/
/**
  * @brief  This function handles TIM7 global interrupt.
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim7);
    HAL_IncTick();
}

/**
  * @brief  This function handles DMA1 channel2 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_i2c3_tx);
}

/**
  * @brief  This function handles DMA1 channel3 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_i2c3_rx);
}

/**
  * @brief  This function handles DMA1 channel5 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim15_ch1_up_trig_com);
}

/**
  * @brief  This function handles DMA1 channel6 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_up);
}

/**
  * @brief  This function handles DMA1 channel7 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
  * @brief  This function handles USART1 global interrupt.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief  This function handles USART2 global interrupt.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief  This function handles DMA2 channel6 global interrupt.
  * @param  None
  * @retval None
  */
void DMA2_Channel6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
  * @brief  This function handles I2C3 event interrupt.
  * @param  None
  * @retval None
  */
void I2C3_EV_IRQHandler(void) {
    HAL_I2C_EV_IRQHandler(&hi2c3);
}

/**
  * @brief  This function handles I2C3 error interrupt.
  * @param  None
  * @retval None
  */
void I2C3_ER_IRQHandler(void) {
    HAL_I2C_ER_IRQHandler(&hi2c3);
}

/**
  * @brief  This function handles EXTI line0 interrupt.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief  This function handles EXTI line[15:10] interrupt.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
