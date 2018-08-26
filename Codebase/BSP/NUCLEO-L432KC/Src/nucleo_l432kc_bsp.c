/**
  ******************************************************************************
  * @file   nucleo_l432kc_bsp.c
  * @author Ji Chen
  * @brief  NUCLEO-L432KC Board Support Package.
  *         configure clocks,
  *         initialize on-chip peripherals (timer, UART, I2C, ...),
  *         enable interrupts,
  *         ...
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Global variables ----------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows:
  *          + System Clock source = PLL (HSI)
  *          + SYSCLK(Hz)          = 80000000
  *          + HCLK(Hz)            = 80000000
  *          + AHB Prescaler       = 1
  *          + APB1 Prescaler      = 1
  *          + APB2 Prescaler      = 1
  *          + HSI Frequency(Hz)   = 16000000
  *          + PLL_M               = 1
  *          + PLL_N               = 10
  *          + PLL_P               = 7
  *          + PLL_Q               = 2
  *          + PLL_R               = 2
  *          + Flash Latency(WS)   = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1);
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while(1);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        while(1);
    }

    /* Configure the main internal regulator output voltage */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        while(1);
    }

    /* Configure the Systick interrupt time */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);

    /* Configure the Systick */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, SYSTICK_INT_PRIORITY, 0);
}

/**
  * @brief  GPIO Initiation function
  * @param  None
  * @retval None
  */
static void BSP_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* GPIOB configuration */
    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    /* Configure GPIO pin: PB3(LD3 [Green]) */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void BSP_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, SYSTICK_INT_PRIORITY - 1U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, SYSTICK_INT_PRIORITY - 1U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void BSP_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while(1);
    }
}

void BSP_MCU_Init(void) {
    /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches.
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Low Level Initialization
    */
    HAL_Init();

    /* Configure the System clock to have a frequency of 80 MHz */
    SystemClock_Config();

    /* Initialize on-chip peripherals */
    BSP_GPIO_Init();
    BSP_DMA_Init();
    BSP_USART2_UART_Init();

    return;
}

#ifdef USE_FULL_ASSERT
void HAL_Assert_Failed(void) {
    /* disable all interrupts */
    __disable_irq();
    /* turn off LED */
    GPIOB->BRR = (uint32_t) GPIO_PIN_3;
    /* stop here */
    while(1);
}
#endif /* end of USE_FULL_ASSERT */

/* Interrupt service routines ------------------------------------------------*/
/**
  * @brief  Period elapsed callback in non-blocking mode.
  * @note   This function is called when TIM7 interrupt took place, inside
  *         HAL_TIM_IRQHandler().
  *         It makes a direct call to HAL_IncTick() to increment a global
  *         variable "uwTick" used as application time base.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (TIM7 == htim->Instance) {
        HAL_IncTick();
    }
}

/******************************** END OF FILE *********************************/
