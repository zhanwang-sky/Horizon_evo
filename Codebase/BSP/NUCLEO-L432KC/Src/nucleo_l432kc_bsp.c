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
#include "nucleo_l432kc_bsp_config.h"

/* Global variables ----------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c3;
#ifdef _REVISE_N_OPTIMIZE_
SPI_HandleTypeDef hspi1;
#endif // _REVISE_N_OPTIMIZE_
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim15;

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows:
  *          + System Clock source = PLL (HSE)
  *          + SYSCLK(Hz)          = 80000000
  *          + HCLK(Hz)            = 80000000
  *          + AHB Prescaler       = 1
  *          + APB1 Prescaler      = 1
  *          + APB2 Prescaler      = 1
  *          + HSE Frequency(Hz)   = 8000000
  *          + PLL_M               = 1
  *          + PLL_N               = 20
  *          + PLL_P               = 7
  *          + PLL_Q               = 2
  *          + PLL_R               = 2
  *          + Flash Latency(WS)   = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = BSP_SYSCLK_PLLM;
    RCC_OscInitStruct.PLL.PLLN = BSP_SYSCLK_PLLN;
    RCC_OscInitStruct.PLL.PLLP = BSP_SYSCLK_PLLP;
    RCC_OscInitStruct.PLL.PLLQ = BSP_SYSCLK_PLLQ;
    RCC_OscInitStruct.PLL.PLLR = BSP_SYSCLK_PLLR;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1);
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while(1);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
                                    | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C3;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
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
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    /* GPIOA configuration */
    /* Configure GPIO pin: PA4(NSS1/D3/A0) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOB configuration */
    /* Configure GPIO pin: PB0(INT1/D4/A1) */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* Configure GPIO pin: PB1(D5/A2), PB3(D0) */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* GPIOC configuration */
    /* Configure GIPI pin: PC14(INT0/D1) */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    /* Configure GIPI pin: PC15(NSS0/D2) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, SYSTICK_INT_PRIORITY - 1U, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, SYSTICK_INT_PRIORITY - 1U, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void BSP_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    /* DMA2_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, SYSTICK_INT_PRIORITY - 2U, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);
}

void BSP_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = BSP_UART_BAUD_RATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while(1);
    }
}

void BSP_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = BSP_UART_BAUD_RATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while(1);
    }
}

void BSP_I2C3_Init(void) {
    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = BSP_I2C_TIMING;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        while(1);
    }

    /* Configure Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        while(1);
    }

    /*Configure Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
        while(1);
    }
}

#ifdef _REVISE_N_OPTIMIZE_
void BSP_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = BSP_SPI_PRESCALER;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        while(1);
    }
}
#endif // _REVISE_N_OPTIMIZE_

void BSP_TIM1_Init(void) {
    TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
    TIM_OC_InitTypeDef sConfig = { 0 };

    /* Configure the TIM peripheral */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = BSP_TIM_DSHOT_PRESCALER;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = BSP_TIM_DSHOT_PERIOD - 1U;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        while(1);
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK) {
        while(1);
    }

    /* Common configuration for all channels */
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.Pulse = 0U;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    /* Set the pulse value for channel 1 */
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
        while(1);
    }
    /* Set the pulse value for channel 2 */
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
        while(1);
    }
    /* Set the pulse value for channel 3 */
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_3) != HAL_OK) {
        while(1);
    }

    /* Start PWM output */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void BSP_TIM15_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfig = { 0 };

    /* Configure the TIM peripheral */
    htim15.Instance = TIM15;
    htim15.Init.Prescaler = BSP_TIM_DSHOT_PRESCALER;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = BSP_TIM_DSHOT_PERIOD - 1U;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
        while(1);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) {
        while(1);
    }

    /* Common configuration for all channels */
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.Pulse = 0U;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    /* Set the pulse value for channel 2 */
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
        while(1);
    }

    /* Start PWM output */
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void BSP_MCU_Init(void) {
#ifdef _REVISE_N_OPTIMIZE_
    /* workaround */
    uint8_t txdata = 0xFF, rxdata = 0;
#endif // _REVISE_N_OPTIMIZE_

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
    BSP_USART1_UART_Init();
    BSP_USART2_UART_Init();
    BSP_I2C3_Init();
#ifdef _REVISE_N_OPTIMIZE_
    BSP_SPI1_Init();
    /* workaround */
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, 10);
#endif // _REVISE_N_OPTIMIZE_
    BSP_TIM1_Init();
    BSP_TIM15_Init();

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

/******************************** END OF FILE *********************************/
