/**
  ******************************************************************************
  * @file   main.c
  * @author Ji Chen
  * @brief  Main program body
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#ifdef HORIZON_MINI_L4
#include "nucleo_l432kc_bsp.h"
#else
#error Please specify a target board.
#endif

int main(void) {
    BSP_MCU_Init();

    while (1) {
        for (int i = 0; i < 2; i++) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            HAL_Delay(50);
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            HAL_Delay(50);
        }
        HAL_Delay(800);
    }
}

/******************************** END OF FILE *********************************/
