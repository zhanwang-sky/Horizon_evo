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
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(80);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(920);
    }
}

/******************************** END OF FILE *********************************/
