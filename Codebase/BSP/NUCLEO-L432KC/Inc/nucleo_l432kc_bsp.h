/**
  ******************************************************************************
  * @file   nucleo_l432kc_bsp.h
  * @author Ji Chen
  * @brief  NUCLEO-L432KC BSP header file.
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef __NUCLEO_L432KC_BSP_H
#define __NUCLEO_L432KC_BSP_H

/* Function prototypes -------------------------------------------------------*/
void BSP_MCU_Init(void);
void BSP_SYSLED_Set_UnderInit(void);
void BSP_SYSLED_Set_Normal(void);
void BSP_SYSLED_Set_Fault(void);

#endif /* __NUCLEO_L432KC_BSP_H */

/******************************** END OF FILE *********************************/
