/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* USER CODE BEGIN DMA_Pending_Clear */
  DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3 |
               DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5 | DMA_IFCR_CGIF6 |
               DMA_IFCR_CGIF7;
  DMAMUX1_ChannelStatus->CFR = DMAMUX_CFR_CSOF0 | DMAMUX_CFR_CSOF1 |
                               DMAMUX_CFR_CSOF2 | DMAMUX_CFR_CSOF3 |
                               DMAMUX_CFR_CSOF4 | DMAMUX_CFR_CSOF5 |
                               DMAMUX_CFR_CSOF6;
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);
  /* USER CODE END DMA_Pending_Clear */

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

