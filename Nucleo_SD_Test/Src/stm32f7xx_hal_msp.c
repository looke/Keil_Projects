/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Src/stm32f7xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    30-December-2016
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "main.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripheral clock #################################*/
  /* TIMx Peripheral clock enable */
  //TIMx_Master_CLK_ENABLE();
  TIMx_32bits_CLK_ENABLE();
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  //HAL_NVIC_SetPriority(TIMx_Master_IRQn, 3, 0);

  /* Enable the TIMx global Interrupt */
  //HAL_NVIC_EnableIRQ(TIMx_Master_IRQn);
}

/**
  * @brief TIM Input Capture MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /*##-1- Enable peripheral clock #################################*/
  /* TIMx Peripheral clock enable */
  //TIMx_Master_CLK_ENABLE();
	//TIMx_Slave_CLK_ENABLE();
	
	TIMx_32bits_CLK_ENABLE();
	
	/* TIMx Channel clock enable */
	//TIMx_Master_CHANNEL_GPIO_PORT();
	//TIMx_Slave_CHANNEL_GPIO_PORT();
	TIMx_32bits_CHANNEL_GPIO_PORT();
	
	// Configure  (TIM3x_Channel) in Alternate function, push-pull and 100MHz speed
	//GPIO_InitStruct.Pin = GPIO_PIN_TIMx_Master_CHANNEL_1;
	GPIO_InitStruct.Pin = GPIO_PIN_TIMx_32bits_CHANNEL_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	//GPIO_InitStruct.Alternate = GPIO_AF_TIMx_Master;
	GPIO_InitStruct.Alternate = GPIO_AF_TIMx_32bits;
	
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//GPIO_InitStruct.Pin = GPIO_PIN_TIMx_Slave_CHANNEL_1;
	//GPIO_InitStruct.Alternate = GPIO_AF_TIMx_Slave;
	
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	// Configure  (TIM2x_Channel) in Alternate function, push-pull and 100MHz speed
	//GPIO_InitStruct.Pin = GPIO_PIN_TIMx_Slave_CHANNEL_1;
	//GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	//GPIO_InitStruct.Pull = GPIO_PULLUP;
	//GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	//GPIO_InitStruct.Alternate = GPIO_AF_TIMx_Slave;
	
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	 /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  //HAL_NVIC_SetPriority(TIMx_Master_IRQn, 3, 0);
	//HAL_NVIC_SetPriority(TIMx_Slave_IRQn, 3, 0);
	HAL_NVIC_SetPriority(TIMx_32bits_IRQn, 4, 0);
	/* Enable the TIMx global Interrupt */
  //HAL_NVIC_EnableIRQ(TIMx_Master_IRQn);
  //HAL_NVIC_EnableIRQ(TIMx_Slave_IRQn);
	HAL_NVIC_EnableIRQ(TIMx_32bits_IRQn);
}


void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* SDMMC1 Peripheral clock enable */
	//SD1_CLK_ENABLE();
	/* SDMMC2 Peripheral clock enable */
  SD2_CLK_ENABLE();
	/* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
	
	/* SDMMC1 port clock enable */
	//SDMMC1_GEN_GPIO_PORT();
	//SDMMC1_CMD_GPIO_PORT();
	
	/* SDMMC2 port clock enable */
  SDMMC2_GEN_GPIO_D0_D4_PORT();
	SDMMC2_CMD_GPIO_PORT();
	
	// Configure SDMMC1 port in Alternate function, push-pull and 100MHz speed
	/*
	GPIO_InitStruct.Pin = SDMMC1_D0 | SDMMC1_D1 | SDMMC1_D2 | SDMMC1_D3 | SDMMC1_CK;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF_SDMMC1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = SDMMC1_CMD;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	*/
	
	// Configure SDMMC2 port in Alternate function, push-pull and 100MHz speed
	GPIO_InitStruct.Pin = SDMMC2_D0 | SDMMC2_D1 | SDMMC2_D2 | SDMMC2_D3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF_SDMMC2_GEN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = SDMMC2_CK | SDMMC2_CMD;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF_SDMMC2_CMD;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(SDMMC2_IRQn, 3, 1);
	HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
	

}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
