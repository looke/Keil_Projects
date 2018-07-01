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
	if(htim->Instance == TIMx_32bits)
	{
	  /*##-1- Enable peripheral clock #################################*/
    /* TIMx Peripheral clock enable */
    TIMx_32bits_CLK_ENABLE();
	
    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set the TIMx priority */
    HAL_NVIC_SetPriority(TIMx_32bits_IRQn, 5, 0);
	
    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIMx_32bits_IRQn);
	}
	if(htim->Instance == TIMx_ARHS)
	{
	  /*##-1- Enable peripheral clock #################################*/
    /* TIMx Peripheral clock enable */
	  TIMx_ARHS_CLK_ENABLE();
	
    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set the TIMx priority */
    HAL_NVIC_SetPriority(TIMx_ARHS_IRQn, 5, 0);
	
    /* Enable the TIMx global Interrupt */
	  HAL_NVIC_EnableIRQ(TIMx_ARHS_IRQn);
	}
	if(htim->Instance == TIMx_BUTTON_SHAKE)
	{
	  /*##-1- Enable peripheral clock #################################*/
    /* TIMx Peripheral clock enable */
	  TIMx_BUTTON_SHAKE_CLK_ENABLE();
	
    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set the TIMx priority */
	  HAL_NVIC_SetPriority(TIMx_BUTTON_SHAKE_IRQn, 6, 0);
	
    /* Enable the TIMx global Interrupt */
	  HAL_NVIC_EnableIRQ(TIMx_BUTTON_SHAKE_IRQn);	
	}

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
	TIMx_32bits_CLK_ENABLE();
	
	/* TIMx Channel clock enable */
	TIMx_32bits_CHANNEL_GPIO_PORT();
	
	// Configure in Alternate function, push-pull and 100MHz speed
	GPIO_InitStruct.Pin = GPIO_PIN_TIMx_32bits_CHANNEL_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF_TIMx_32bits;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	 /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
	HAL_NVIC_SetPriority(TIMx_32bits_IRQn, 5, 0);
	
	/* Enable the TIMx global Interrupt */
	HAL_NVIC_EnableIRQ(TIMx_32bits_IRQn);
}


void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* SDMMC1 Peripheral clock enable */
	SD1_CLK_ENABLE();
	/* SDMMC2 Peripheral clock enable */
  //SD2_CLK_ENABLE();
	
	/* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
	
	/* SDMMC1 port clock enable */
	SDMMC1_GEN_GPIO_PORT();
	SDMMC1_CMD_GPIO_PORT();
	
	/* SDMMC2 port clock enable */
  //SDMMC2_GEN_GPIO_D0_D4_PORT();
	//SDMMC2_CMD_GPIO_PORT();
	
	// Configure SDMMC1 port in Alternate function, push-pull and 100MHz speed
	
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
	
	HAL_NVIC_SetPriority(SDMMC1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
	
	
	/*
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

	HAL_NVIC_SetPriority(SDMMC2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
	*/
	
}

/*
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
	//SDMMC1 Peripheral clock disable
	SD1_CLK_DISABLE();
	
	HAL_GPIO_DeInit(GPIOC, SDMMC1_D0 | SDMMC1_D1 | SDMMC1_D2 | SDMMC1_D3 | SDMMC1_CK);
  HAL_GPIO_DeInit(GPIOD, SDMMC1_CMD);
	
	SDMMC1_GEN_GPIO_PORT_OFF();
	SDMMC1_CMD_GPIO_PORT_OFF();
	
	HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
}
*/

/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(hspi->Instance == SPIx)
  {     
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    //SPIx_SCK_GPIO_CLK_ENABLE();
    //SPIx_MISO_GPIO_CLK_ENABLE();
    //SPIx_MOSI_GPIO_CLK_ENABLE();
		SPIx_SCK_MISO_MOSI_GPIO_CLK_ENABLE();
		SPIx_CS_GPIO_CLK_ENABLE();
		
    /* Enable SPI clock */
    SPIx_CLK_ENABLE(); 
    
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);
		
		/* SPI CS GPIO pin configuration  */
		GPIO_InitStruct.Pin       = SPIx_CS_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SPIx_CS_AF;
		HAL_GPIO_Init(SPIx_CS_GPIO_PORT, &GPIO_InitStruct);
		
		/* Set CS Pin to High*/
		HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET); 
	}
	
}

/**
  * @brief SPI MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to its default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPIx)
  {   
    /*##-1- Reset peripherals ##################################################*/
    SPIx_FORCE_RESET();
    SPIx_RELEASE_RESET();
    
    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
		
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
		
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
		
		 /* Configure SPI CS as alternate function  */
    HAL_GPIO_DeInit(SPIx_CS_GPIO_PORT, SPIx_CS_PIN);
  }
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
