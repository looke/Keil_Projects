/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Src/stm32f7xx_it.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    30-December-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include "stm32f7xx_it.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_TimeBase
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef    TimHandle_32bits;
extern TIM_HandleTypeDef    TimHandle_ARHS;

extern SD_HandleTypeDef    SDHandle_SDMMC;
extern DMA_HandleTypeDef   hdma_sdmmc;

extern LOOKE_SD_Global_Data_Cache SD_File_Cache;
extern uint8_t MAIN_STATUS;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F7xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f7xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles TIM Master interrupt request.
  * @param  None
  * @retval None
  */
//void TIMx_Master_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&TimHandle_Master);
//}

/**
  * @brief  This function handles TIM Slave interrupt request.
  * @param  None
  * @retval None
  */
//void TIMx_Slave_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&TimHandle_Slave);
//}


/**
  * @brief  This function handles TIM Slave interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_32bits_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle_32bits);
}

/**
  * @brief  This function handles TIM Slave interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_ARHS_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle_ARHS);
}



/**
  * @brief  This function handles SDMMC1 interrupt request.
  * @param  None
  * @retval None
  */
void SDMMC1_IRQHandler(void)
{
  HAL_SD_IRQHandler(&SDHandle_SDMMC);
}

/**
  * @brief  This function handles SDMMC2 interrupt request.
  * @param  None
  * @retval None
  */
//void SDMMC2_IRQHandler(void)
//{
//  HAL_SD_IRQHandler(&SDHandle_SDMMC);
//}


//void DMA2_Stream5_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(&hdma_sdmmc);
//}

void DMA2_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_sdmmc);
}


void EXTI15_10_IRQHandler(void)  
{  
	HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
	
	
	/*
  //Switch MAIN_STATUS after USER_BUTTON Pressed
	if(MAIN_STATUS == MAIN_STATUS_STOP)
	{
		//Start ARHS Timer and Input Capture Timer
		if (HAL_TIM_Base_Start_IT(&TimHandle_ARHS) == HAL_OK && HAL_TIM_IC_Start_IT(&TimHandle_32bits,TIM_CHANNEL_1) == HAL_OK)
    {
			MAIN_STATUS = MAIN_STATUS_START;
		  SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER;
    }
		else
		{
		  HAL_TIM_Base_Stop_IT(&TimHandle_ARHS);
			HAL_TIM_IC_Stop_IT(&TimHandle_32bits,TIM_CHANNEL_1);
		}
	}
	else if(MAIN_STATUS == MAIN_STATUS_START) //Stop the timer / Switch cache buffer state
  {
		if(HAL_TIM_Base_Stop_IT(&TimHandle_ARHS) == HAL_OK && HAL_TIM_IC_Stop_IT(&TimHandle_32bits,TIM_CHANNEL_1) == HAL_OK)
		{
			MAIN_STATUS = MAIN_STATUS_STOP;
		  SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_STOP;
		  SD_File_Cache.ARHS_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NEED_SYNC;
		  SD_File_Cache.TimeBase_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NEED_SYNC;
		}
		else
		{
		  MAIN_STATUS = MAIN_STATUS_ERROR;
		}
	}
	*/
}  

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
