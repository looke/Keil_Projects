/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    30-December-2016
  * @brief   This sample code shows how to use STM32F7xx TIM HAL API to generate
  *          a time base of one second with the corresponding Interrupt request.
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

/** @addtogroup TIM_TimeBase
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t MAIN_STATUS;


SD_HandleTypeDef       SDHandle_SDMMC;

HAL_SD_CardStatusTypedef SDCardStatus;

//HAL_SD_StateTypeDef SDState;

DMA_HandleTypeDef hdma_sdmmc_Write;
//DMA_HandleTypeDef hdma_sdmmc_Read;

__align(4) uint8_t aBuffer_Block_Target[LOOKE_SD_FILE_BLOCK_SIZE*LOOKE_SD_FILE_CACHE_SIZE];
/* Prescaler declaration */
__IO uint32_t uwPrescalerValue = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
//void LOOKE_SD_DMA_XferError(DMA_HandleTypeDef *hdma);

/* Private functions ---------------------------------------------------------*/
void DelaySomeTime(void);
void DelayShortTime(void);

void ConfigSDMMC(void);
void ConfigHDMA_Write(void);

/* Private functions ---------------------------------------------------------*/


void Test_SyncCacheToSDCard_ARHS(uint8_t *pBuff);
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
	
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Configure LED1 & LED3 */
  BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

  	
  /*##-1- Configure the SD peripheral #######################################*/
	ConfigSDMMC();
	
	
	DelaySomeTime();
	if (HAL_SD_Init(&SDHandle_SDMMC) != HAL_OK)
  {
    Error_Handler();
  }
  DelaySomeTime();
  if (HAL_SD_ConfigWideBusOperation(&SDHandle_SDMMC, SDMMC_BUS_WIDE_4B) != HAL_OK )
	{
		Error_Handler();
	}
	
	DelaySomeTime();
  //if (HAL_SD_GetCardStatus(&SDHandle_SDMMC, &SDCardStatus) != HAL_OK)
	//{
	//	Error_Handler();
	//}

	//Config DMA Channel for SDMMC1
	ConfigHDMA_Write();

	
	__HAL_DMA_DISABLE(&hdma_sdmmc_Write);
	HAL_DMA_DeInit(&hdma_sdmmc_Write);
	
	if (HAL_DMA_Init(&hdma_sdmmc_Write) != HAL_OK)
  {
    Error_Handler();
	}
		
	//DMA Write Link
	__HAL_LINKDMA(&SDHandle_SDMMC,hdmatx,hdma_sdmmc_Write);
	/* DMA2_Channel4_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 2);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  
	DelaySomeTime();
	
	
	
		//SCB_CleanDCache();
	  Test_SyncCacheToSDCard_ARHS(aBuffer_Block_Target);
	  DelaySomeTime();
	  DelaySomeTime();
	
  while (1)
  {
		
  }
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }

  /* Activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    while(1) {};
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    while(1) {};
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}




void DelaySomeTime(void)
{
   uint16_t i=0;
	 uint16_t time=0xFFFF;
   while(time--)
   {
      i=0xFF;
      while(i--);
   }
}

void DelayShortTime(void)
{
	 uint16_t time=0x00FF;
   while(time--)
   {

   }
}



void ConfigSDMMC(void)
{
  SDHandle_SDMMC.Instance = SDMMC1;
	SDHandle_SDMMC.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	//SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_ENABLE;
  SDHandle_SDMMC.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  SDHandle_SDMMC.Init.BusWide = SDMMC_BUS_WIDE_1B;
	SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	//SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  //SDHandle_SDMMC.Init.ClockDiv = 0; //24Mhz
  //SDHandle_SDMMC.Init.ClockDiv = 1; //12Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 2; //8Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 3; //6Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 4; //4.8Mhz
	SDHandle_SDMMC.Init.ClockDiv = 5; //4Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 6; //3.4Mhz for Sony 4C run 4 hours
	//SDHandle_SDMMC.Init.ClockDiv = 7; //3Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 11; //2Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 23; //1Mhz
}

void ConfigHDMA_Write(void)
{
	//hdma_sdmmc.Instance = DMA2_Stream6;
  hdma_sdmmc_Write.Instance = DMA2_Stream3;
	hdma_sdmmc_Write.Init.Channel = DMA_CHANNEL_4;
	
	//hdma_sdmmc.Instance = DMA2_Stream5;
	//hdma_sdmmc.Init.Channel = DMA_CHANNEL_11;
	
	//DMA read process DMA_PERIPH_TO_MEMORY
	//hdma_sdmmc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	
	//DMA write process DMA_MEMORY_TO_PERIPH
	hdma_sdmmc_Write.Init.Direction = DMA_MEMORY_TO_PERIPH;
	
  hdma_sdmmc_Write.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sdmmc_Write.Init.MemInc = DMA_MINC_ENABLE;

	hdma_sdmmc_Write.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sdmmc_Write.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

	hdma_sdmmc_Write.Init.Mode = DMA_SxCR_PFCTRL;
  hdma_sdmmc_Write.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_sdmmc_Write.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	//hdma_sdmmc_Write.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_sdmmc_Write.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	
	hdma_sdmmc_Write.Init.MemBurst = DMA_MBURST_INC4;
	hdma_sdmmc_Write.Init.PeriphBurst = DMA_PBURST_INC4;
	
	//hdma_sdmmc.XferErrorCallback = LOOKE_SD_DMA_XferError;
}


void Test_SyncCacheToSDCard_ARHS(uint8_t *pBuff)
{
	//SCB_CleanDCache();
	//if (HAL_SD_WriteBlocks_DMA(hsd, pBufferUnion->DataArray, currentBlockIndexOnSD, LOOKE_SD_FILE_CACHE_SIZE) != HAL_OK)
  //if (HAL_SD_WriteBlocks_DMA(&SDHandle_SDMMC, SD_File_Cache.ARHS_Cache.ARHS_DataBuffer_Slave.DataArray, 0x02, LOOKE_SD_FILE_CACHE_SIZE) != HAL_OK)
  if (HAL_SD_WriteBlocks_DMA(&SDHandle_SDMMC, aBuffer_Block_Target, 0x100, LOOKE_SD_FILE_CACHE_SIZE) != HAL_OK)
	{
	  Error_Handler();
	}
	
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
