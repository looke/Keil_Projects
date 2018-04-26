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
/* TIM handle declaration */
//TIM_HandleTypeDef    TimHandle_Master;
//TIM_HandleTypeDef    TimHandle_Slave;

TIM_HandleTypeDef    TimHandle_32bits;
TIM_HandleTypeDef    TimHandle_ARHS;

SD_HandleTypeDef       SDHandle_SDMMC;
HAL_SD_CardInfoTypeDef SDCardInfo;
HAL_SD_CardCSDTypedef  SDCardCSDInfo;
HAL_SD_CardCIDTypedef  SDCardCIDInfo;

HAL_SD_CardStateTypedef SDCardState;
HAL_SD_CardStatusTypedef SDCardStatus;

HAL_SD_StateTypeDef SDState;

//TIM_MasterConfigTypeDef TimMasterConfig;
//TIM_SlaveConfigTypeDef  TimSlaveConfig;

//TIM_IC_InitTypeDef   TimsConfig;

//GPIO_InitTypeDef   GPIO_InitStruct;

DMA_HandleTypeDef hdma_sdmmc;

//uint32_t uwIC1Value1_Master = 0;
//uint32_t uwIC1Value2_Master = 0;
//uint32_t uwDiffCapture_Master = 0;

//uint32_t uwIC1Value1_Slave = 0;
//uint32_t uwIC1Value2_Slave = 0;
//uint32_t uwDiffCapture_Slave = 0;

//uint32_t uwIC1Value1_32bits = 0;
//uint32_t uwIC1Value2_32bits = 0;
//uint32_t uwDiffCapture_32bits = 0;

//uint32_t counter_cap = 0;

uint32_t errorstate;

//uint32_t testArray_Period[TEST_LIMIT];
//int testArray_Period_Diff[TEST_LIMIT];



__align(4) uint8_t align[16];
//__align(4) uint8_t aBuffer_Block_Rx[LOOKE_SD_FILE_BLOCK_SIZE*LOOKE_SD_FILE_CACHE_SIZE];
//__align(4) uint8_t aBuffer_Block_Tx[BLOCK_SIZE*CACHE_SIZE];

__align(4) LOOKE_SD_FileSys_Para_Union SD_FileSysParaUnion;
__align(4) LOOKE_SD_Global_Data_Cache SD_File_Cache;


//__align(4) LOOKE_SD_ARHS_Data_Cache ARHS_cache;
//__align(4) LOOKE_SD_TimeBase_Data_Cache TimeBase_cache;

//LOOKE_SD_FileSys_Para FileSysPara;
//LOOKE_SD_MeasureSection_Para MeasureSectionPara;
//LOOKE_SD_TimeBase_Data_Block TimeBaseDataBlock;
//LOOKE_SD_ARHS_Data_Block AHRSDataBlock;

//__align(4) uint8_t aBuffer_Block_Tx_Write[LOOKE_SD_FILE_BLOCK_SIZE];
__align(4) uint8_t aBuffer_Block_Rx[LOOKE_SD_FILE_BLOCK_SIZE];
__align(4) uint8_t aBuffer_Block_Tx[LOOKE_SD_FILE_BLOCK_SIZE];
/* Prescaler declaration */
__IO uint32_t uwPrescalerValue = 0;

uint32_t startTimeStamp;
uint32_t endTimeStamp;
uint32_t timeEclipse;

uint32_t timeTest;

uint32_t sizeTest;

LOOKE_SD_TimeBase_Data timeBaseDataTest;

uint8_t i;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/
void DelaySomeTime(void);
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
	sizeTest = sizeof(LOOKE_SD_FileSys_Para);
	sizeTest = sizeof(LOOKE_SD_MeasureSection_Para);
	sizeTest = sizeof(LOOKE_SD_TimeBase_Data_Block);
	sizeTest = sizeof(LOOKE_SD_ARHS_Data_Block);
	
	sizeTest = sizeof(LOOKE_SD_FileSys_Para_Union);
	sizeTest = sizeof(LOOKE_SD_MeasureSection_Para_Union);
	
	sizeTest = sizeof(LOOKE_SD_TimeBase_Data_Buffer_Union);
	sizeTest = sizeof(LOOKE_SD_ARHS_Data_Buffer_Union);
	
  sizeTest = sizeof(LOOKE_SD_ARHS_Data_Cache);
	sizeTest = sizeof(LOOKE_SD_TimeBase_Data_Cache);
	
	
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Configure LED1 & LED3 */
  BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

	//Setup Global Cache
	SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER;
  SD_File_Cache.TimeBase_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NOT_FULL;
  SD_File_Cache.TimeBase_Cache.CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_MASTER;
	
	
	TimHandle_32bits.Instance = TIMx_32bits;
	//TimHandle_32bits.Init.Period            = 0xFFFFFFFF;
	TimHandle_32bits.Init.Period            = 120000-1; //100Sps
	//TimHandle_32bits.Init.Prescaler         = 26;	// 108Mhz/27 = 4Mhz
	//TimHandle_32bits.Init.Prescaler         = 2;	// 108Mhz/3 = 36Mhz
	TimHandle_32bits.Init.Prescaler         = 8;	// 108Mhz/9 = 12Mhz
	//TimHandle_32bits.Init.Prescaler         = 107;	// 108Mhz/108 = 1Mhz
  TimHandle_32bits.Init.ClockDivision     = 0;
  TimHandle_32bits.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_32bits.Init.RepetitionCounter = 0;
  TimHandle_32bits.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  //uint32_t ucPrescaler = (uint32_t)((SystemCoreClock/2)/1000000) - 1;
	TimHandle_ARHS.Instance = TIMx_ARHS;
	TimHandle_ARHS.Init.Period            = 9999;
	//TimHandle_ARHS.Init.Prescaler         = 26;	// 108Mhz/27 = 4Mhz
	//TimHandle_ARHS.Init.Prescaler         = 2;	// 108Mhz/3 = 36Mhz
	//TimHandle_ARHS.Init.Prescaler         = 8;	// 108Mhz/9 = 12Mhz
	//TimHandle_ARHS.Init.Prescaler         = 107;	// 108Mhz/108 = 1Mhz
	TimHandle_ARHS.Init.Prescaler         = 10799;	// 108Mhz/10800 = 10Khz
	TimHandle_ARHS.Init.ClockDivision     = 0;
  TimHandle_ARHS.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_ARHS.Init.RepetitionCounter = 0;
  TimHandle_ARHS.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  /*##-1- Configure the SD peripheral #######################################*/
  SDHandle_SDMMC.Instance = SDMMC2;
	SDHandle_SDMMC.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	//SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_ENABLE;
  SDHandle_SDMMC.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  SDHandle_SDMMC.Init.BusWide = SDMMC_BUS_WIDE_1B;
	SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  //SDHandle_SDMMC.Init.ClockDiv = 0; //24Mhz
  SDHandle_SDMMC.Init.ClockDiv = 1; //12Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 2; //8Mhz
	
	if (HAL_SD_Init(&SDHandle_SDMMC) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_SD_ConfigWideBusOperation(&SDHandle_SDMMC, SDMMC_BUS_WIDE_4B) != HAL_OK )
	{
		Error_Handler();
	}

  if (HAL_SD_GetCardInfo(&SDHandle_SDMMC, &SDCardInfo) != HAL_OK)
	{
		Error_Handler();
	}
	
	if (HAL_SD_GetCardCSD(&SDHandle_SDMMC, &SDCardCSDInfo) != HAL_OK)
	{
		Error_Handler();
	}
	
	if (HAL_SD_GetCardCID(&SDHandle_SDMMC, &SDCardCIDInfo) != HAL_OK)
	{
		Error_Handler();
	}
	
	/* Configure the SD DPSM (Data Path State Machine) */ 
	//SDMMC_DataInitTypeDef config;
  //config.DataTimeOut   = SDMMC_DATATIMEOUT;
  //config.DataLength    = 64;
  //config.DataBlockSize = SDMMC_DATABLOCK_SIZE_64B;
  //config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
  //config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
  //config.DPSM          = SDMMC_DPSM_ENABLE;
  //SDMMC_ConfigData(SDHandle_SDMMC1.Instance, &config);
	//errorstate = SDMMC_CmdBlockLength(SDHandle_SDMMC1.Instance, 64);
	
  if (HAL_SD_GetCardStatus(&SDHandle_SDMMC, &SDCardStatus) != HAL_OK)
	{
		Error_Handler();
	}
	
	//Config DMA Channel for SDMMC2
	hdma_sdmmc.Instance = DMA2_Stream5;
	hdma_sdmmc.Init.Channel = DMA_CHANNEL_11;

	//Config DMA Channel for SDMMC1
	//hdma_sdmmc.Instance = DMA2_Stream6;
  //hdma_sdmmc.Instance = DMA2_Stream3;
	//hdma_sdmmc.Init.Channel = DMA_CHANNEL_4;
	
	//DMA read process DMA_PERIPH_TO_MEMORY
	//hdma_sdmmc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	
	//DMA write process DMA_MEMORY_TO_PERIPH
	hdma_sdmmc.Init.Direction = DMA_MEMORY_TO_PERIPH;
	
  hdma_sdmmc.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sdmmc.Init.MemInc = DMA_MINC_ENABLE;

	hdma_sdmmc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sdmmc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

	hdma_sdmmc.Init.Mode = DMA_SxCR_PFCTRL;
  hdma_sdmmc.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_sdmmc.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_sdmmc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	
	hdma_sdmmc.Init.MemBurst = DMA_MBURST_INC4;
	hdma_sdmmc.Init.PeriphBurst = DMA_PBURST_INC4;
	
	/* DMA2 Stream6 */
	__HAL_DMA_DISABLE(&hdma_sdmmc);
	HAL_DMA_DeInit(&hdma_sdmmc);
	
	if (HAL_DMA_Init(&hdma_sdmmc) != HAL_OK)
  {
    Error_Handler();
  }
	
	//DMA Read Link
	//__HAL_LINKDMA(&SDHandle_SDMMC,hdmarx,hdma_sdmmc);
	
	//DMA Write Link
	__HAL_LINKDMA(&SDHandle_SDMMC,hdmatx,hdma_sdmmc);

	/* DMA interrupt init */
  /* DMA2_Channel11_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	
	/* DMA2_Channel4_6_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	
  /*
	//Format SD File SYSPara
	SD_FileSysParaUnion.FileSysPara.NumberOfMeasurementSection = 0;
	SD_FileSysParaUnion.FileSysPara.SectionIndexArray[0].SectionStartBlock = 0;
	SD_FileSysParaUnion.FileSysPara.SectionIndexArray[0].SectionEndBlock = 0;
  LOOKE_SD_File_WriteSysPara(&SDHandle_SDMMC, &SD_FileSysParaUnion);
	
	LOOKE_SD_File_CreateMeasureSection(&SDHandle_SDMMC, &SD_FileSysParaUnion);
	*/
	
	//Init SD File SYSPara
	if(LOOKE_SD_File_ReadSysPara(&SDHandle_SDMMC, &SD_FileSysParaUnion) != HAL_OK)
	{
	  BSP_LED_Toggle(LED3);
	}
	
	
	//if(HAL_SD_ReadBlocks_DMA(&SDHandle_SDMMC, aBuffer_Block_Rx, 0x02, 1) == HAL_OK)
	//{
	//  SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
	//}
	
	//if(HAL_SD_ReadBlocks(&SDHandle_SDMMC, aBuffer_Block_Rx, 0x10, 1, 0xfff) == HAL_OK)
	//{
	//  SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
	//}	
	
	/*
	uint16_t initForBuffer;
	for(initForBuffer = 0 ; initForBuffer<LOOKE_SD_FILE_BLOCK_SIZE; initForBuffer++)
	{
		aBuffer_Block_Tx[initForBuffer] = 0x00;
	}
	
	uint8_t testValue = 0x01;
	for(initForBuffer = 4 ; initForBuffer<LOOKE_SD_FILE_BLOCK_SIZE; )
	{
		aBuffer_Block_Tx[initForBuffer] = testValue;
		initForBuffer = initForBuffer+4;
		aBuffer_Block_Tx[initForBuffer] = testValue;
		initForBuffer = initForBuffer+4;
		testValue++;
	}
	aBuffer_Block_Tx[508] = 0x00;
	aBuffer_Block_Tx[509] = 0x00;
	aBuffer_Block_Tx[510] = 0x00;
	aBuffer_Block_Tx[511] = 0x00;
	
	SCB_CleanDCache();
	
	//startTimeStamp = __HAL_TIM_GET_COUNTER(&TimHandle_32bits);
	
	if(HAL_SD_WriteBlocks_DMA(&SDHandle_SDMMC, aBuffer_Block_Tx, 0x10, 1) == HAL_OK)
	{
	  SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
		//Error_Handler();
	}
  */
	//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
	
	
	//LOOKE_SD_TimeBase_Data_Buffer_TEST_Union testUnion;
	//for(i=0; i<TIMEBASE_BLOCK_DATA_SIZE; i++)
	//{
	//  testUnion.dataBlockArray.TimeBaseData[i].DataIndex = i+1;
	//	testUnion.dataBlockArray.TimeBaseData[i].TimeStamp = i+1;
	//}
	
	//SCB_CleanDCache();
	//if (HAL_SD_WriteBlocks_DMA(&SDHandle_SDMMC, testUnion.DataArray, 0x02, 1) == HAL_OK)
	//{
	//	SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
	//}
	
	/*
	uint16_t initForBuffer;
	for(initForBuffer = 0 ; initForBuffer<BLOCK_SIZE; initForBuffer++)
	{
		aBuffer_Block_Tx_Write[initForBuffer] = 0x56;
	}
	
  if (HAL_SD_WriteBlocks(&SDHandle_SDMMC, aBuffer_Block_Tx_Write, 0x00, 1, 0x0F) != HAL_OK)
  {
		Error_Handler();
	}
	
	if (HAL_SD_WriteBlocks(&SDHandle_SDMMC, aBuffer_Block_Tx_Write, 0x01, 1, 0x0F) != HAL_OK)
  {
		Error_Handler();
	}
	
	if (HAL_SD_WriteBlocks(&SDHandle_SDMMC, aBuffer_Block_Tx_Write, 0x02, 1, 0x0F) != HAL_OK)
  {
		Error_Handler();
	}
	
	if (HAL_SD_WriteBlocks(&SDHandle_SDMMC, aBuffer_Block_Tx_Write, 0x03, 1, 0x0F) != HAL_OK)
  {
		Error_Handler();
	}
	 
	if (HAL_SD_WriteBlocks(&SDHandle_SDMMC, aBuffer_Block_Tx_Write, 0x04, 1, 0x0F) != HAL_OK)
  {
		Error_Handler();
	}
	*/
	
	/*
	if (HAL_SD_GetCardInfo(&SDHandle_SDMMC, &SDCardInfo) != HAL_OK)
	{
		Error_Handler();
	}
	
	if (HAL_SD_GetCardCSD(&SDHandle_SDMMC, &SDCardCSDInfo) != HAL_OK)
	{
		Error_Handler();
	}
	
	if (HAL_SD_GetCardCID(&SDHandle_SDMMC, &SDCardCIDInfo) != HAL_OK)
	{
		Error_Handler();
	}
	*/
	
	//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
	/* USER CODE BEGIN 2 */

	//int16_t i = 0;
	//for( ; i<BLOCK_SIZE; i++)
	//{
	//	aBuffer_Block_Tx[i] = i;
	//}
	
	
	//Set Timer for test case
	if (HAL_TIM_Base_Init(&TimHandle_32bits) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }
	
	//Start Time Base
	if (HAL_TIM_Base_Start_IT(&TimHandle_32bits) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }
	
	
	/*
	//Set Timer for test case
	if (HAL_TIM_Base_Init(&TimHandle_ARHS) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }
	
	//Start Time Base
	if (HAL_TIM_Base_Start_IT(&TimHandle_ARHS) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }
	*/
	
  while (1)
  {
		//SDState = HAL_SD_GetState(&SDHandle_SDMMC1);
		//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC1);
		
		//if (HAL_SD_ReadBlocks(&SDHandle_SDMMC1, aBuffer_Block_Rx, 0x02, 0x01, 0x0F) != HAL_OK)
		//{
		//	Error_Handler();
	  //}
		
		/*
		timeEclipse = endTimeStamp - startTimeStamp;
		DelaySomeTime();
		*/
		
		//Check Cache Globle State and Buffer State. Start Sync Process if Need;
		if(SD_File_Cache.TimeBase_Cache.CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC && SD_File_Cache.CurrentGlobalState == LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER)
		{

			//Sync Process
		  LOOKE_SD_FILE_SYNC_TRANSFER_RESULT syncResult = LOOKE_SD_File_SyncCacheToSDCard_TimeBase(&SDHandle_SDMMC, &SD_FileSysParaUnion.FileSysPara, &SD_File_Cache);

		}
		
		
		//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
		//BSP_LED_Toggle(LED3);
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
  /* Turn LED3 on */
  BSP_LED_On(LED3);
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

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
	uint32_t error = hsd->hdmatx->ErrorCode;
	
	//SD TX Transfer ERROR,Switch SD Cache Global State
	SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_SYNC_ERROR;
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

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
	//endTimeStamp = __HAL_TIM_GET_COUNTER(&TimHandle_32bits);
	
	//After DMA TX Transfer, Switch State
	if(SD_File_Cache.CurrentGlobalState == LOOKE_SD_FILE_GLOBAL_CACHE_STATE_SYNC_TIMEBASE)
	{
	  SD_File_Cache.TimeBase_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NOT_FULL;
	}
	else if(SD_File_Cache.CurrentGlobalState == LOOKE_SD_FILE_GLOBAL_CACHE_STATE_SYNC_ARHS)
	{
	  SD_File_Cache.ARHS_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NOT_FULL;
	}
	
	//Update File SysPara
	uint32_t measureSectionNum = SD_FileSysParaUnion.FileSysPara.NumberOfMeasurementSection;
	
	SD_FileSysParaUnion.FileSysPara.SectionIndexArray[measureSectionNum-1].SectionEndBlock += LOOKE_SD_FILE_CACHE_SIZE;
	
	//Sync File SysPara to SD Card
	LOOKE_SD_File_WriteSysPara(&SDHandle_SDMMC, &SD_FileSysParaUnion);
	
	SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Test SD File Cache
	if(htim->Instance == TIMx_32bits)
	{
	  BSP_LED_Toggle(LED1);
	  timeTest++;
	
	  timeBaseDataTest.DataIndex = timeTest;
	  timeBaseDataTest.TimeStamp = timeTest;
	
	  LOOKE_SD_File_AddTimeBaseMeasureToCache(&SD_File_Cache,&timeBaseDataTest);
	}
	
  if(htim->Instance == TIMx_ARHS)
	{
		BSP_LED_Toggle(LED2);
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
