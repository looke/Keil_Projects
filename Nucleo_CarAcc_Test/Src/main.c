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
/* TIM handle declaration */

TIM_HandleTypeDef    TimHandle_32bits;
TIM_HandleTypeDef    TimHandle_ARHS;

/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

SD_HandleTypeDef       SDHandle_SDMMC;
HAL_SD_CardInfoTypeDef SDCardInfo;
HAL_SD_CardCSDTypedef  SDCardCSDInfo;
HAL_SD_CardCIDTypedef  SDCardCIDInfo;

HAL_SD_CardStateTypedef SDCardState;
HAL_SD_CardStatusTypedef SDCardStatus;

HAL_SD_StateTypeDef SDState;

DMA_HandleTypeDef hdma_sdmmc;

TIM_IC_InitTypeDef TimsConfig;

uint32_t errorstate;
__align(4) uint8_t align[16];

__align(4) LOOKE_SD_FileSys_Para_Union SD_FileSysParaUnion;
__align(4) LOOKE_SD_Global_Data_Cache SD_File_Cache;

LOOKE_SD_TimeBase_Data timeBaseDataTest;
LOOKE_SD_ARHS_Data arhsDataTest;

//__align(4) LOOKE_SD_ARHS_Data_Cache ARHS_cache;
//__align(4) LOOKE_SD_TimeBase_Data_Cache TimeBase_cache;

//LOOKE_SD_FileSys_Para FileSysPara;
//LOOKE_SD_MeasureSection_Para MeasureSectionPara;
//LOOKE_SD_TimeBase_Data_Block TimeBaseDataBlock;
//LOOKE_SD_ARHS_Data_Block AHRSDataBlock;

//__align(4) uint8_t aBuffer_Block_Tx_Write[LOOKE_SD_FILE_BLOCK_SIZE];
//__align(4) uint8_t aBuffer_Block_Rx[LOOKE_SD_FILE_BLOCK_SIZE];
//__align(4) uint8_t aBuffer_Block_Tx[LOOKE_SD_FILE_BLOCK_SIZE*LOOKE_SD_FILE_CACHE_SIZE];
/* Prescaler declaration */
__IO uint32_t uwPrescalerValue = 0;

uint32_t startTimeStamp;
uint32_t endTimeStamp;
uint32_t timeEclipse;

uint32_t captureNumber;
uint32_t autoReloadValue;

uint32_t sizeTest;


uint8_t i;

uint16_t waitCounter4SDMMC;
uint32_t startBlockIndex;
uint32_t timerCounter;
uint32_t whileInSDCounter;
uint32_t whileCounter;

uint32_t TotalERRORCounter;
uint32_t SDTransErrorCounter;
uint32_t FIFOErrorCounter;
uint32_t DMEErrorCounter;
uint32_t TIMEOUTErrorCounter;
uint32_t PARAMErrorCounter;
uint32_t NOXFERErrorCounter;
uint32_t NOTSUPPErrorCounter;

uint32_t DMASendErrorCounter;
uint32_t waitErrorCounter;
uint32_t missOutCounter;
uint32_t EraseError;

//uint8_t GLOBAL_SDTRANSFER_SWITCH;
uint8_t DMAERROR_SDTRANSFER;

uint32_t errorBlockIndex[ERROR_BLOCKINDEX_LIMIT];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
void LOOKE_SD_DMA_XferError(DMA_HandleTypeDef *hdma);

/* Private functions ---------------------------------------------------------*/
void DelaySomeTime(void);
void DelayShortTime(void);

void ConfigSDMMC(void);
void ConfigHDMA(void);

/* Private functions ---------------------------------------------------------*/
uint8_t SPI_MPU9250_Init(void);

uint8_t SPI_MPU9250_SendByte(SPI_HandleTypeDef* SPI_Handle, uint8_t cmd, uint8_t data);
uint8_t SPI_MPU9250_ReadByte(SPI_HandleTypeDef* SPI_Handle, uint8_t reg);


void SPI_CS_Active(void);
void SPI_CS_Deactive(void);

void SPI_MPU9250_ReadAcc(void);

uint8_t cmd_readWhoIam;
uint8_t deviceID = 0xFF;

uint8_t acc_x_L = 0xFF;
uint8_t acc_x_H = 0xFF;
int16_t acc_x = 0x0000;

uint8_t acc_y_L = 0xFF;
uint8_t acc_y_H = 0xFF;
int16_t acc_y = 0x0000;

uint8_t acc_z_L = 0xFF;
uint8_t acc_z_H = 0xFF;
int16_t acc_z = 0x0000;


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
	
	waitCounter4SDMMC = 0x00000000;
  startBlockIndex = 0x00010000;
  timerCounter = 0;
	whileInSDCounter = 0;
	whileCounter = 0;
	
	TotalERRORCounter = 0;
	SDTransErrorCounter = 0;
	FIFOErrorCounter = 0;
	DMEErrorCounter = 0;
  TIMEOUTErrorCounter = 0;
  PARAMErrorCounter = 0;
  NOXFERErrorCounter = 0;
  NOTSUPPErrorCounter = 0;
	
	DMASendErrorCounter = 0;
	waitErrorCounter = 0;
	missOutCounter = 0;
	EraseError = 0;
	
	//GLOBAL_SDTRANSFER_SWITCH = 0;
	DMAERROR_SDTRANSFER = 0;
	
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Configure LED1 & LED3 */
  BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

  /* Configure User push-button button */
  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_EXTI);

  /* Wait for User push-button press before starting the Communication */
  //while (BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_SET)
  //{
  //  BSP_LED_Toggle(LED1);
  //  HAL_Delay(100);
  //}
  //BSP_LED_Off(LED1);
	
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.Mode = SPI_MODE_MASTER;
	
	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	DelaySomeTime();
	
	cmd_readWhoIam = MPU9250_SPI_READ_MASK | MPU9250_REG_WHO_AM_I;
	
  	
	SPI_CS_Active();
  deviceID = SPI_MPU9250_ReadByte(&SpiHandle,cmd_readWhoIam);
	SPI_CS_Deactive();
	
	DelayShortTime();
	
	SPI_CS_Active();
	deviceID = SPI_MPU9250_ReadByte(&SpiHandle,MPU9250_SPI_READ_MASK | MPU9250_REG_PWR_MGMT_1);
	SPI_CS_Deactive();
	
	DelayShortTime();
	
	SPI_CS_Active();
	deviceID = SPI_MPU9250_ReadByte(&SpiHandle,MPU9250_SPI_READ_MASK | MPU9250_REG_PWR_MGMT_2);
	SPI_CS_Deactive();
	
	DelayShortTime();
	SPI_MPU9250_Init();
	
	//Setup Main Status
	MAIN_STATUS = MAIN_STATUS_STOP;
	//Setup Global Cache
	SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_STOP;
  SD_File_Cache.TimeBase_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NOT_FULL;
  SD_File_Cache.TimeBase_Cache.CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_MASTER;
  SD_File_Cache.ARHS_Cache.CacheBufferState = LOOKE_SD_FILE_BUFFER_NOT_FULL;
  SD_File_Cache.ARHS_Cache.CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_MASTER;
	
  /*
	TimHandle_32bits.Instance = TIMx_32bits;
	//TimHandle_32bits.Init.Period            = 0xFFFFFFFF;
	TimHandle_32bits.Init.Period            = 1200000-1; //10 Sps / 100ms Period
	//TimHandle_32bits.Init.Period            = 6000000-1; //2 Sps / 500 ms Period
	//TimHandle_32bits.Init.Period            = 3000000-1; //4 Sps / 250 ms Period
	//TimHandle_32bits.Init.Period            = 12000000-1; //1 Sps / 1000 ms Period
	//TimHandle_32bits.Init.Prescaler         = 26;	// 108Mhz/27 = 4Mhz
	//TimHandle_32bits.Init.Prescaler         = 2;	// 108Mhz/3 = 36Mhz
	TimHandle_32bits.Init.Prescaler         = 8;	// 108Mhz/9 = 12Mhz
	//TimHandle_32bits.Init.Prescaler         = 107;	// 108Mhz/108 = 1Mhz
  TimHandle_32bits.Init.ClockDivision     = 0;
  TimHandle_32bits.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_32bits.Init.RepetitionCounter = 0;
  TimHandle_32bits.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  */
	
	TimHandle_32bits.Instance = TIMx_32bits;
	TimHandle_32bits.Init.Period            = 0xFFFFFFFF;
	//TimHandle_32bits.Init.Prescaler         = 26;	// 108Mhz/27 = 4Mhz
	TimHandle_32bits.Init.Prescaler         = 2;	// 108Mhz/3 = 36Mhz
  TimHandle_32bits.Init.ClockDivision     = 0;
  TimHandle_32bits.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_32bits.Init.RepetitionCounter = 0;
  TimHandle_32bits.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	//Input Capture Config
	TimsConfig.ICPrescaler = TIM_ICPSC_DIV1;
	TimsConfig.ICFilter = 0;
	TimsConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	TimsConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	
  //uint32_t ucPrescaler = (uint32_t)((SystemCoreClock/2)/1000000) - 1;
	TimHandle_ARHS.Instance = TIMx_ARHS;
	//TimHandle_ARHS.Init.Period            = 9999; // 1 SPS / 1000ms Period
	//TimHandle_ARHS.Init.Period            = 4999; // 2 SPS / 500ms Period
	TimHandle_ARHS.Init.Period            = 1999; // 5 SPS / 200ms Period
	//TimHandle_ARHS.Init.Period            = 999; // 10 SPS / 100ms Period
	
	//TimHandle_ARHS.Init.Prescaler         = 26;	// 108Mhz/27 = 4Mhz
	//TimHandle_ARHS.Init.Prescaler         = 2;	// 108Mhz/3 = 36Mhz
	//TimHandle_ARHS.Init.Prescaler         = 8;	// 108Mhz/9 = 12Mhz
	//TimHandle_ARHS.Init.Prescaler         = 107;	// 108Mhz/108 = 1Mhz
	TimHandle_ARHS.Init.Prescaler         = 10799;	// 108Mhz/10800 = 10Khz
	TimHandle_ARHS.Init.ClockDivision     = 0;
  TimHandle_ARHS.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_ARHS.Init.RepetitionCounter = 0;
  TimHandle_ARHS.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	//HAL_DBGMCU_EnableDBGStandbyMode();     
  //HAL_DBGMCU_EnableDBGStopMode();
	//__HAL_DBGMCU_FREEZE_TIM2();
	
  /*##-1- Configure the SD peripheral #######################################*/
	ConfigSDMMC();
	/*
  SDHandle_SDMMC.Instance = SDMMC1;
	SDHandle_SDMMC.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	//SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_ENABLE;
  SDHandle_SDMMC.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  SDHandle_SDMMC.Init.BusWide = SDMMC_BUS_WIDE_1B;
	SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	//SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  SDHandle_SDMMC.Init.ClockDiv = 0; //24Mhz
  //SDHandle_SDMMC.Init.ClockDiv = 1; //12Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 2; //8Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 3; //6Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 4; //4.8Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 5; //4Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 6; //3.4Mhz for Sony 4C run 4 hours
	//SDHandle_SDMMC.Init.ClockDiv = 7; //3Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 11; //2Mhz
	//SDHandle_SDMMC.Init.ClockDiv = 23; //1Mhz
	*/
	
	/*
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
  if (HAL_SD_GetCardInfo(&SDHandle_SDMMC, &SDCardInfo) != HAL_OK)
	{
		Error_Handler();
	}
	DelaySomeTime();
	if (HAL_SD_GetCardCSD(&SDHandle_SDMMC, &SDCardCSDInfo) != HAL_OK)
	{
		Error_Handler();
	}
	DelaySomeTime();
	if (HAL_SD_GetCardCID(&SDHandle_SDMMC, &SDCardCIDInfo) != HAL_OK)
	{
		Error_Handler();
	}
	DelaySomeTime();
	//Erase SD Card Block
	if (HAL_SD_Erase(&SDHandle_SDMMC,0x00000000,0x000FFFFF) != HAL_OK)
	{
	  Error_Handler();
	}
	// Configure the SD DPSM (Data Path State Machine)
	//SDMMC_DataInitTypeDef config;
  //config.DataTimeOut   = SDMMC_DATATIMEOUT;
  //config.DataLength    = 64;
  //config.DataBlockSize = SDMMC_DATABLOCK_SIZE_64B;
  //config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
  //config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
  //config.DPSM          = SDMMC_DPSM_ENABLE;
  //SDMMC_ConfigData(SDHandle_SDMMC1.Instance, &config);
	//errorstate = SDMMC_CmdBlockLength(SDHandle_SDMMC1.Instance, 64);
	DelaySomeTime();
	DelaySomeTime();
  if (HAL_SD_GetCardStatus(&SDHandle_SDMMC, &SDCardStatus) != HAL_OK)
	{
		Error_Handler();
	}
	*/
	
	//Config DMA Channel for SDMMC2
	//hdma_sdmmc.Instance = DMA2_Stream5;
	//hdma_sdmmc.Init.Channel = DMA_CHANNEL_11;

	//Config DMA Channel for SDMMC1
	ConfigHDMA();
	/*
	hdma_sdmmc.Instance = DMA2_Stream6;
  //hdma_sdmmc.Instance = DMA2_Stream3;
	hdma_sdmmc.Init.Channel = DMA_CHANNEL_4;
	
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
	//hdma_sdmmc.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_sdmmc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_sdmmc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	
	hdma_sdmmc.Init.MemBurst = DMA_MBURST_INC4;
	hdma_sdmmc.Init.PeriphBurst = DMA_PBURST_INC4;
	hdma_sdmmc.XferErrorCallback = LOOKE_SD_DMA_XferError;
	
	*/
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
  //HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 1);
  //HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	
	/* DMA2_Channel4_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	
  
	//Format SD File SYSPara
	SD_FileSysParaUnion.FileSysPara.NumberOfMeasurementSection = 0;
	SD_FileSysParaUnion.FileSysPara.SectionIndexArray[0].SectionStartBlock = 0;
	SD_FileSysParaUnion.FileSysPara.SectionIndexArray[0].SectionEndBlock = 0;
	/*
  if(LOOKE_SD_File_WriteSysPara(&SDHandle_SDMMC, &SD_FileSysParaUnion) != HAL_OK)
	{
	  Error_Handler();
	}
	DelaySomeTime();
	if(LOOKE_SD_File_CreateMeasureSection(&SDHandle_SDMMC, &SD_FileSysParaUnion) != HAL_OK)
	{
	  Error_Handler();
	}
	DelaySomeTime();
	//Init SD File SYSPara
	if(LOOKE_SD_File_ReadSysPara(&SDHandle_SDMMC, &SD_FileSysParaUnion) != HAL_OK)
	{
	  BSP_LED_Toggle(LED2);
	}
	*/
	
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
	for(initForBuffer = 0 ; initForBuffer<LOOKE_SD_FILE_BLOCK_SIZE*LOOKE_SD_FILE_CACHE_SIZE; initForBuffer++)
	{
		aBuffer_Block_Tx[initForBuffer] = initForBuffer;
		//aBuffer_Block_Tx[initForBuffer] = 0;
	}
	*/
	
	//uint8_t testValue = 0x01;
	//for(initForBuffer = 4 ; initForBuffer<LOOKE_SD_FILE_BLOCK_SIZE; )
	//{
	//	aBuffer_Block_Tx[initForBuffer] = testValue;
	//	initForBuffer = initForBuffer+4;
	//	aBuffer_Block_Tx[initForBuffer] = testValue;
	//	initForBuffer = initForBuffer+4;
	//	testValue++;
	//}
	//aBuffer_Block_Tx[508] = 0x00;
	//aBuffer_Block_Tx[509] = 0x00;
	//aBuffer_Block_Tx[510] = 0x00;
	//aBuffer_Block_Tx[511] = 0x00;
	
	SCB_CleanDCache();
	/*
	//startTimeStamp = __HAL_TIM_GET_COUNTER(&TimHandle_32bits);
	
	if(HAL_SD_WriteBlocks_DMA(&SDHandle_SDMMC, aBuffer_Block_Tx, 0x10, LOOKE_SD_FILE_CACHE_SIZE) == HAL_OK)
	{
	  //SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
		//Error_Handler();
		DelaySomeTime();
		DelaySomeTime();
		//DelaySomeTime();
	}
  
	//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
	*/
	
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
	
	/*
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
	*/
	
	
	//Set Timer for test case
	if (HAL_TIM_Base_Init(&TimHandle_ARHS) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }
	
	/*
	//Start Time Base
	if (HAL_TIM_Base_Start_IT(&TimHandle_ARHS) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }
	*/
	
	
	if (HAL_TIM_IC_Init(&TimHandle_32bits) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }
	
	if(HAL_TIM_IC_ConfigChannel(&TimHandle_32bits, &TimsConfig, TIM_CHANNEL_1) != HAL_OK)
  {
		// Initialization Error
    Error_Handler();
  }
	
	/*
	if (HAL_TIM_IC_Start_IT(&TimHandle_32bits,TIM_CHANNEL_1) != HAL_OK)
  {
    // Starting Error
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
		
		/*
		//Check Cache Globle State and Buffer State. Start Sync Process if Need;
		if(SD_File_Cache.TimeBase_Cache.CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC && SD_File_Cache.CurrentGlobalState == LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER)
		{

			//Sync Process
		  LOOKE_SD_FILE_SYNC_TRANSFER_RESULT syncResult = LOOKE_SD_File_SyncCacheToSDCard_TimeBase(&SDHandle_SDMMC, &SD_FileSysParaUnion.FileSysPara, &SD_File_Cache);

		}
		*/
		
		//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
		//BSP_LED_Toggle(LED3);
		/*
		if(waitErrorCounter > 0)
		{
			//restart SDMMC
			if(HAL_SD_DeInit(&SDHandle_SDMMC) == HAL_OK)
			{
        SDHandle_SDMMC.Instance = SDMMC2;
	      SDHandle_SDMMC.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
        SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
        SDHandle_SDMMC.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
        SDHandle_SDMMC.Init.BusWide = SDMMC_BUS_WIDE_1B;
	      SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
        SDHandle_SDMMC.Init.ClockDiv = 0; //24Mhz
				
			  if (HAL_SD_Init(&SDHandle_SDMMC) == HAL_OK)
        {
				  DelaySomeTime();
					
					if (HAL_SD_ConfigWideBusOperation(&SDHandle_SDMMC, SDMMC_BUS_WIDE_4B) == HAL_OK )
	        {
		        waitErrorCounter = 0;
	        }
				}
			}
		}
		*/
		whileCounter++;
		
		/*
		if(SDTransErrorCounter > 0)
		{
			//Restart SDMMC
		  if  (HAL_SD_Abort(&SDHandle_SDMMC) != HAL_OK)
			{
			  Error_Handler();
			}
			if (HAL_SD_DeInit(&SDHandle_SDMMC) != HAL_OK)
			{
			  Error_Handler();
			}
			ConfigSDMMC();
			DelaySomeTime();
			if (HAL_SD_Init(&SDHandle_SDMMC ) != HAL_OK)
      {
        Error_Handler();
      }
      DelaySomeTime();
      if (HAL_SD_ConfigWideBusOperation(&SDHandle_SDMMC, SDMMC_BUS_WIDE_4B) != HAL_OK)
	    {
		    Error_Handler();
	    }
			
			// DMA2 Stream6 
	    HAL_DMA_DeInit(&hdma_sdmmc);
	    
			ConfigHDMA();
			
	    if (HAL_DMA_Init(&hdma_sdmmc) != HAL_OK)
      {
        Error_Handler();
      }
			
			// DMA interrupt init //
      // DMA2_Channel11_5_IRQn interrupt configuration //
			HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
      HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 3, 1);
      HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
			
	    __HAL_LINKDMA(&SDHandle_SDMMC,hdmatx,hdma_sdmmc);
			
			SDTransErrorCounter = 0;
			
		}
		*/
		//SDCardState = HAL_SD_GetCardState(&SDHandle_SDMMC);
		
		if(MAIN_STATUS == MAIN_STATUS_START)
		{
			whileInSDCounter++;
			
		  //Check Cache Globle State and Buffer State. Start Sync Process if Need;
		  if(SD_File_Cache.TimeBase_Cache.CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC && SD_File_Cache.CurrentGlobalState == LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER)
		  {
			  //Sync Process
		    LOOKE_SD_FILE_SYNC_TRANSFER_RESULT syncResult = LOOKE_SD_File_SyncCacheToSDCard_TimeBase(&SDHandle_SDMMC, &SD_FileSysParaUnion.FileSysPara, &SD_File_Cache);
		    break;
			}
			
		  //Check Cache Globle State and Buffer State. Start Sync Process if Need;
		  if(SD_File_Cache.ARHS_Cache.CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC && SD_File_Cache.CurrentGlobalState == LOOKE_SD_FILE_GLOBAL_CACHE_STATE_TRANSFER)
		  {
			  //Sync Process
		    LOOKE_SD_FILE_SYNC_TRANSFER_RESULT syncResult = LOOKE_SD_File_SyncCacheToSDCard_ARHS(&SDHandle_SDMMC, &SD_FileSysParaUnion.FileSysPara, &SD_File_Cache);
		    break;
			}
			
			/*
		  //startTimeStamp = __HAL_TIM_GET_COUNTER(&TimHandle_32bits);
			
			if(HAL_SD_GetCardState(&SDHandle_SDMMC) == HAL_SD_CARD_TRANSFER)
		  {
		    //if( HAL_SD_WriteBlocks_DMA(&SDHandle_SDMMC, aBuffer_Block_Tx, startBlockIndex, LOOKE_SD_FILE_CACHE_SIZE) != HAL_OK )
	      //{
	      //  DMASendErrorCounter++;
	      //}
		  }
		  else
		  {
		    missOutCounter++;
		  }
			
			//endTimeStamp = __HAL_TIM_GET_COUNTER(&TimHandle_32bits);
			*/
		}
		
		if(MAIN_STATUS == MAIN_STATUS_STOP)
		{
			//Check Cache Globle State. Start final step Sync Process if Need;
		  if(SD_File_Cache.TimeBase_Cache.CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC)
		  {
			  //Sync Process
		    LOOKE_SD_FILE_SYNC_TRANSFER_RESULT syncResult = LOOKE_SD_File_SyncCacheToSDCard_TimeBase(&SDHandle_SDMMC, &SD_FileSysParaUnion.FileSysPara, &SD_File_Cache);
		    break;
			}
			
		  //Check Cache Globle State and Buffer State. Start Sync Process if Need;
		  if(SD_File_Cache.ARHS_Cache.CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC)
		  {
			  //Sync Process
		    LOOKE_SD_FILE_SYNC_TRANSFER_RESULT syncResult = LOOKE_SD_File_SyncCacheToSDCard_ARHS(&SDHandle_SDMMC, &SD_FileSysParaUnion.FileSysPara, &SD_File_Cache);
		    break;
			}
			
		}
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

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
	if(TotalERRORCounter < 10)
	{
		errorBlockIndex[TotalERRORCounter] = startBlockIndex;
	}
	
	TotalERRORCounter++;
	
	if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_TE)
	{
	  SDTransErrorCounter++;
	}
	
	if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_FE)
	{
	  FIFOErrorCounter++;
	}
	
	if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_DME)
	{
	  DMEErrorCounter++;
	}
	
  if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_TIMEOUT)
	{
	  TIMEOUTErrorCounter++;
	}
	
	if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_PARAM)
	{
	  PARAMErrorCounter++;
	}
	
  if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_NO_XFER)
	{
	  NOXFERErrorCounter++;
	}
	
	if(hsd->hdmatx->ErrorCode == HAL_DMA_ERROR_NOT_SUPPORTED)
	{
	  NOTSUPPErrorCounter++;
	}
	
	//SD TX Transfer ERROR,Switch SD Cache Global State
	//SD_File_Cache.CurrentGlobalState = LOOKE_SD_FILE_GLOBAL_CACHE_STATE_SYNC_ERROR;
	
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

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
	//endTimeStamp = __HAL_TIM_GET_COUNTER(&TimHandle_32bits);
	
	/*
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
	*/
	
	/*
  uint16_t waitCounter = 0x0000;
	while (waitCounter < 0xFFFF)
	{
		if(HAL_SD_GetCardState(&SDHandle_SDMMC) == HAL_SD_CARD_TRANSFER)
		{
			startBlockIndex += LOOKE_SD_FILE_CACHE_SIZE;
			waitCounter4SDMMC = waitCounter;
			return;
		}
		waitCounter++;
		DelayShortTime();
	}
	waitErrorCounter++;
	*/
}

/**
  * @brief  DMA Error Callback for SD card DMA Transfer
  * @param  hdma : DMA handle
  * @retval None
  */
//void LOOKE_SD_DMA_XferError(DMA_HandleTypeDef *hdma)
//{
//  DMATransErrorCounter++;
//	DMAERROR_SDTRANSFER = 1;
//}
	

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*
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
	
	*/
	
	//Set DMA Write OP Switch
	//GLOBAL_SDTRANSFER_SWITCH = 1;
	//BSP_LED_Toggle(LED2);
	//timerCounter++;
	//whileCounter = 0;
	//SCB_CleanDCache();
	//uint16_t waitCounter = 0x0000;
	//while (waitCounter < 0xFFFF)
	//{
		
    //waitCounter++;
	//}
	//if(waitCounter > waitCounter4SDMMC)
	//{
	//	waitCounter4SDMMC = waitCounter;
	//}
	//waitErrorCounter++;
	
	SPI_MPU9250_ReadAcc();
	
	arhsDataTest.ACC_X = acc_x;
	arhsDataTest.ACC_Y = acc_y;
	arhsDataTest.ACC_Z = acc_z;
	
	//LOOKE_SD_File_AddARHSMeasureToCache(&SD_File_Cache ,&arhsDataTest);
	
}

/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	

  if(htim->Instance == TIMx_32bits)
	{
	  BSP_LED_Toggle(LED2);
	  captureNumber++;
		
	  autoReloadValue = __HAL_TIM_GET_AUTORELOAD(htim);
	
	  timeBaseDataTest.DataIndex = captureNumber;
	  timeBaseDataTest.TimeStamp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);;
	
	  //LOOKE_SD_File_AddTimeBaseMeasureToCache(&SD_File_Cache,&timeBaseDataTest);
	}
	
	
}

void ConfigSDMMC()
{
  SDHandle_SDMMC.Instance = SDMMC1;
	SDHandle_SDMMC.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	//SDHandle_SDMMC.Init.ClockBypass = SDMMC_CLOCK_BYPASS_ENABLE;
  SDHandle_SDMMC.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  SDHandle_SDMMC.Init.BusWide = SDMMC_BUS_WIDE_1B;
	//SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	SDHandle_SDMMC.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
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

void ConfigHDMA()
{
	//hdma_sdmmc.Instance = DMA2_Stream6;
  hdma_sdmmc.Instance = DMA2_Stream3;
	hdma_sdmmc.Init.Channel = DMA_CHANNEL_4;
	
	//hdma_sdmmc.Instance = DMA2_Stream5;
	//hdma_sdmmc.Init.Channel = DMA_CHANNEL_11;
	
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
	//hdma_sdmmc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_sdmmc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	
	hdma_sdmmc.Init.MemBurst = DMA_MBURST_INC4;
	hdma_sdmmc.Init.PeriphBurst = DMA_PBURST_INC4;
	
	//hdma_sdmmc.XferErrorCallback = LOOKE_SD_DMA_XferError;
}


uint8_t SPI_MPU9250_SendByte(SPI_HandleTypeDef* SPI_Handle, uint8_t cmd, uint8_t data)
{
	uint8_t status = 0;

	if (HAL_SPI_TransmitReceive(SPI_Handle, &cmd, &status, 1, (uint32_t)100) == HAL_OK)
		if (HAL_SPI_TransmitReceive(SPI_Handle, &data, &status, 1, (uint32_t)100) == HAL_OK)
		return status;
	return 0;
}

uint8_t SPI_MPU9250_ReadByte(SPI_HandleTypeDef* SPI_Handle, uint8_t reg)
{
	uint8_t data = 0xFF;
	uint8_t temp = 0xFF;
	if (HAL_SPI_TransmitReceive(SPI_Handle, &reg, &data, 1, (uint32_t)100) == HAL_OK)
		if(HAL_SPI_TransmitReceive(SPI_Handle, &temp, &data, 1, (uint32_t)100) == HAL_OK)
	    return data;
	return 0xFF;
}

void SPI_CS_Active()
{
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_RESET);
}

void SPI_CS_Deactive()
{
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
}

/*
	init MPU9250 for ACC\Gyro\Mag
*/
uint8_t SPI_MPU9250_Init()
{
	uint8_t test = 0xFF;

	//Clock source auto select
	SPI_CS_Active();
	SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_PWR_MGMT_1, 0x01); 
  SPI_CS_Deactive();

	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_PWR_MGMT_1); 
	SPI_CS_Deactive();
	
	DelayShortTime();
	
	//Enable Acc&Gyro
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_PWR_MGMT_2, 0x00); 
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_PWR_MGMT_2); 
	SPI_CS_Deactive();
  DelayShortTime();
		
  //Bandwidth 250Hz/Sample Rate;1kHz if F_choice_b=00
  SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_CONFIG, 0x00);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_CONFIG);
	SPI_CS_Deactive();
	DelayShortTime();
	
  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) if F_choice_b=00
	SPI_CS_Active();
	SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_SMPLRT_DIV, 0x00);
	SPI_CS_Deactive();
  
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_SMPLRT_DIV);
	SPI_CS_Deactive();
	DelayShortTime();
	
  //Gyro Full Scale:250dps{0x00}/F_choice_b:11
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_GYRO_CONFIG, 0x03);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_GYRO_CONFIG);
	SPI_CS_Deactive();
	DelayShortTime();
	
	 //XYZ SelfTest OFF/ Full Scale:2g{0x00}
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle,MPU9250_REG_ACCEL_CONFIG, 0x00);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
  test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_ACCEL_CONFIG);
	SPI_CS_Deactive();
	DelayShortTime();
	
	//DLPF 99hz / 3DB 1Khz F_choice_b:0
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_ACCEL_CONFIG2, 0x02);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
  test = SPI_MPU9250_ReadByte(&SpiHandle,MPU9250_SPI_READ_MASK|MPU9250_REG_ACCEL_CONFIG2);
	SPI_CS_Deactive();
	
	return test;
}

/*
  Read ACC X,Y,Z from MPU9250
*/
void SPI_MPU9250_ReadAcc(void)
{
  uint8_t data = 0xFF;
	uint8_t temp = 0xFF;
	uint8_t reg = MPU9250_SPI_READ_MASK|MPU9250_REG_ACCEL_XOUT_H;
	
	SPI_CS_Active();

  //testArray_CapCounter_preSPI[counter_3axis] = counter_cap;
	
	if (HAL_SPI_TransmitReceive(&SpiHandle, &reg, &data, 1, (uint32_t)100) == HAL_OK)
	{
		HAL_SPI_TransmitReceive(&SpiHandle, &temp, &acc_x_H, 1, (uint32_t)100);
	  HAL_SPI_TransmitReceive(&SpiHandle, &temp, &acc_x_L, 1, (uint32_t)100);
		 
		HAL_SPI_TransmitReceive(&SpiHandle, &temp, &acc_y_H, 1, (uint32_t)100);
	  HAL_SPI_TransmitReceive(&SpiHandle, &temp, &acc_y_L, 1, (uint32_t)100);
		
		HAL_SPI_TransmitReceive(&SpiHandle, &temp, &acc_z_H, 1, (uint32_t)100);
	  HAL_SPI_TransmitReceive(&SpiHandle, &temp, &acc_z_L, 1, (uint32_t)100);
	}
	SPI_CS_Deactive();
	
	acc_x = 0x0000;
	acc_x = acc_x | acc_x_H;
	acc_x = acc_x << 8;
	acc_x = acc_x | acc_x_L;
	
	acc_y = 0x0000;
	acc_y = acc_y | acc_y_H;
	acc_y = acc_y << 8;
	acc_y = acc_y | acc_y_L;
	
	acc_z = 0x0000;
	acc_z = acc_z | acc_z_H;
	acc_z = acc_z << 8;
	acc_z = acc_z | acc_z_L;
	
	/*
	if(counter_3axis < TEST_LIMIT)
	{
		//testArray_Period_Before[counter_3axis] = period_before;
	  //testArray_Period_Current[counter_3axis] = period_current;
    //testArray_Period_Diff[counter_3axis] = period_Diff;
    testArray_CapCounter[counter_3axis] = counter_cap;
		testArray_ACC_X[counter_3axis] = acc_x;
		testArray_ACC_Y[counter_3axis] = acc_y;
		testArray_ACC_Z[counter_3axis] = acc_z;
	}
	*/
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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
