/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    30-December-2016
  * @brief   This sample code shows how to use STM32F7xx SPI HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          Polling transfer.
  *          The communication is done using 2 Boards.
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

TIM_HandleTypeDef    TimHandle_3Axis;
TIM_HandleTypeDef    TimHandle_32bits;

//TIM_MasterConfigTypeDef TimMasterConfig;
//TIM_SlaveConfigTypeDef  TimSlaveConfig;

TIM_IC_InitTypeDef   TimsConfig;

/* Prescaler declaration */
__IO uint32_t uwPrescalerValue = 0;

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup SPI_FullDuplex_ComPolling
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Uncomment this line to use the board as master, if not it is used as slave */
#define MASTER_BOARD

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* Buffer used for transmission */
//uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Polling **** SPI Message ******** SPI Message ******** SPI Message ****";

/* Buffer used for reception */
//uint8_t aRxBuffer[BUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void Timeout_Error_Handler(void);
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/
uint8_t SPI_MPU9250_SendByte(SPI_HandleTypeDef* SPI_Handle, uint8_t cmd, uint8_t data);
uint8_t SPI_MPU9250_ReadByte(SPI_HandleTypeDef* SPI_Handle, uint8_t reg);

uint8_t SPI_MPU9250_Init(void);

void SPI_MPU9250_ReadAcc(void);
//void SPI_MPU9250_ReadAcc_Y(void);
//void SPI_MPU9250_ReadAcc_Z(void);


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

void SPI_CS_Active(void);
void SPI_CS_Deactive(void);


//uint32_t uwIC1Value1_Master = 0;
//uint32_t uwIC1Value2_Master = 0;
//uint32_t uwDiffCapture_Master = 0;

//uint32_t uwIC1Value1_Slave = 0;
//uint32_t uwIC1Value2_Slave = 0;
//uint32_t uwDiffCapture_Slave = 0;

uint32_t uwIC1Value1_32bits = 0;
uint32_t uwIC1Value2_32bits = 0;
uint32_t uwDiffCapture_32bits = 0;

uint32_t counter_cap = 0;
uint32_t counter_3axis = 0;

uint32_t testArray_Period[TEST_LIMIT];
int testArray_Diff[TEST_LIMIT];
uint32_t period_before = 0;
uint32_t period_current = 0;
int period_Diff = 0;

uint32_t testArray_Period_Before[TEST_LIMIT];
uint32_t testArray_Period_Current[TEST_LIMIT];
int testArray_Period_Diff[TEST_LIMIT];

int16_t testArray_ACC_X[TEST_LIMIT];
int16_t testArray_ACC_Y[TEST_LIMIT];
int16_t testArray_ACC_Z[TEST_LIMIT];
uint32_t testArray_CapCounter_preSPI[TEST_LIMIT];
uint32_t testArray_CapCounter[TEST_LIMIT];

int isStart = 0;
/**
  * @brief  Main program.
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

  /* Configure LED1, LED2 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);

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

#ifdef MASTER_BOARD
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
#else
  SpiHandle.Init.Mode = SPI_MODE_SLAVE;
#endif /* MASTER_BOARD */

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

#ifdef MASTER_BOARD

  /* Configure User push-button button */
  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_GPIO);

  /* Wait for User push-button press before starting the Communication */
  while (BSP_PB_GetState(BUTTON_USER) != GPIO_PIN_SET)
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(100);
  }
  BSP_LED_Off(LED1);
#endif /* MASTER_BOARD */

  /*##-2- Start the Full Duplex Communication process ########################*/  
  /* While the SPI in TransmitReceive process, user can transmit data through 
     "aTxBuffer" buffer & receive data through "aRxBuffer" */
  /* Timeout is set to 5S */
  
//  switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE, 5000))
//  {
//    case HAL_OK:
      /* Communication is completed ___________________________________________ */
      /* Compare the sent and received buffers */
//      if (Buffercmp((uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE))
//      {
        /* Transfer error in transmission process */
//        Error_Handler();
//      }
      /* Turn LED1 on: Transfer in transmission process is correct */
//      BSP_LED_On(LED1);
      /* Turn LED2 on: Transfer in reception process is correct */
//      BSP_LED_On(LED2);
//      break;

//    case HAL_TIMEOUT:
      /* A Timeout Occur ______________________________________________________*/
      /* Call Timeout Handler */
//      Timeout_Error_Handler();
//      break;
      /* An Error Occur ______________________________________________________ */
//    case HAL_ERROR:
      /* Call Timeout Handler */
//      Error_Handler();
//      break;
//    default:
//      break;
//  }
	cmd_readWhoIam = MPU9250_SPI_READ_MASK | MPU9250_REG_WHO_AM_I;
	
  	
	SPI_CS_Active();
  deviceID = SPI_MPU9250_ReadByte(&SpiHandle,cmd_readWhoIam);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	deviceID = SPI_MPU9250_ReadByte(&SpiHandle,MPU9250_SPI_READ_MASK | MPU9250_REG_PWR_MGMT_1);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	deviceID = SPI_MPU9250_ReadByte(&SpiHandle,MPU9250_SPI_READ_MASK | MPU9250_REG_PWR_MGMT_2);
	SPI_CS_Deactive();
	
	
	SPI_MPU9250_Init();
	
	uwPrescalerValue = (uint32_t)((SystemCoreClock/ 2) / 10000) - 1;
	TimHandle_3Axis.Instance = TIMx_3Axis;
	TimHandle_3Axis.Init.Period            = 10-1; //1000sps
  TimHandle_3Axis.Init.Prescaler         = uwPrescalerValue;
  TimHandle_3Axis.Init.ClockDivision     = 0;
  TimHandle_3Axis.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_3Axis.Init.RepetitionCounter = 0;
  TimHandle_3Axis.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	if (HAL_TIM_Base_Init(&TimHandle_3Axis) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	/*## Configure the TIM peripheral #######################################*/
	TimHandle_32bits.Instance = TIMx_32bits;
	TimHandle_32bits.Init.Period            = 0xFFFFFFFF;
	//TimHandle_32bits.Init.Prescaler       = 26;	// 108Mhz/27 = 4Mhz
	TimHandle_32bits.Init.Prescaler         = 2;	// 108Mhz/3 = 36Mhz
  TimHandle_32bits.Init.ClockDivision     = 0;
  TimHandle_32bits.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle_32bits.Init.RepetitionCounter = 0;
  TimHandle_32bits.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	if (HAL_TIM_IC_Init(&TimHandle_32bits) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

	//Input Capture Config
	TimsConfig.ICPrescaler = TIM_ICPSC_DIV1;
	TimsConfig.ICFilter = 0;
	TimsConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	TimsConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	
	if(HAL_TIM_IC_ConfigChannel(&TimHandle_32bits, &TimsConfig, TIM_CHANNEL_1) != HAL_OK)
  {
		/* Initialization Error */
    Error_Handler();
  }
	
	if (HAL_TIM_IC_Start_IT(&TimHandle_32bits,TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
	
	if (HAL_TIM_Base_Start_IT(&TimHandle_3Axis) != HAL_OK)
  {
    /* Starting Error */
     Error_Handler();
  }
	
  /* Infinite loop */
  while (1)
  {
		//SPI_MPU9250_ReadAcc();
		//HAL_Delay(100);
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
  while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Timeout_Error_Handler(void)
{
  /* Toggle LED3 on */
  while(1)
  {
    BSP_LED_On(LED3);
    HAL_Delay(500);
    BSP_LED_Off(LED3);
    HAL_Delay(500);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
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
};

void SPI_CS_Deactive()
{
  HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
};



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
	
	//Enable Acc&Gyro
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_PWR_MGMT_2, 0x00); 
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_PWR_MGMT_2); 
	SPI_CS_Deactive();

  //Bandwidth 250Hz/Sample Rate;1kHz if F_choice_b=00
  SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_CONFIG, 0x00);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_CONFIG);
	SPI_CS_Deactive();
	
  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) if F_choice_b=00
	SPI_CS_Active();
	SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_SMPLRT_DIV, 0x00);
	SPI_CS_Deactive();
  
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_SMPLRT_DIV);
	SPI_CS_Deactive();

  //Gyro Full Scale:250dps{0x00}/F_choice_b:11
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_GYRO_CONFIG, 0x03);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
	test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_GYRO_CONFIG);
	SPI_CS_Deactive();

	 //XYZ SelfTest OFF/ Full Scale:2g{0x00}
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle,MPU9250_REG_ACCEL_CONFIG, 0x00);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
  test = SPI_MPU9250_ReadByte(&SpiHandle, MPU9250_SPI_READ_MASK|MPU9250_REG_ACCEL_CONFIG);
	SPI_CS_Deactive();
	
	//DLPF 99hz / 3DB 1Khz F_choice_b:0
	SPI_CS_Active();
  SPI_MPU9250_SendByte(&SpiHandle, MPU9250_REG_ACCEL_CONFIG2, 0x02);
	SPI_CS_Deactive();
	
	SPI_CS_Active();
  test = SPI_MPU9250_ReadByte(&SpiHandle,MPU9250_SPI_READ_MASK|MPU9250_REG_ACCEL_CONFIG2);
	SPI_CS_Deactive();
	
	return test;
};

/*
  Read ACC X,Y,Z from MPU9250
*/
void SPI_MPU9250_ReadAcc(void)
{
  uint8_t data = 0xFF;
	uint8_t temp = 0xFF;
	uint8_t reg = MPU9250_SPI_READ_MASK|MPU9250_REG_ACCEL_XOUT_H;
	
	SPI_CS_Active();

  testArray_CapCounter_preSPI[counter_3axis] = counter_cap;
	
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
};


/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//if(isStart)
	//{
	  //
	if(htim->Instance == TIMx_3Axis)
	{
		if(counter_3axis < TEST_LIMIT)
			counter_3axis++;
		
		BSP_LED_Toggle(LED1);
	  SPI_MPU9250_ReadAcc();
	}


	//}

}


/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
  if(counter_cap == 0)
	{
		uwIC1Value1_32bits=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		testArray_Period[counter_cap] = 0;
		testArray_Period_Diff[counter_cap] = 0;
		
		period_before = uwIC1Value1_32bits;
		period_current = uwIC1Value1_32bits;
		//testArray_Period_Diff = 0;
		counter_cap++;
		isStart = 1;
	}
	else if(counter_cap < TEST_LIMIT)
	{
		uwIC1Value2_32bits=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		
		if(uwIC1Value2_32bits > uwIC1Value1_32bits)
		{
		  uwDiffCapture_32bits = uwIC1Value2_32bits - uwIC1Value1_32bits; 
		}
		else
		{
			uwDiffCapture_32bits = (__HAL_TIM_GET_AUTORELOAD(htim) -uwIC1Value1_32bits + 1) + uwIC1Value2_32bits; 
		}
		uwIC1Value1_32bits = uwIC1Value2_32bits;
		testArray_Period[counter_cap] = uwDiffCapture_32bits;
		
		testArray_Period_Diff[counter_cap] = uwDiffCapture_32bits - testArray_Period[counter_cap-1];
		//period_before = period_current;
		//period_current = uwDiffCapture_32bits;
		//period_Diff = period_current - period_before;
		counter_cap++;
	}	
	//else
	//{
		//BSP_LED_Toggle(LED1);
	//}

}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
