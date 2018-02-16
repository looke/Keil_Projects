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
TIM_HandleTypeDef    TimHandle_Master;
TIM_HandleTypeDef    TimHandle_Slave;

TIM_HandleTypeDef    TimHandle_32bits;

TIM_MasterConfigTypeDef TimMasterConfig;
TIM_SlaveConfigTypeDef  TimSlaveConfig;

TIM_IC_InitTypeDef   TimsConfig;

GPIO_InitTypeDef   GPIO_InitStruct;

uint32_t uwIC1Value1_Master = 0;
uint32_t uwIC1Value2_Master = 0;
uint32_t uwDiffCapture_Master = 0;

uint32_t uwIC1Value1_Slave = 0;
uint32_t uwIC1Value2_Slave = 0;
uint32_t uwDiffCapture_Slave = 0;

uint32_t uwIC1Value1_32bits = 0;
uint32_t uwIC1Value2_32bits = 0;
uint32_t uwDiffCapture_32bits = 0;

uint32_t counter_cap = 0;


uint32_t testArray_Period[TEST_LIMIT];
int testArray_Period_Diff[TEST_LIMIT];

/* Prescaler declaration */
__IO uint32_t uwPrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
/* Private functions ---------------------------------------------------------*/

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
  BSP_LED_Init(LED3);

  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is equal to 4.
      TIM3CLK = PCLK1*2
      PCLK1 = HCLK/4
      => TIM3CLK = HCLK/2 = SystemCoreClock/2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as follows:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock/2) /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f7xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)((SystemCoreClock/ 2) / 1000000) - 1;

  /* Set TIMx instance TIMx=TIM3*/
  //TimHandle_Master.Instance = TIMx_Master;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  //TimHandle.Init.Period            = 20000 - 1;
	//TimHandle_Master.Init.Period            = 0xFFFFFFFF;
  //TimHandle_Master.Init.Prescaler         = uwPrescalerValue;
	//TimHandle_Master.Init.Prescaler         = 26;
  //TimHandle_Master.Init.ClockDivision     = 0;
  //TimHandle_Master.Init.CounterMode       = TIM_COUNTERMODE_UP;
  //TimHandle_Master.Init.RepetitionCounter = 0;
  //TimHandle_Master.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
  //if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //}
	// init tim master in input capture mode
	//if (HAL_TIM_IC_Init(&TimHandle_Master) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //}
	
	// init tim slave in input capture mode
	//TimHandle_Slave.Instance = TIMx_Slave;
	//TimHandle_Slave.Init.Period            = 0xFFFFFFFF;
  //TimHandle_Slave.Init.Prescaler         = 0x0;
  //TimHandle_Slave.Init.ClockDivision     = 0;
  //TimHandle_Slave.Init.CounterMode       = TIM_COUNTERMODE_UP;
  //TimHandle_Slave.Init.RepetitionCounter = 0;
  //TimHandle_Slave.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	//if (HAL_TIM_IC_Init(&TimHandle_Slave) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //}
	
	
	TimHandle_32bits.Instance = TIMx_32bits;
	TimHandle_32bits.Init.Period            = 0xFFFFFFFF;
	//TimHandle_32bits.Init.Prescaler         = 26;	// 108Mhz/27 = 4Mhz
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
	
	//if(HAL_TIM_IC_ConfigChannel(&TimHandle_Master, &TimsConfig, TIM_CHANNEL_1) != HAL_OK)
  //{
		/* Initialization Error */
  //  Error_Handler();
  //}
	
	//if(HAL_TIM_IC_ConfigChannel(&TimHandle_Slave, &TimsConfig, TIM_CHANNEL_1) != HAL_OK)
  //{
		/* Initialization Error */
  //  Error_Handler();
  //}
	
	if(HAL_TIM_IC_ConfigChannel(&TimHandle_32bits, &TimsConfig, TIM_CHANNEL_1) != HAL_OK)
  {
		/* Initialization Error */
    Error_Handler();
  }
	
	//if(HAL_TIM_IC_ConfigChannel(&TimHandle, &TimsConfig, TIM_CHANNEL_2) != HAL_OK)
  //{
	//	/* Initialization Error */
  //  Error_Handler();
  //}	
	
	//TIM 32bit as Master
	//TimSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	//TimSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	//TimSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	//TimSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	//if (HAL_TIM_SlaveConfigSynchronization(&TimHandle_32bits, &TimSlaveConfig) != HAL_OK)  
  //{  
  //  Error_Handler();  
  //}  
	
	/*
	//TIM3 as Master
	TimSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	TimSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	TimSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	TimSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	if (HAL_TIM_SlaveConfigSynchronization(&TimHandle_Master, &TimSlaveConfig) != HAL_OK)  
  {  
    Error_Handler();  
  }  
	
	TimMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	TimMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle_Master, &TimMasterConfig) != HAL_OK)  
  {  
    Error_Handler();  
  } 
	
	/////TIM2 as Slave
  TimSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	TimSlaveConfig.InputTrigger = TIM_TS_ITR2;
	TimSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	TimSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	if (HAL_TIM_SlaveConfigSynchronization(&TimHandle_Slave, &TimSlaveConfig) != HAL_OK)  
  {  
    Error_Handler();  
  }  
	
	TimMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle_Slave, &TimMasterConfig) != HAL_OK)  
  {  
    Error_Handler();  
  } 
	*/

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
 	//if (HAL_TIM_IC_Start_IT(&TimHandle_Slave,TIM_CHANNEL_1) != HAL_OK)
  //{
    /* Starting Error */
  //  Error_Handler();
  //}
	
	//if (HAL_TIM_IC_Start_IT(&TimHandle_Master,TIM_CHANNEL_1) != HAL_OK)
  //{
    /* Starting Error */
  //  Error_Handler();
  //}
	
	if (HAL_TIM_IC_Start_IT(&TimHandle_32bits,TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

	
	
	//if (HAL_TIM_IC_Start_IT(&TimHandle,TIM_CHANNEL_2) != HAL_OK)
  //{
  //  /* Starting Error */
  //  Error_Handler();
  //} 
	//if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  //{
  //  /* Starting Error */
  //  Error_Handler();
  //}

  while (1)
  {
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(LED1);
}


/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//BSP_LED_Toggle(LED1);
	
	/*
	if(htim->Instance == TIMx_Master)
	{
	  BSP_LED_Toggle(LED1);
		
		//First Cap
	  if(counter_cap == 0)
	  {
		  uwIC1Value1_Master=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		  counter_cap++;
	  }
	  else
	  {
		  uwIC1Value2_Master=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		
		  if(uwIC1Value2_Master > uwIC1Value1_Master)
		  {
				uwDiffCapture_Master = uwIC1Value2_Master - uwIC1Value1_Master; 
		  }
		  else
		  {
				uwDiffCapture_Master = (__HAL_TIM_GET_AUTORELOAD(htim) -uwIC1Value1_Master + 1) + uwIC1Value2_Master; 
		  }
		  uwIC1Value1_Master = uwIC1Value2_Master;
	  }
	}
	
	
	if(htim->Instance == TIMx_Slave)
	{
		//First Cap
	  if(counter_cap == 0)
	  {
		  uwIC1Value1_Slave=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		  counter_cap++;
	  }
	  else
	  {
		  uwIC1Value2_Slave=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		
		  if(uwIC1Value2_Slave > uwIC1Value1_Slave)
		  {
				uwDiffCapture_Slave = uwIC1Value2_Slave - uwIC1Value1_Slave; 
		  }
		  else
		  {
				uwDiffCapture_Slave = (__HAL_TIM_GET_AUTORELOAD(htim) -uwIC1Value1_Slave + 1) + uwIC1Value2_Slave; 
		  }
		  uwIC1Value1_Slave = uwIC1Value2_Slave;
	  }		
	}
  
	if(htim->Instance == TIMx_32bits)
	{
		BSP_LED_Toggle(LED1);
		//First Cap
	  if(counter_cap == 0)
	  {
		  uwIC1Value1_32bits=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		  counter_cap++;
	  }
	  else
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
	  }
		
		
		
		
	}*/
  if(counter_cap == 0)
	{
		uwIC1Value1_32bits=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		testArray_Period[counter_cap] = 0;
		testArray_Period_Diff[counter_cap] = 0;
		counter_cap++;
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
		counter_cap++;
	}	
	else
	{
		BSP_LED_Toggle(LED1);
	}
	/*
	//First Cap
	if(counter_cap == 0)
	{
	  uwIC1Value1_Slave=HAL_TIM_ReadCapturedValue(&TimHandle_Slave, TIM_CHANNEL_1);
	  counter_cap++;
	}
	else
	{
	  uwIC1Value2_Slave=HAL_TIM_ReadCapturedValue(&TimHandle_Slave, TIM_CHANNEL_1);
		
		if(uwIC1Value2_Slave > uwIC1Value1_Slave)
		{
		  uwDiffCapture_Slave = uwIC1Value2_Slave - uwIC1Value1_Slave; 
		}
		else
		{
			uwDiffCapture_Slave = (__HAL_TIM_GET_AUTORELOAD(htim) -uwIC1Value1_Slave + 1) + uwIC1Value2_Slave; 
		}
		  uwIC1Value1_Slave = uwIC1Value2_Slave;
	 }	
  */	 
	
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
