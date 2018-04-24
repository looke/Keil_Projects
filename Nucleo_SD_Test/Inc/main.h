/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Inc/main.h
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    30-December-2016
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_nucleo_144.h"
#include "looke_fileFormat_SD.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* User can use this section to tailor TIMx instance used and associated
   resources */
/* Definition for TIMx clock resources */
#define TIMx_32bits                    TIM2
#define TIMx_32bits_CLK_ENABLE()       __HAL_RCC_TIM2_CLK_ENABLE()

#define TIMx_ARHS                      TIM3
#define TIMx_ARHS_CLK_ENABLE()         __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for SD Card clock resources */
#define SD1_CLK_ENABLE()							 __HAL_RCC_SDMMC1_CLK_ENABLE()
#define SD2_CLK_ENABLE()							 __HAL_RCC_SDMMC2_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIMx_32bits_IRQn               TIM2_IRQn
#define TIMx_32bits_IRQHandler         TIM2_IRQHandler

#define TIMx_ARHS_IRQn                 TIM3_IRQn
#define TIMx_ARHS_IRQHandler           TIM3_IRQHandler

//
#define TIMx_32bits_CHANNEL_GPIO_PORT() __GPIOA_CLK_ENABLE()
#define TIMx_ARHS_CHANNEL_GPIO_PORT()   __GPIOB_CLK_ENABLE()

#define SDMMC1_GEN_GPIO_PORT()          __GPIOC_CLK_ENABLE()
#define SDMMC1_CMD_GPIO_PORT()          __GPIOD_CLK_ENABLE()

#define SDMMC2_GEN_GPIO_D0_D4_PORT()    __GPIOB_CLK_ENABLE()
#define SDMMC2_CMD_GPIO_PORT()          __GPIOD_CLK_ENABLE()


#define GPIO_AF_TIMx_32bits            GPIO_AF1_TIM2

#define GPIO_AF_SDMMC1								 GPIO_AF12_SDMMC1

#define GPIO_AF_SDMMC2_GEN						 GPIO_AF10_SDMMC2
#define GPIO_AF_SDMMC2_CMD						 GPIO_AF11_SDMMC2



#define GPIO_PIN_TIMx_32bits_CHANNEL_1       GPIO_PIN_5

#define SDMMC1_D0            GPIO_PIN_8   //PC8
#define SDMMC1_D1            GPIO_PIN_9   //PC9
#define SDMMC1_D2            GPIO_PIN_10  //PC10
#define SDMMC1_D3            GPIO_PIN_11  //PC11
#define SDMMC1_CK            GPIO_PIN_12  //PC12
#define SDMMC1_CMD           GPIO_PIN_2   //PD2

#define SDMMC2_D0            GPIO_PIN_14	//PB14
#define SDMMC2_D1            GPIO_PIN_15  //PB15
#define SDMMC2_D2            GPIO_PIN_3   //PB3
#define SDMMC2_D3            GPIO_PIN_4   //PB4
#define SDMMC2_CK            GPIO_PIN_6   //PD6
#define SDMMC2_CMD           GPIO_PIN_7   //PD7


//#define TEST_LIMIT 						600

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
