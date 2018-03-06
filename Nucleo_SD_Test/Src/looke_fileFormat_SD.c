/**
  ******************************************************************************
  * @file    looke_fileFormat_SD.c
  * @author  Spiride Application Team
  * @version V1.0.2
  * @date    01-March-2018
  * @brief   Main SD File System Service Routines.
  *          
  *          
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

#include "looke_fileFormat_SD.h"

/** @addtogroup 
  * @{
  */

/** @addtogroup 
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Read the SD Card File System Parameters to the specified buffer 
            SD_HandleTypeDef and create the associated handle.
  * @param  hsd: Pointer to the SD handle  
	* @param  pData: Pointer to the buffer that will contain the received data
  * @retval HAL status
  */
HAL_StatusTypeDef LOOKE_SD_File_ReadSysPara(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para_Union *pSysParaUnion)
{
	
	if(HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER)
	{
		return HAL_BUSY;
	}
	
  return HAL_SD_ReadBlocks(hsd, pSysParaUnion->DataArray, 0x00, 1, 0xFFF);
};

/**
  * @brief  Write the SD Card File System Parameters to the SD Card 
  * @param  hsd: Pointer to the SD handle  
  * @param  pData: Pointer to the file system parameters data
  * @retval HAL status
  */
HAL_StatusTypeDef LOOKE_SD_File_WriteSysPara(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para_Union *pSysParaUnion)
{
	
	if(HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER)
	{
		return HAL_BUSY;
	}
	
  return HAL_SD_WriteBlocks(hsd, pSysParaUnion->DataArray, 0x00, 1, 0x0F);
};

/**
  * @brief  Create a New Measure Section on SD Card 
  * @param  hsd: Pointer to the SD handle  
  * @param  pFileSysPara: Pointer to the file system parameters data union
  * @retval HAL status
  */
HAL_StatusTypeDef LOOKE_SD_File_CreateMeasureSection(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para_Union* pFileSysPara)
{
	uint32_t newMeasureSectionBlockIndex = 1;
	
	uint32_t measureSectionNum = pFileSysPara->FileSysPara.NumberOfMeasurementSection;
	
	if(measureSectionNum > 0)
	{
	  newMeasureSectionBlockIndex = pFileSysPara->FileSysPara.SectionIndexArray[measureSectionNum-1].SectionEndBlock;
		newMeasureSectionBlockIndex++;
	}
	
	__align(4) LOOKE_SD_MeasureSection_Para_Union newSectionParaUnion;
	newSectionParaUnion.MeasureSectionPara.NumberOfAHRSDataBlock = 0;
	newSectionParaUnion.MeasureSectionPara.NumberOfTimeBaseDataBlock = 0;
	
	if(HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER)
	{
		return HAL_BUSY;
	}
	
	//Write New Measure Section Parameters to SD Card
	if(HAL_SD_WriteBlocks(hsd, newSectionParaUnion.DataArray, newMeasureSectionBlockIndex, 1, 0x0FF) != HAL_OK )
	{
		return HAL_ERROR;
	}
	
	pFileSysPara->FileSysPara.NumberOfMeasurementSection = measureSectionNum + 1;
	pFileSysPara->FileSysPara.SectionIndexArray[measureSectionNum].SectionStartBlock = newMeasureSectionBlockIndex;
	pFileSysPara->FileSysPara.SectionIndexArray[measureSectionNum].SectionEndBlock = newMeasureSectionBlockIndex;
	
	if(HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER)
	{
		return HAL_BUSY;
	}
	
	//Write New File System Parameters to SD Card
	if(HAL_SD_WriteBlocks(hsd, pFileSysPara->DataArray, 0, 1, 0x0FF) != HAL_OK )
	{
		pFileSysPara->FileSysPara.NumberOfMeasurementSection = measureSectionNum;
	  pFileSysPara->FileSysPara.SectionIndexArray[measureSectionNum].SectionStartBlock = 0;
	  pFileSysPara->FileSysPara.SectionIndexArray[measureSectionNum].SectionEndBlock = 0;

		return HAL_ERROR;
	}
	return HAL_OK;
};
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
