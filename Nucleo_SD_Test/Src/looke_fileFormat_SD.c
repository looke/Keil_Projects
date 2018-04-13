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
  * @brief  Add a New TimeBase Measure data to Cache 
  * @param  pCache: Pointer to the TimeBase Cache  
  * @param  pData: Pointer to the TimeBase Measure data
  * @retval HAL status
  */
HAL_StatusTypeDef LOOKE_SD_File_AddTimeBaseMeasureToCache(LOOKE_SD_TimeBase_Data_Cache* pCache, LOOKE_SD_TimeBase_Data* pData)
{
	LOOKE_SD_TimeBase_Data_Buffer_Union *pBufferUnion;
	
	//Check MeasureIndex and Increase Block Index if Need
	if(pCache->CurrentMeasureIndex == TIMEBASE_BLOCK_DATA_SIZE)
	{
		pCache->CurrentMeasureIndex = 0x0000;
		pCache->CurrentBlockIndex++;
	}
		
	//Check Block Index and Switch Buffer if Need
	if(pCache->CurrentBlockIndex >= LOOKE_SD_FILE_CACHE_SIZE)
	{
		//Buffer is Full, Should Switch to Empty Buffer and Sync Full Buffer
			
		////Check Buffer State in Case the Master and Slave Buffer all Full
		if(pCache->CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC)
		{
			//Cache Master and Slave Buffer both are Full.
			return HAL_ERROR;
		}
		 
		//Clear Buffer Index
		pCache->CurrentMeasureIndex = 0x0000;
		pCache->CurrentBlockIndex = 0x0000;
			
		//Switch Buffer
		if(pCache->CurrentDataBuffer == LOOKE_SD_FILE_BUFFER_MASTER)
	  {
		  pCache->CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_SLAVE;
	  }
	  else
		{
			pCache->CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_MASTER;
		}
			
		//Put Buffer State to Need Sync
		pCache->CacheBufferState = LOOKE_SD_FILE_BUFFER_NEED_SYNC;
	}

	// Find Cache Buffer currently in use
	if(pCache->CurrentDataBuffer == LOOKE_SD_FILE_BUFFER_MASTER)
	{
		pBufferUnion = &(pCache->TimeBase_DataBuffer_Master);
	}
	else
	{
		pBufferUnion = &(pCache->TimeBase_DataBuffer_Slave);
	}
		
  //Put New Data into Buffer
  pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].TimeBaseData[pCache->CurrentMeasureIndex].DataIndex = pData->DataIndex;
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].TimeBaseData[pCache->CurrentMeasureIndex].TimeStamp = pData->TimeStamp;
		
	//Increase Buffer Index
	pCache->CurrentMeasureIndex++;	

	return HAL_OK;
};




/**
  * @brief  Add a New ARHS Measure data to Cache 
  * @param  pCache: Pointer to the ARHS Cache  
  * @param  pData: Pointer to the ARHS Measure data
  * @retval HAL status
  */
HAL_StatusTypeDef LOOKE_SD_File_AddARHSMeasureToCache(LOOKE_SD_ARHS_Data_Cache* pCache, LOOKE_SD_ARHS_Data* pData)
{
	LOOKE_SD_ARHS_Data_Buffer_Union *pBufferUnion;
	
	//Check MeasureIndex and Increase Block Index if Need
  if(pCache->CurrentMeasureIndex >= AHRS_BLOCK_DATA_SIZE)
	{
    pCache->CurrentMeasureIndex = 0x0000;
		pCache->CurrentBlockIndex++;
	}

	//Check Block Index and Switch Buffer if Need
	if(pCache->CurrentBlockIndex >= LOOKE_SD_FILE_CACHE_SIZE)
	{
	  //Buffer is Full, Should Switch to Empty Buffer and Sync Full Buffer
			
		//Check Buffer State in Case the Master and Slave Buffer all Full
		if(pCache->CacheBufferState == LOOKE_SD_FILE_BUFFER_NEED_SYNC)
		{
		  //Cache Master and Slave Buffer both are Full.
		  return HAL_ERROR;
		}
		 
		//Clear Buffer Index
		pCache->CurrentMeasureIndex = 0x0000;
		pCache->CurrentBlockIndex = 0x0000;
			
		//Switch Buffer
		if(pCache->CurrentDataBuffer == LOOKE_SD_FILE_BUFFER_MASTER)
	  {
		  pCache->CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_SLAVE;
	  }
	  else
		{
			pCache->CurrentDataBuffer = LOOKE_SD_FILE_BUFFER_MASTER;
		}
			
		//Put Buffer State to Need Sync
		pCache->CacheBufferState = LOOKE_SD_FILE_BUFFER_NEED_SYNC;
	}

	// Find Cache Buffer currently in use
	if(pCache->CurrentDataBuffer == LOOKE_SD_FILE_BUFFER_MASTER)
	{
		pBufferUnion = &(pCache->ARHS_DataBuffer_Master);
	}
	else
	{
		pBufferUnion = &(pCache->ARHS_DataBuffer_Slave);
	}
		
  //Put New Data into Buffer
  pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].DataType = pData->DataType;
	
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].ACC_X = pData->ACC_X;
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].ACC_Y = pData->ACC_Y;
  pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].ACC_Z = pData->ACC_Z;
	
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].Gyro_X = pData->Gyro_X;
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].Gyro_Y = pData->Gyro_Y;
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].Gyro_Z = pData->Gyro_Z;
	
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].Mag_X = pData->Mag_X;
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].Mag_Y = pData->Mag_Y;
	pBufferUnion->dataBlockArray[pCache->CurrentBlockIndex].ARHS_Data[pCache->CurrentMeasureIndex].Mag_Z = pData->Mag_Z;
	
	//Increase Buffer Index
	pCache->CurrentMeasureIndex++;		
  return HAL_OK;
};



/**
  * @brief  ARHS cache write to SD Card.
  * @param  hsd: Pointer to the SD handle
  * @retval LOOKE_SD_FILE_TRANSFER_RESULT
  */
LOOKE_SD_FILE_SYNC_TRANSFER_RESULT LOOKE_SD_File_SyncCacheToSDCard_ARHS(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para *pFileSysPara, LOOKE_SD_ARHS_Data_Cache *pCache)
{
	LOOKE_SD_ARHS_Data_Buffer_Union *pBufferUnion;
	uint8_t currentBlockIndexOnSD;
  
	// Find Cache Buffer currently in use
  // The one that not in use should be sync to SD card
	if(pCache->CurrentDataBuffer == LOOKE_SD_FILE_BUFFER_MASTER)
	{
		pBufferUnion = &(pCache->ARHS_DataBuffer_Slave);
	}
	else
	{
		pBufferUnion = &(pCache->ARHS_DataBuffer_Master);
	}
	
	//Find the end Block of Current Measure Section
	//Then increase one step for new block.
	currentBlockIndexOnSD = 1 + pFileSysPara->SectionIndexArray[pFileSysPara->NumberOfMeasurementSection - 1].SectionEndBlock;
	
	//Check SD card size
	if(currentBlockIndexOnSD+LOOKE_SD_FILE_CACHE_SIZE >= hsd->SdCard.BlockNbr)
	{
	  return LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_SDFULL;
	}
	
	//Check SD Card State
  if(HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER)
	{
		return LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_SDBUSY;
	}
	
	//Start DMA Write Transfer
	if (HAL_SD_WriteBlocks_DMA(hsd, pBufferUnion->DataArray, currentBlockIndexOnSD, LOOKE_SD_FILE_CACHE_SIZE) != HAL_OK)
	{
		return LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_DMA;
	}
	
	return LOOKE_SD_FILE_SYNC_TRANSFER_OK;
	
	
	/////////////////////////////////////////////////////////////////////
	//
	// Buffer State Should be Changed in DMA Transfer Complete Interrupt
	// FileSysPara Should be Update in DMA Transfer Complete Interrupt
	//
	/////////////////////////////////////////////////////////////////////
	
};



/**
  * @brief  Time Base cache write to SD Card.
  * @param  hsd: Pointer to the SD handle
  * @retval LOOKE_SD_FILE_TRANSFER_RESULT
  */
LOOKE_SD_FILE_SYNC_TRANSFER_RESULT LOOKE_SD_File_SyncCacheToSDCard_TimeBase(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para *pFileSysPara, LOOKE_SD_TimeBase_Data_Cache *pCache)
{
	LOOKE_SD_TimeBase_Data_Buffer_Union *pBufferUnion;
	uint8_t currentBlockIndexOnSD;
  
	// Find Cache Buffer currently in use
  // The one that not in use should be sync to SD card
	if(pCache->CurrentDataBuffer == LOOKE_SD_FILE_BUFFER_MASTER)
	{
		pBufferUnion = &(pCache->TimeBase_DataBuffer_Slave);
	}
	else
	{
		pBufferUnion = &(pCache->TimeBase_DataBuffer_Master);
	}
	
	//Find the end Block of Current Measure Section
	//Then increase one step for new block.
	currentBlockIndexOnSD = 1 + pFileSysPara->SectionIndexArray[pFileSysPara->NumberOfMeasurementSection - 1].SectionEndBlock;
	
	//Check SD card size
	if(currentBlockIndexOnSD+LOOKE_SD_FILE_CACHE_SIZE >= hsd->SdCard.BlockNbr)
	{
	  return LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_SDFULL;
	}
	
	//Check SD Card State
  if(HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER)
	{
		return LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_SDBUSY;
	}
	
	//Start DMA Write Transfer
	if (HAL_SD_WriteBlocks_DMA(hsd, pBufferUnion->DataArray, currentBlockIndexOnSD, LOOKE_SD_FILE_CACHE_SIZE) != HAL_OK)
	{
		return LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_DMA;
	}
	
	return LOOKE_SD_FILE_SYNC_TRANSFER_OK;
	
	
	/////////////////////////////////////////////////////////////////////
	//
	// Buffer State Should be Changed in DMA Transfer Complete Interrupt
	// FileSysPara Should be Update in DMA Transfer Complete Interrupt
	//
	/////////////////////////////////////////////////////////////////////
	
};
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
