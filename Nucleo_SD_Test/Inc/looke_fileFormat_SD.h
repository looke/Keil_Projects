/**
  ******************************************************************************
  * @file    looke_fileFormat_SD.h
  * @author  Spiride Application Team
  * @version V1.0.2
  * @date    01-March-2018
  * @brief   This file contains the headers of the file format for SD card.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOOKE_FILEFORMAT_SD_H
#define __LOOKE_FILEFORMAT_SD_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_sd.h"


//typedef enum
//{
  //LOOKE_SD_FILE_MEASURE_READY     = ((uint32_t)0x00000001U),  /*!< Measure Section state is ready          */
  //LOOKE_SD_FILE_MEASURE_BUSY      = ((uint32_t)0x00000002U),  /*!< Measure Section state is busy           */
  //LOOKE_SD_FILE_MEASURE_IDLE      = ((uint32_t)0x00000003U),  /*!< Measure Section state is idle           */
	//LOOKE_SD_FILE_MEASURE_ERROR     = ((uint32_t)0x00000004U),  /*!< Measure Section state is error          */
	//LOOKE_SD_FILE_MEASURE_FINISHED  = ((uint32_t)0x00000005U),  /*!< Measure Section state is finished       */
//}LOOKE_SD_FILE_MeasureSection_State;


/** 
  * @brief  SD Card Measurement Sectrion Address Index Structure definition
	* 8 Bytes in total
  */ 
typedef struct
{
  uint32_t SectionStartBlock;                     				 /*!< Specifies the data Type                         */
  
  uint32_t SectionEndBlock;                                /*!< Time Base Data array                            */

}LOOKE_SD_FileSys_MeasureSection_ADDIndex;

#define MEASURE_SECTION_ADDINDEX_SIZE   63         /* Measure Section Address Index Data size    */
//#define SD_FILE_SYSPARA_RESERVED        4          /* SD File System Parameter Reserved in Bytes */
#define SD_FILE_SYSPARA_BLOCK_INDEX     0          /* SD File System Parameter Block Index       */
/** 
  * @brief  SD Card File System Paremeters Structure definition
	* 512 Bytes in total
  */ 
typedef struct
{
  uint32_t NumberOfMeasurementSection;                     							                           /*!< Specifies the Number of Measurement Section in this SD Card    */
  
  LOOKE_SD_FileSys_MeasureSection_ADDIndex SectionIndexArray[MEASURE_SECTION_ADDINDEX_SIZE];       /*!< Measurement Section Address Index Data array                   */

  uint32_t Reserved;                        	                                                     /*!< Reserved space                                                 */

}LOOKE_SD_FileSys_Para;


#define SD_FILE_MeasureSection_RESERVED        126          /* SD Card Measure Section Parameter Reserved in Bytes */
/** 
  * @brief  SD Card Measurement Section Paremeters Structure definition
	* 512 Bytes in total
  */ 
typedef struct
{
	//LOOKE_SD_FILE_MeasureSection_State MeasureSectionState;           /*!< Specifies the State of the Measurement Section.    */
	
  uint32_t NumberOfTimeBaseDataBlock;                     						/*!< Specifies the Number of Time Base Data Block in this Measurement Section    */
	
	uint32_t NumberOfAHRSDataBlock;                     							  /*!< Specifies the Number of AHRS Data Block in this Measurement Section    */
  
  uint32_t Reserved[SD_FILE_MeasureSection_RESERVED];                 /*!< Reserved space                                                 */

}LOOKE_SD_MeasureSection_Para;



	/** 
  * @brief  SD Card Time Base Data Structure definition
	* 8 Bytes in total
  */ 
typedef struct
{
  uint32_t DataIndex;                    /*!< Specifies the data index number                 */
  
  uint32_t TimeStamp;                    /*!< Specifies the time stamp triggered by input capture       */
	
}LOOKE_SD_TimeBase_Data;


#define TIMEBASE_BLOCK_DATA_SIZE   63         /* Time base block data size */

/** 
  * @brief  SD Card Time Base Data Block Structure definition
	* 512 Bytes in total
  */ 
typedef struct
{
  uint32_t DataType;                     										           /*!< Specifies the data Type                         */
  
  LOOKE_SD_TimeBase_Data TimeBaseData[TIMEBASE_BLOCK_DATA_SIZE];       /*!< Time Base Data array                            */

  uint32_t Reserved;                        	                         /*!< Reserved space                                  */

}LOOKE_SD_TimeBase_Data_Block;



/** 
  * @brief  SD Card ARHS Data Structure definition
  * 40 Bytes in total
  */ 
typedef struct
{
  uint32_t DataType;               /*!< Specifies the data Type                         */
  
  uint32_t ACC_X;                  /*!< Specifies Acceleration sensor X axis data       */

  uint32_t ACC_Y;                  /*!< Specifies Acceleration sensor Y axis data       */

  uint32_t ACC_Z;                  /*!< Specifies Acceleration sensor Z axis data       */
  
  uint32_t Gyro_X;                 /*!< Specifies Gyro sensor X axis data               */

  uint32_t Gyro_Y;                 /*!< Specifies Gyro sensor Y axis data               */
  
  uint32_t Gyro_Z;                 /*!< Specifies Gyro sensor Z axis data               */

  uint32_t Mag_X;                  /*!< Specifies Magnetometer sensor X axis data       */
	
	uint32_t Mag_Y;                  /*!< Specifies Magnetometer sensor Y axis data       */
	
	uint32_t Mag_Z;                  /*!< Specifies Magnetometer sensor Z axis data       */

}LOOKE_SD_ARHS_Data;


#define AHRS_BLOCK_DATA_SIZE   12          /* AHRS block data size in Bytes */
#define AHRS_BLOCK_RESV_SIZE   7           /* AHRS block reserved size in Bytes */

/** 
  * @brief  SD Card ARHS Data Block Structure definition
  * 512 Bytes in total
  */ 
typedef struct
{
  uint32_t  DataType;                                       /*!< Specifies the data Type                         */
  
  LOOKE_SD_ARHS_Data ARHS_Data[AHRS_BLOCK_DATA_SIZE];      /*!< ARHS Data array                                 */

  uint32_t Reserved[AHRS_BLOCK_RESV_SIZE];                  /*!< Reserved space                                  */

}LOOKE_SD_ARHS_Data_Block;


////////////////////////////////////////////////////////////////////////////////////////
/**
  *
  *
  */
////////////////////////////////////////////////////////////////////////////////////////
typedef union
{
  uint8_t DataArray[512];                     		/*!< uint8_t data array for SD card transfer                     */
	
  LOOKE_SD_FileSys_Para FileSysPara;              /*!< SD Card File System Parameters                              */

}LOOKE_SD_FileSys_Para_Union;

typedef union
{
  uint8_t DataArray[512];                     		    /*!< uint8_t data array for SD card transfer   */
	
  LOOKE_SD_MeasureSection_Para MeasureSectionPara;    /*!< SD Card Measure Section Parameters        */

}LOOKE_SD_MeasureSection_Para_Union;



/** @defgroup SD_File_Functions_Group1 initialization functions
  * @{
  */

HAL_StatusTypeDef LOOKE_SD_File_ReadSysPara(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para_Union *pSysParaUnion);
HAL_StatusTypeDef LOOKE_SD_File_WriteSysPara(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para_Union *pSysParaUnion);
HAL_StatusTypeDef LOOKE_SD_File_CreateMeasureSection(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para_Union* pFileSysPara);


////////////////////////////////////////////////////////////////////////////////////////
/**
  *
  *
  */
////////////////////////////////////////////////////////////////////////////////////////
#define LOOKE_SD_FILE_BLOCK_SIZE            512 /* Block Size in Bytes */
#define LOOKE_SD_FILE_CACHE_SIZE            30  /* Cache Size in Block */

typedef union
{
  uint8_t DataArray[LOOKE_SD_FILE_BLOCK_SIZE*LOOKE_SD_FILE_CACHE_SIZE];       /*!< uint8_t data array for SD card transfer   */
	
  LOOKE_SD_TimeBase_Data_Block dataBlockArray[LOOKE_SD_FILE_CACHE_SIZE];      /*!< SD Card Time Base Data Buffer Array       */

}LOOKE_SD_TimeBase_Data_Buffer_Union;


typedef union
{
  uint8_t DataArray[LOOKE_SD_FILE_BLOCK_SIZE*LOOKE_SD_FILE_CACHE_SIZE];       /*!< uint8_t data array for SD card transfer   */
	
  LOOKE_SD_ARHS_Data_Block dataBlockArray[LOOKE_SD_FILE_CACHE_SIZE];          /*!< SD Card ARHS Data Buffer Array            */

}LOOKE_SD_ARHS_Data_Buffer_Union;



typedef enum
{
  LOOKE_SD_FILE_BUFFER_NOT_FULL     = ((uint32_t)0x00000000U),    /*!< Data Master and Slave Buffer bothh are not full   */
	LOOKE_SD_FILE_BUFFER_HALF_FULL    = ((uint32_t)0x00000001U),    /*!< Data Master or Slave Buffer is full               */
	LOOKE_SD_FILE_BUFFER_FULL         = ((uint32_t)0x00000002U),    /*!< Data Master and Slave Buffer are all full         */
}LOOKE_SD_FILE_Buffer_State;

typedef enum
{
  LOOKE_SD_FILE_BUFFER_MASTER       = ((uint32_t)0x00000000U),    /*!< Current Data Buffer is Master Buffer          */
	LOOKE_SD_FILE_BUFFER_SLAVE        = ((uint32_t)0x00000001U),    /*!< Current Data Buffer is Slave Buffer           */
}LOOKE_SD_FILE_Buffer_Current;

/** 
  * @brief  SD Card ARHS Data Cache Structure definition
  * 30732 Bytes in total
  */ 
typedef struct
{
	
	uint32_t CurrentBlockIndex;
	
  uint32_t CurrentMeasureIndex;
	
	LOOKE_SD_FILE_Buffer_Current CurrentDataBuffer;              /*!< Specifies the Data Buffer currently in use Master or Slave     */
	
	LOOKE_SD_FILE_Buffer_State CacheBufferState;                 /*!< Specifies the Data Buffer State NOT_FULLL or FULL              */
	
  LOOKE_SD_ARHS_Data_Buffer_Union ARHS_DataBuffer_Master;      /*!< ARHS Data Buffer Master                         */

  LOOKE_SD_ARHS_Data_Buffer_Union ARHS_DataBuffer_Slave;       /*!< ARHS Data Buffer Slave                          */

}LOOKE_SD_ARHS_Data_Cache;


/** 
  * @brief  SD Card TimeBase Data Cache Structure definition
  * 30732 Bytes in total
  */ 
typedef struct
{
	
	uint32_t CurrentBlockIndex;                                          /*!< Specifies the Block that contains the new data                 */
	
  uint32_t CurrentMeasureIndex;                                        /*!< Specifies the last data in the block                           */
	
	LOOKE_SD_FILE_Buffer_Current CurrentDataBuffer;                      /*!< Specifies the Data Buffer currently in use Master or Slave     */
	
	LOOKE_SD_FILE_Buffer_State CacheBufferState;                         /*!< Specifies the Data Buffer State NOT_FULLL or FULL              */
	
  LOOKE_SD_TimeBase_Data_Buffer_Union TimeBase_DataBuffer_Master;      /*!< Time Base Data Buffer Master                         */

  LOOKE_SD_TimeBase_Data_Buffer_Union TimeBase_DataBuffer_Slave;       /*!< Time Base Data Buffer Slave                          */

}LOOKE_SD_TimeBase_Data_Cache;


/** @defgroup SD_File_Functions_Group2 cache functions
  * @{
  */
typedef enum
{
  LOOKE_SD_FILE_SYNC_TRANSFER_OK     						   = ((uint32_t)0x00000001U),  /*!< Measure Section state is ready          */
  LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_SDCAPACITY      = ((uint32_t)0x00000002U),  /*!< Measure Section state is busy           */
  LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_SDBUSY          = ((uint32_t)0x00000003U),  /*!< Measure Section state is busy           */
	LOOKE_SD_FILE_SYNC_TRANSFER_ERROR_DMA             = ((uint32_t)0x00000004U),  /*!< Measure Section state is error          */
	
}LOOKE_SD_FILE_SYNC_TRANSFER_RESULT;

LOOKE_SD_FILE_SYNC_TRANSFER_RESULT LOOKE_SD_File_SyncCacheToSDCard_ARHS(SD_HandleTypeDef *hsd, LOOKE_SD_FileSys_Para *pFileSysPara, LOOKE_SD_ARHS_Data_Cache *pCache);

HAL_StatusTypeDef LOOKE_SD_File_SyncCacheToSDCard_TimeBase(SD_HandleTypeDef *hsd, LOOKE_SD_TimeBase_Data_Cache *pCahce);


////////////////////////////////////////////////////////////////////////////////////////
/**
  *
  *
  */
////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
  LOOKE_SD_FILE_GLOBLE_CACHE_STATE_TRANSFER   = ((uint32_t)0x00000000U),    /*!< Data Buffer is not in Sync Process and ready for Sync          */
	LOOKE_SD_FILE_GLOBLE_CACHE_STATE_SYNC       = ((uint32_t)0x00000001U),    /*!< Data Buffer is in Sync Process and is writing data to SD Card  */
	LOOKE_SD_FILE_GLOBLE_CACHE_STATE_STOP       = ((uint32_t)0x00000002U),    /*!< Data Collection is stopped                                     */
}LOOKE_SD_FILE_GLOBLE_CACHE_STATE;

/** 
  * @brief  SD Card Global Data Cache Structure definition
  * 30732*2 Bytes in total
  */ 
typedef struct
{
	LOOKE_SD_FILE_GLOBLE_CACHE_STATE CurrentState;    /*!< Specifies the Cache State             */
	
  LOOKE_SD_TimeBase_Data_Cache TimeBase_Cache;      /*!< Time Base Data Cache                  */

  LOOKE_SD_ARHS_Data_Cache ARHS_Cache;              /*!< ARHS Data Cache                       */

}LOOKE_SD_Global_Data_Cache;


#ifdef __cplusplus
}
#endif

#endif /* __LOOKE_FILEFORMAT_SD_H */

/***************************************************************END OF FILE****/
