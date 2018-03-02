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


/** 
  * @brief  SD Card Measurement Sectrion Address Index Structure definition
	* 8 Bytes in total
  */ 
typedef struct
{
  uint32_t SectionStartBlock;                     				 /*!< Specifies the data Type                         */
  
  uint32_t SectionEndBlock;                                /*!< Time Base Data array                            */

}LOOKE_SD_FileSys_MeasureSection_ADDIndex;

#define MEASURE_SECTION_ADDINDEX_SIZE   63         /* Measure Section Address Index Data size */
#define SD_FILE_SYSPARA_RESERVED        4          /* SD File System Parameter Reserved in Bytes */

/** 
  * @brief  SD Card File System Paremeters Structure definition
	* 512 Bytes in total
  */ 
typedef struct
{
  uint32_t NumberOfMeasurementSection;                     							                           /*!< Specifies the Number of Measurement Section in this SD Card    */
  
  LOOKE_SD_FileSys_MeasureSection_ADDIndex SectionIndexArray[MEASURE_SECTION_ADDINDEX_SIZE];       /*!< Measurement Section Address Index Data array                   */

  uint8_t Reserved[SD_FILE_SYSPARA_RESERVED];                        	                             /*!< Reserved space                                                 */

}LOOKE_SD_FileSys_Para;


#define SD_FILE_MeasureSection_RESERVED        508          /* SD Card Measure Section Parameter Reserved in Bytes */
/** 
  * @brief  SD Card Measurement Section Paremeters Structure definition
	* 512 Bytes in total
  */ 
typedef struct
{
  uint32_t NumberOfTimeBaseDataBlock;                     							                           /*!< Specifies the Number of Time Base Data Block in this Measurement Section    */
	
	uint32_t NumberOfAHRSDataBlock;                     							                               /*!< Specifies the Number of AHRS Data Block in this Measurement Section    */
  
  uint8_t Reserved[SD_FILE_MeasureSection_RESERVED];                        	                     /*!< Reserved space                                                 */

}LOOKE_SD_MeasureSection_Para;


#define TIMEBASE_BLOCK_DATA_SIZE   63         /* Time base block data size in Bytes */
#define TIMEBASE_BLOCK_RESV_SIZE   7          /* Time base block reserved size in Bytes */
	/** 
  * @brief  SD Card Time Base Data Structure definition
	* 8 Bytes in total
  */ 
typedef struct
{
  uint32_t DataIndex;                    /*!< Specifies the data index number                 */
  
  uint32_t TimeStamp;                    /*!< Specifies the time stamp triggered by input capture       */
	
}LOOKE_SD_TimeBase_Data;


/** 
  * @brief  SD Card Time Base Data Block Structure definition
	* 512 Bytes in total
  */ 
typedef struct
{
  uint8_t DataType;                     										           /*!< Specifies the data Type                         */
  
  LOOKE_SD_TimeBase_Data TimeBaseData[TIMEBASE_BLOCK_DATA_SIZE];       /*!< Time Base Data array                            */

  uint8_t Reserved[TIMEBASE_BLOCK_RESV_SIZE];                        	 /*!< Reserved space                                  */

}LOOKE_SD_TimeBase_Data_Block;



/** 
  * @brief  SD Card ARHS Data Structure definition
  * 40 Bytes in total
  */ 
typedef struct
{
  uint8_t  DataType;               /*!< Specifies the data Type                         */
  
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
#define AHRS_BLOCK_RESV_SIZE   31          /* AHRS block reserved size in Bytes */

/** 
  * @brief  SD Card ARHS Data Block Structure definition
  * 512 Bytes in total
  */ 
typedef struct
{
  uint8_t  DataType;                                       /*!< Specifies the data Type                         */
  
  LOOKE_SD_ARHS_Data ARHS_Data[AHRS_BLOCK_DATA_SIZE];      /*!< ARHS Data array                                 */

  uint8_t Reserved[AHRS_BLOCK_RESV_SIZE];                  /*!< Reserved space                                  */

}LOOKE_SD_ARHS_Data_Block;


#ifdef __cplusplus
}
#endif

#endif /* __LOOKE_FILEFORMAT_SD_H */

/***************************************************************END OF FILE****/
