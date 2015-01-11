/**
 ******************************************************************************
 * @file    lsm6ds3.c
 * @author  MEMS Application Team
 * @version V0.0.1
 * @date    18-September-2014
 * @brief   This file provides a set of functions needed to manage the LSM6DS3 sensor
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "lsm6ds3.h"
#include <math.h>

/** @addtogroup BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A1
 * @{
 */

/** @addtogroup LSM6DS3
 * @{
 */

/** @addtogroup LSM6DS3_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_Defines
 * @{
 */

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_Macros
 * @{
 */

/**
 * @}
 */


/** @addtogroup LSM6DS3_Callable_Private_FunctionPrototypes
 * @{
 */
static IMU_6AXES_StatusTypeDef    LSM6DS3_Init( IMU_6AXES_InitTypeDef *LSM6DS3_Init );
static IMU_6AXES_StatusTypeDef    LSM6DS3_Read_XG_ID( uint8_t *xg_id);
static IMU_6AXES_StatusTypeDef    LSM6DS3_X_GetAxes( int32_t *pData );
static IMU_6AXES_StatusTypeDef    LSM6DS3_G_GetAxes( int32_t *pData );
static IMU_6AXES_StatusTypeDef    LSM6DS3_X_GetSensitivity( float *pfData );
static IMU_6AXES_StatusTypeDef    LSM6DS3_G_GetSensitivity( float *pfData );

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_Variables
 * @{
 */
IMU_6AXES_DrvTypeDef LSM6DS3Drv = {
    LSM6DS3_Init,
    LSM6DS3_Read_XG_ID,
    LSM6DS3_X_GetAxes,
    LSM6DS3_G_GetAxes,
    LSM6DS3_X_GetSensitivity,
    LSM6DS3_G_GetSensitivity
};

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_FunctionPrototypes
 * @{
 */

static IMU_6AXES_StatusTypeDef LSM6DS3_X_GetAxesRaw(int16_t *pData);
static IMU_6AXES_StatusTypeDef LSM6DS3_G_GetAxesRaw(int16_t *pData);

/**
 * @}
 */

/** @addtogroup LSM6DS3_Private_FunctionPrototypes
 * @{
 */

/**
 * @brief  Set LSM6DS3 Initialization
 * @param  LSM6DS3_Init the configuration setting for the LSM6DS3
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef    LSM6DS3_Init( IMU_6AXES_InitTypeDef *LSM6DS3_Init )
{
    /*Here we have to add the check if the parameters are valid*/
    uint8_t tmp1 = 0x00;

    /* Configure the low level interface -------------------------------------*/
    if(LSM6DS3_IO_Init() != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }


    /******** Common init *********/

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL3_C, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    /* Enable register address automatically incremented during a multiple byte
       access with a serial interface (I2C or SPI) */
    tmp1 &= ~(LSM6DS3_XG_IF_INC_MASK);
    tmp1 |= LSM6DS3_XG_IF_INC;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL3_C, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }


    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL5, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    /* FIFO ODR selection */
    tmp1 &= ~(LSM6DS3_XG_FIFO_ODR_MASK);
    tmp1 |= LSM6DS3_XG_FIFO_ODR_NA;

    /* FIFO mode selection */
    tmp1 &= ~(LSM6DS3_XG_FIFO_MODE_MASK);
    tmp1 |= LSM6DS3_XG_FIFO_MODE_BYPASS;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_FIFO_CTRL5, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }


    /******* Gyroscope init *******/

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    /* Output Data Rate selection */
    tmp1 &= ~(LSM6DS3_G_ODR_MASK);
    tmp1 |= LSM6DS3_Init->G_OutputDataRate;

    /* Full scale selection */
    tmp1 &= ~(LSM6DS3_G_FS_MASK);
    tmp1 |= LSM6DS3_Init->G_FullScale;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }


    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL10_C, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    /* Enable X axis selection */
    tmp1 &= ~(LSM6DS3_G_XEN_MASK);
    tmp1 |= LSM6DS3_Init->G_X_Axis;

    /* Enable Y axis selection */
    tmp1 &= ~(LSM6DS3_G_YEN_MASK);
    tmp1 |= LSM6DS3_Init->G_Y_Axis;

    /* Enable Z axis selection */
    tmp1 &= ~(LSM6DS3_G_ZEN_MASK);
    tmp1 |= LSM6DS3_Init->G_Z_Axis;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL10_C, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }


    /***** Accelerometer init *****/

    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    /* Output Data Rate selection */
    tmp1 &= ~(LSM6DS3_XL_ODR_MASK);
    tmp1 |= LSM6DS3_Init->X_OutputDataRate;

    /* Full scale selection */
    tmp1 &= ~(LSM6DS3_XL_FS_MASK);
    tmp1 |= LSM6DS3_Init->X_FullScale;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }


    if(LSM6DS3_IO_Read(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    /* Enable X axis selection */
    tmp1 &= ~(LSM6DS3_XL_XEN_MASK);
    tmp1 |= LSM6DS3_Init->X_X_Axis;

    /* Enable Y axis selection */
    tmp1 &= ~(LSM6DS3_XL_YEN_MASK);
    tmp1 |= LSM6DS3_Init->X_Y_Axis;

    /* Enable Z axis selection */
    tmp1 &= ~(LSM6DS3_XL_ZEN_MASK);
    tmp1 |= LSM6DS3_Init->X_Z_Axis;

    if(LSM6DS3_IO_Write(&tmp1, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }
    
    return IMU_6AXES_OK;
}

/**
 * @brief  Read ID of LSM6DS3 Accelerometer and Gyroscope
 * @param  xg_id the pointer where the ID of the device is stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef    LSM6DS3_Read_XG_ID( uint8_t *xg_id)
{
    if(!xg_id){ return IMU_6AXES_ERROR; }

    return LSM6DS3_IO_Read(xg_id, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_WHO_AM_I_ADDR, 1);
}



/**
 * @brief  Read raw data from LSM6DS3 Accelerometer output register
 * @param  pData the pointer where the accelerometer raw data are stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef LSM6DS3_X_GetAxesRaw( int16_t *pData )
{
    /*Here we have to add the check if the parameters are valid*/
  
    uint8_t tempReg[2] = {0,0};


    if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_OUT_X_L_XL /*+ 0x80*/, 2) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    pData[0] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_OUT_Y_L_XL /*+ 0x80*/, 2) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    pData[1] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_OUT_Z_L_XL /*+ 0x80*/, 2) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    pData[2] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
    
    return IMU_6AXES_OK;
}



/**
 * @brief  Read data from LSM6DS3 Accelerometer and calculate linear acceleration in mg
 * @param  pData the pointer where the accelerometer data are stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef    LSM6DS3_X_GetAxes( int32_t *pData )
{
    /*Here we have to add the check if the parameters are valid*/
  
    uint8_t tempReg = 0x00;
    int16_t pDataRaw[3];
    float sensitivity = 0;

    if(LSM6DS3_X_GetAxesRaw(pDataRaw) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    if(LSM6DS3_IO_Read(&tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    tempReg &= LSM6DS3_XL_FS_MASK;

    switch(tempReg)
    {
      case LSM6DS3_XL_FS_2G:
        sensitivity = 0.061;
        break;
      case LSM6DS3_XL_FS_4G:
        sensitivity = 0.122;
        break;
      case LSM6DS3_XL_FS_8G:
        sensitivity = 0.244;
        break;
      case LSM6DS3_XL_FS_16G:
        sensitivity = 0.488;
        break;
    }

    pData[0] = (int32_t)(pDataRaw[0] * sensitivity);
    pData[1] = (int32_t)(pDataRaw[1] * sensitivity);
    pData[2] = (int32_t)(pDataRaw[2] * sensitivity);
  
    return IMU_6AXES_OK;
}



/**
 * @brief  Read raw data from LSM6DS3 Gyroscope output register
 * @param  pData the pointer where the gyroscope raw data are stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef LSM6DS3_G_GetAxesRaw( int16_t *pData )
{
    /*Here we have to add the check if the parameters are valid*/
  
    uint8_t tempReg[2] = {0,0};


    if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_OUT_X_L_G /*+ 0x80*/, 2) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    pData[0] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_OUT_Y_L_G /*+ 0x80*/, 2) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    pData[1] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);

    if(LSM6DS3_IO_Read(&tempReg[0], LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_OUT_Z_L_G /*+ 0x80*/, 2) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    pData[2] = ((((int16_t)tempReg[1]) << 8)+(int16_t)tempReg[0]);
    
    return IMU_6AXES_OK;
}



/**
 * @brief  Read data from LSM6DS3 Gyroscope and calculate angular rate in mdps
 * @param  pData the pointer where the gyroscope data are stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef    LSM6DS3_G_GetAxes( int32_t *pData )
{
    /*Here we have to add the check if the parameters are valid*/
  
    uint8_t tempReg = 0x00;
    int16_t pDataRaw[3];
    float sensitivity = 0;

    if(LSM6DS3_G_GetAxesRaw(pDataRaw) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    if(LSM6DS3_IO_Read(&tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }

    tempReg &= LSM6DS3_G_FS_MASK;

    switch(tempReg)
    {
      case LSM6DS3_G_FS_125:
        sensitivity = 4.375;
        break;
      case LSM6DS3_G_FS_245:
        sensitivity = 8.75;
        break;
      case LSM6DS3_G_FS_500:
        sensitivity = 17.50;
        break;
      case LSM6DS3_G_FS_1000:
        sensitivity = 35;
        break;
      case LSM6DS3_G_FS_2000:
        sensitivity = 70;
        break;
    }

    pData[0] = (int32_t)(pDataRaw[0] * sensitivity);
    pData[1] = (int32_t)(pDataRaw[1] * sensitivity);
    pData[2] = (int32_t)(pDataRaw[2] * sensitivity);
  
    return IMU_6AXES_OK;
}

/**
 * @brief  Read Accelero Sensitivity
 * @param  pfData the pointer where the accelerometer sensitivity is stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
 */
static IMU_6AXES_StatusTypeDef    LSM6DS3_X_GetSensitivity( float *pfData )
{
    /*Here we have to add the check if the parameters are valid*/
    
    uint8_t tempReg = 0x00;
    
    
    if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1 ) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }
    
    tempReg &= LSM6DS3_XL_FS_MASK;
    
    switch( tempReg ) {
    case LSM6DS3_XL_FS_2G:
        *pfData = 0.061;
        break;
    case LSM6DS3_XL_FS_4G:
        *pfData = 0.122;
        break;
    case LSM6DS3_XL_FS_8G:
        *pfData = 0.244;
        break;
    case LSM6DS3_XL_FS_16G:
        *pfData = 0.488;
        break;
    default:
        break;
    }
    
    return IMU_6AXES_OK;
}



/**
 * @brief  Read Gyro Sensitivity
 * @param  pfData the pointer where the gyroscope sensitivity is stored
 * @retval IMU_6AXES_OK in case of success, an error code otherwise
*/
static IMU_6AXES_StatusTypeDef    LSM6DS3_G_GetSensitivity( float *pfData )
{
    /*Here we have to add the check if the parameters are valid*/
    
    uint8_t tempReg = 0x00;
    
    
    if(LSM6DS3_IO_Read( &tempReg, LSM6DS3_XG_MEMS_ADDRESS, LSM6DS3_XG_CTRL2_G, 1 ) != IMU_6AXES_OK)
    {
      return IMU_6AXES_ERROR;
    }
    
    tempReg &= LSM6DS3_G_FS_MASK;
    
    switch( tempReg ) {
    case LSM6DS3_G_FS_125:
        *pfData = 4.375;
        break;
    case LSM6DS3_G_FS_245:
        *pfData = 8.75;
        break;
    case LSM6DS3_G_FS_500:
        *pfData = 17.50;
        break;
    case LSM6DS3_G_FS_1000:
        *pfData = 35;
        break;
    case LSM6DS3_G_FS_2000:
        *pfData = 70;
        break;
    default:
        break;
    }
    
    return IMU_6AXES_OK;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
