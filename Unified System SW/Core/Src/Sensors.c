#include "main.h"

#include "Sensors.h"
#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

t_Sensor g_tSensor[4U];         // Global variable

uint16_t SwapLsbMsb(uint16_t data)
{
  t_Senor16Data pSwapedData;
  uint8_t uitemp = 0U;
  
  pSwapedData.iWordArray=data;
  uitemp = pSwapedData.uiByteArray[0U];
  pSwapedData.uiByteArray[0U] = pSwapedData.uiByteArray[1U];
  pSwapedData.uiByteArray[1U] = uitemp;
  
  return pSwapedData.iWordArray;
}

void Unlock_Sensor (uint8_t uiSenorNum)
{
  t_Senor32Data pUnlockSwapData;
  HAL_StatusTypeDef ErrorCheck;
    
  pUnlockSwapData.uiWordArray[0U]= SwapLsbMsb(REFRESH_UNLOCK_MSB);
  pUnlockSwapData.uiWordArray[1U]= SwapLsbMsb(REFRESH_UNLOCK_LSB);
  
  ErrorCheck = HAL_I2C_Master_Transmit(g_tSensor[uiSenorNum].Channel, g_tSensor[uiSenorNum].I2C_Adress, pUnlockSwapData.uiByteArray, 4U, I2C_TIMEOUT_MS);
  if(ErrorCheck !=  HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(REFRESH_RATE_DELAY_MS);
  return;
}


void QIF_Write_I2C_Register(uint8_t uiSensorIndex, uint16_t uiAdress, uint16_t uiData)
{
  t_Senor16Data uiSwapedDate;
  HAL_StatusTypeDef ErrorCheck;
  
  uiSwapedDate.iWordArray=SwapLsbMsb(uiData);
  ErrorCheck = HAL_I2C_Mem_Write(g_tSensor[uiSensorIndex].Channel,g_tSensor[uiSensorIndex].I2C_Adress , uiAdress, I2C_MEMADD_SIZE_16BIT,uiSwapedDate.uiByteArray,2U,I2C_TIMEOUT_MS);
  if(ErrorCheck !=  HAL_OK)
  {
    Error_Handler();
  }
  return;
}

uint16_t QIF_Read_I2C_Register(uint8_t uiSensorIndex, uint16_t uiAdress)
{
  t_Senor16Data uiReturnedDate;
  HAL_StatusTypeDef ErrorCheck;
  
  ErrorCheck = HAL_I2C_Mem_Read(g_tSensor[uiSensorIndex].Channel,g_tSensor[uiSensorIndex].I2C_Adress , uiAdress, I2C_MEMADD_SIZE_16BIT,uiReturnedDate.uiByteArray,2U,I2C_TIMEOUT_MS);
  if(ErrorCheck !=  HAL_OK)
  {
    Error_Handler();
  }
  uiReturnedDate.iWordArray=SwapLsbMsb(uiReturnedDate.iWordArray);
  return uiReturnedDate.iWordArray;
}

void QIF_Write_I2C_EE(uint8_t uiSensorIndex, uint16_t uiAdress, uint16_t uiData)
{
  t_Senor16Data uiSwapedDate;
  HAL_StatusTypeDef ErrorCheck;
  uint32_t uiCounter=0U;
  uiCounter=LL_TIM_GetCounter(TIM5);
  do
  {
      if(LL_TIM_GetCounter(TIM5) - uiCounter <=  100U) // 100 is 1mili second.
      {
      uiSwapedDate.iWordArray = QIF_Read_I2C_Register(uiSensorIndex,MLX90632_SENSOR_REG_STATUS);
      g_tSensor[uiSensorIndex].RegStatusUnified.uiRegStatus=SwapLsbMsb(uiSwapedDate.iWordArray);
      }
      else
      {
        Error_Handler();
      }
  }
  while (g_tSensor[uiSensorIndex].RegStatusUnified.RegStatus.uiEEprom_busy == 1U);
  
  Unlock_Sensor(uiSensorIndex);
   
  uiSwapedDate.iWordArray=SwapLsbMsb(uiData);
  ErrorCheck = HAL_I2C_Mem_Write(g_tSensor[uiSensorIndex].Channel,g_tSensor[uiSensorIndex].I2C_Adress , uiAdress, I2C_MEMADD_SIZE_16BIT,uiSwapedDate.uiByteArray,2U,I2C_TIMEOUT_MS);
  if(ErrorCheck !=  HAL_OK)
  {
    Error_Handler();
  }
  return;
}

uint16_t QIF_Read_I2C_EE(uint8_t uiSensorIndex, uint16_t uiAdress)
{
  t_Senor16Data uiReturnedDate;
  HAL_StatusTypeDef ErrorCheck;
  
  ErrorCheck = HAL_I2C_Mem_Read(g_tSensor[uiSensorIndex].Channel,g_tSensor[uiSensorIndex].I2C_Adress , uiAdress, I2C_MEMADD_SIZE_16BIT,uiReturnedDate.uiByteArray,2U,I2C_TIMEOUT_MS);
  if(ErrorCheck !=  HAL_OK)
  {
    Error_Handler();
  }
  uiReturnedDate.iWordArray=SwapLsbMsb(uiReturnedDate.iWordArray);
  return uiReturnedDate.iWordArray;
}


void Reset_Sensors (uint8_t uiSenorNum)
{
  t_Senor32Data pResetData;
  HAL_StatusTypeDef ErrorCheck;
    
  pResetData.uiWordArray[0U]= SwapLsbMsb(MSB_RESET_DATA);
  pResetData.uiWordArray[1U]= SwapLsbMsb(LSB_RESET_DATA);
  
  ErrorCheck = HAL_I2C_Master_Transmit(g_tSensor[uiSenorNum].Channel, g_tSensor[uiSenorNum].I2C_Adress, pResetData.uiByteArray, 4U, I2C_TIMEOUT_MS);
  if(ErrorCheck !=  HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(200);
  return;
}



void Senors_I2C_Init()
{
  t_Senor16Data t_temp;
  t_Senor16Data t_Buffer[37U];
  int32_t iShiftVariable;
  const uint16_t EE_Meas_Adress[3U] = {MLX90632_EE_EXTENDED_MEAS1 , MLX90632_EE_EXTENDED_MEAS2 , MLX90632_EE_EXTENDED_MEAS3};
  const uint16_t EE_Meas_Data[3U] = {MLX90632_EE_EXTENDED_MEAS1_DATA , MLX90632_EE_EXTENDED_MEAS2_DATA , MLX90632_EE_EXTENDED_MEAS3_DATA};
  
  g_tSensor[0U].I2C_Adress=0x3A<<1;   //0x3A<<9;
  g_tSensor[1U].I2C_Adress=0x3B<<1;   //0x3B<<9;
  g_tSensor[2U].I2C_Adress=0x3A<<1;   //0x3A<<9;
  g_tSensor[3U].I2C_Adress=0x3B<<1;   //0x3B<<9;
  g_tSensor[0U].Channel= &hi2c1;
  g_tSensor[1U].Channel= &hi2c1;
  g_tSensor[2U].Channel= &hi2c2;
  g_tSensor[3U].Channel= &hi2c2;
  
  for(uint8_t uiSenorIndex=0U; uiSenorIndex< 4U ; uiSenorIndex++)
  {
    Reset_Sensors (uiSenorIndex);
    
    t_temp.iWordArray = QIF_Read_I2C_Register(uiSenorIndex,MLX90632_SENSOR_REG_CTRL);
    g_tSensor[uiSenorIndex].RegCtr.RegContorl.uiMode = eHALT_MODE;
    g_tSensor[uiSenorIndex].RegCtr.RegContorl.uiMeas_Select = eEXTENDED;
    t_temp.iWordArray =  g_tSensor[uiSenorIndex].RegCtr.iWordArray;
    QIF_Write_I2C_Register(uiSenorIndex,MLX90632_SENSOR_REG_CTRL,t_temp.iWordArray);
    
    //---------------Set Refresh Rate for Sensors----------// 
    for(uint8_t UiSenIndex=0U ;  UiSenIndex < 3U ; UiSenIndex++)
    {
      QIF_Write_I2C_EE(uiSenorIndex, EE_Meas_Adress[UiSenIndex], 0U); // Erase in order to Write the Data
      QIF_Write_I2C_EE(uiSenorIndex, EE_Meas_Adress[UiSenIndex], EE_Meas_Data[UiSenIndex]);
    }        
    //---------------------------------------------------// 
    g_tSensor[uiSenorIndex].RegCtr.iWordArray=QIF_Read_I2C_Register(uiSenorIndex,MLX90632_SENSOR_REG_CTRL);
    g_tSensor[uiSenorIndex].RegCtr.RegContorl.uiMode = eCONTINIOUS_MODE;
    QIF_Write_I2C_Register(uiSenorIndex,MLX90632_SENSOR_REG_CTRL, g_tSensor[uiSenorIndex].RegCtr.iWordArray);
    for(uint8_t uiSenorIndex2=0U; uiSenorIndex2 <37U ; uiSenorIndex2++)
    {
      t_Buffer[uiSenorIndex2].iWordArray = QIF_Read_I2C_EE(uiSenorIndex,MLX90632_EE_P_R + uiSenorIndex2);
    }
    iShiftVariable=t_Buffer[1U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[0U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_P_R=iShiftVariable*(pow(2.0F,-8.0F));
    
    iShiftVariable=t_Buffer[3U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[2U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_P_G=iShiftVariable*(pow(2.0F,-20.0F));
    
    iShiftVariable=t_Buffer[5U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[4U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_P_T=iShiftVariable*(pow(2.0F,-44.0F));
    
    iShiftVariable=t_Buffer[7U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[6U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_P_O=iShiftVariable*(pow(2.0F,-8.0F));
    
    iShiftVariable=t_Buffer[25U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[24U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_Ea=iShiftVariable*(pow(2.0F,-16.0F));
    
    iShiftVariable=t_Buffer[27U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[26U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_Eb=iShiftVariable*(pow(2.0F,-8.0F));
    
    iShiftVariable=t_Buffer[29U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[28U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_Fa=iShiftVariable*(pow(2.0F,-46.0F));
    
    iShiftVariable=t_Buffer[31U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[30U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_Fb=iShiftVariable*(pow(2.0F,-36.0F));
    
    iShiftVariable=t_Buffer[33U].iWordArray;
    iShiftVariable=(iShiftVariable<<16U) + t_Buffer[32U].iWordArray;
    g_tSensor[uiSenorIndex].fEE_Ga=iShiftVariable*(pow(2.0F,-36.0F));
    
    g_tSensor[uiSenorIndex].fEE_Gb=t_Buffer[34U].iWordArray*(pow(2.0F,-10.0F));
    g_tSensor[uiSenorIndex].fEE_Ka=t_Buffer[35U].iWordArray*(pow(2.0F,-10.0F));
    g_tSensor[uiSenorIndex].fEE_Kb=t_Buffer[36U].iWordArray*(1.0F);
    g_tSensor[uiSenorIndex].fEE_Ha=1.0F;
    g_tSensor[uiSenorIndex].fEE_Hb=0.0F;
  }
}

void Calculate_Temps(uint8_t uiSensorIndex, double* Ambient_Temp, double* Object_Temp)
{
  int16_t iMeasurements[8U]={0U};
  double dVR_TA=0.0F;
  double dAMB=0.0F;
  double dS=0.0F;
  double dVRTO=0.0F;
  double dSTO=0.0F;
  double dTa=0.0F;
  double dTO=0.0F;
  double dTO_1=0.0F;
  double dTO_2=0.0F;
  double dTO_3=0.0F;
  for(uint8_t uiPtr=0U; uiPtr< 8U; uiPtr++)
  {
  iMeasurements[uiPtr]=QIF_Read_I2C_Register(uiSensorIndex,RAM_52+uiPtr);
  }
  dVR_TA=(double)(iMeasurements[5U])+g_tSensor[uiSensorIndex].fEE_Gb*((double)(iMeasurements[2U]/(12U)));
  dAMB=(((double)(iMeasurements[2U]/(12U)))/(dVR_TA))*(pow(2.0F,19.0F));
  dS=(double)(((iMeasurements[0U]-iMeasurements[1U]-iMeasurements[3U]+iMeasurements[4U])/2U)+iMeasurements[6U]+iMeasurements[7U]);
  dVRTO=(double)(iMeasurements[5U])+g_tSensor[uiSensorIndex].fEE_Ka*(iMeasurements[2U]/(12U));
  dSTO=(dS/12.0F)/dVRTO*(pow(2.0F,19.0F));
  dTa=g_tSensor[uiSensorIndex].fEE_P_O + (dAMB-g_tSensor[uiSensorIndex].fEE_P_R)/g_tSensor[uiSensorIndex].fEE_P_G + g_tSensor[uiSensorIndex].fEE_P_T * pow((dAMB - g_tSensor[uiSensorIndex].fEE_P_R),2U);
  
 dTO_1=((g_tSensor[uiSensorIndex].fEE_Fa/2U)*g_tSensor[uiSensorIndex].fEE_Ha)*((1+g_tSensor[uiSensorIndex].fEE_Ga*()
}




