#include "main.h"

#include "PID_Control.h"
#include "Sensors.h"
#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

extern t_Sensor g_tSensor[4U];         // Global variable
                           
extern TIM_HandleTypeDef htim1;
                           

void Calculate_PID (uint8_t uiSensorIndex)
{
  float fe_t = 0.0F;
  static float f_previous_e_t = 0.0F;  
  float fP_t = 0.0F;
  static float fI_t = 0.0F;
  float fD_t = 0.0F;
  float fPWM_t = 0.0F;
  uint16_t uiPWM_t;
  
  fe_t = IDEAL_TEMP - g_tSensor[uiSensorIndex].fObject_Temp;
  fP_t = K_P *  fe_t;
  fI_t = fI_t + K_I * fe_t;
  fD_t = K_D * (fe_t - f_previous_e_t);
  
  fPWM_t = K_PWM * (fP_t + fI_t + fD_t);
  uiPWM_t= (uint16_t)fPWM_t;
  
  if (uiPWM_t < MIN_DC)
  {
    uiPWM_t = 1U;
  }
  TIM1->CCR1=uiPWM_t;
  
}