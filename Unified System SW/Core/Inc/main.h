/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_dma.h"

#include "stm32l4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PE6_VPS_Detect_Pin GPIO_PIN_6
#define PE6_VPS_Detect_GPIO_Port GPIOE
#define PC13_USER_PB_Pin GPIO_PIN_13
#define PC13_USER_PB_GPIO_Port GPIOC
#define PA4_VCC9_SENS_Measure_Pin GPIO_PIN_4
#define PA4_VCC9_SENS_Measure_GPIO_Port GPIOA
#define PA5_VCC9_HEAT_Measure_Pin GPIO_PIN_5
#define PA5_VCC9_HEAT_Measure_GPIO_Port GPIOA
#define PA6_VCC9_LCD_Measure_Pin GPIO_PIN_6
#define PA6_VCC9_LCD_Measure_GPIO_Port GPIOA
#define PA7_VCC3_3_Measure_Pin GPIO_PIN_7
#define PA7_VCC3_3_Measure_GPIO_Port GPIOA
#define PB1_HEAT_Sense_Measure_Pin GPIO_PIN_1
#define PB1_HEAT_Sense_Measure_GPIO_Port GPIOB
#define PE9_Heat_CTRL_Pin GPIO_PIN_9
#define PE9_Heat_CTRL_GPIO_Port GPIOE
#define PB10_I2C2_SCL_Pin GPIO_PIN_10
#define PB10_I2C2_SCL_GPIO_Port GPIOB
#define PB11_I2C2_SDA_Pin GPIO_PIN_11
#define PB11_I2C2_SDA_GPIO_Port GPIOB
#define PD8_VCC9_SENS_EN_Pin GPIO_PIN_8
#define PD8_VCC9_SENS_EN_GPIO_Port GPIOD
#define PD9_VCC9_HEAT_EN_Pin GPIO_PIN_9
#define PD9_VCC9_HEAT_EN_GPIO_Port GPIOD
#define PD10_VCC9_LCD_EN_Pin GPIO_PIN_10
#define PD10_VCC9_LCD_EN_GPIO_Port GPIOD
#define PD11_STAT1_Pin GPIO_PIN_11
#define PD11_STAT1_GPIO_Port GPIOD
#define PD12_STAT2_Pin GPIO_PIN_12
#define PD12_STAT2_GPIO_Port GPIOD
#define PD13_PG_N_Pin GPIO_PIN_13
#define PD13_PG_N_GPIO_Port GPIOD
#define PC9_BTT_CE_Pin GPIO_PIN_9
#define PC9_BTT_CE_GPIO_Port GPIOC
#define PA8_CHG_Booster_EN_Pin GPIO_PIN_8
#define PA8_CHG_Booster_EN_GPIO_Port GPIOA
#define PA11_MCU_TX_Pin GPIO_PIN_9
#define PA11_MCU_TX_GPIO_Port GPIOA
#define PA10_MCU_RX_Pin GPIO_PIN_10
#define PA10_MCU_RX_GPIO_Port GPIOA
#define PA11_PS_On_N_Pin GPIO_PIN_11
#define PA11_PS_On_N_GPIO_Port GPIOA
#define PA12_PS_Off_N_Pin GPIO_PIN_12
#define PA12_PS_Off_N_GPIO_Port GPIOA
#define PA13_SWDIO_Pin GPIO_PIN_13
#define PA13_SWDIO_GPIO_Port GPIOA
#define PA14_SWCLK_Pin GPIO_PIN_14
#define PA14_SWCLK_GPIO_Port GPIOA
#define PC10_BTT_On_N_Pin GPIO_PIN_10
#define PC10_BTT_On_N_GPIO_Port GPIOC
#define PC11_BTT_Off_N_Pin GPIO_PIN_11
#define PC11_BTT_Off_N_GPIO_Port GPIOC
#define PD6_VBTT_Sense_EN_Pin GPIO_PIN_6
#define PD6_VBTT_Sense_EN_GPIO_Port GPIOD
#define PD7_PS_Remote_Off_Pin GPIO_PIN_7
#define PD7_PS_Remote_Off_GPIO_Port GPIOD
#define PB6_I2C1_SCL_Pin GPIO_PIN_6
#define PB6_I2C1_SCL_GPIO_Port GPIOB
#define PB7_I2C1_SDA_Pin GPIO_PIN_7
#define PB7_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
