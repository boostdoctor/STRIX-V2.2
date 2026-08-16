/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define CRANK_TIM5_CH1_Pin GPIO_PIN_0
#define CRANK_TIM5_CH1_GPIO_Port GPIOA
#define MAP_ADC1_IN1_Pin GPIO_PIN_1
#define MAP_ADC1_IN1_GPIO_Port GPIOA
#define TPS_ADC1_IN2_Pin GPIO_PIN_2
#define TPS_ADC1_IN2_GPIO_Port GPIOA
#define CLT_ADC1_IN3_Pin GPIO_PIN_3
#define CLT_ADC1_IN3_GPIO_Port GPIOA
#define IAT_ADC1_IN4_Pin GPIO_PIN_4
#define IAT_ADC1_IN4_GPIO_Port GPIOA
#define O2_ADC1_IN5_Pin GPIO_PIN_5
#define O2_ADC1_IN5_GPIO_Port GPIOA
#define KNOCK_ADC1_IN6_Pin GPIO_PIN_6
#define KNOCK_ADC1_IN6_GPIO_Port GPIOA
#define VBATT_ADC1_IN7_Pin GPIO_PIN_7
#define VBATT_ADC1_IN7_GPIO_Port GPIOA
#define IGN1_Pin GPIO_PIN_0
#define IGN1_GPIO_Port GPIOB
#define IGN2_Pin GPIO_PIN_1
#define IGN2_GPIO_Port GPIOB
#define IGN3_Pin GPIO_PIN_2
#define IGN3_GPIO_Port GPIOB
#define FUEL_PUMP_Pin GPIO_PIN_10
#define FUEL_PUMP_GPIO_Port GPIOB
#define CLUTCH_SW_Pin GPIO_PIN_13
#define CLUTCH_SW_GPIO_Port GPIOB
#define VVT2_TIM1_CH2N_Pin GPIO_PIN_14
#define VVT2_TIM1_CH2N_GPIO_Port GPIOB
#define INJ1_Pin GPIO_PIN_15
#define INJ1_GPIO_Port GPIOB
#define ETB_PWM_TIM1_CH1_Pin GPIO_PIN_8
#define ETB_PWM_TIM1_CH1_GPIO_Port GPIOA
#define ETB_DIR_Pin GPIO_PIN_9
#define ETB_DIR_GPIO_Port GPIOA
#define VVT1_TIM1_CH3_Pin GPIO_PIN_10
#define VVT1_TIM1_CH3_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define CAM1_TIM2_CH1_Pin GPIO_PIN_15
#define CAM1_TIM2_CH1_GPIO_Port GPIOA
#define IGN4_Pin GPIO_PIN_3
#define IGN4_GPIO_Port GPIOB
#define CAM2_TIM3_CH1_Pin GPIO_PIN_4
#define CAM2_TIM3_CH1_GPIO_Port GPIOB
#define INJ2_Pin GPIO_PIN_5
#define INJ2_GPIO_Port GPIOB
#define INJ3_Pin GPIO_PIN_6
#define INJ3_GPIO_Port GPIOB
#define INJ4_Pin GPIO_PIN_7
#define INJ4_GPIO_Port GPIOB
#define BOOST_TIM4_CH3_Pin GPIO_PIN_8
#define BOOST_TIM4_CH3_GPIO_Port GPIOB
#define FAN_RELAY_Pin GPIO_PIN_9
#define FAN_RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
