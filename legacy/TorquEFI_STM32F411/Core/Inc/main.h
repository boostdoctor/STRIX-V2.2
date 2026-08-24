#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* GPIO labels — align with CubeMX User Labels when regenerating */
#define IGN1_Pin GPIO_PIN_0
#define IGN1_GPIO_Port GPIOB
#define IGN2_Pin GPIO_PIN_1
#define IGN2_GPIO_Port GPIOB
#define IGN3_Pin GPIO_PIN_2
#define IGN3_GPIO_Port GPIOB
#define IGN4_Pin GPIO_PIN_3
#define IGN4_GPIO_Port GPIOB

#define INJ1_Pin GPIO_PIN_15
#define INJ1_GPIO_Port GPIOB
#define INJ2_Pin GPIO_PIN_5
#define INJ2_GPIO_Port GPIOB
#define INJ3_Pin GPIO_PIN_6
#define INJ3_GPIO_Port GPIOB
#define INJ4_Pin GPIO_PIN_7
#define INJ4_GPIO_Port GPIOB

#define BOOST_Pin GPIO_PIN_8
#define BOOST_GPIO_Port GPIOB
#define ETB_PWM_Pin GPIO_PIN_8
#define ETB_PWM_GPIO_Port GPIOA
#define ETB_DIR_Pin GPIO_PIN_9
#define ETB_DIR_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_9
#define FAN_GPIO_Port GPIOB
#define FP_Pin GPIO_PIN_10
#define FP_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif
#endif
