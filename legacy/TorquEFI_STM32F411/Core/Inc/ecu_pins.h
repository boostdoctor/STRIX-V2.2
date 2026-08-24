/**
 * TorquEFI — STM32F411 Black Pill, 4-cylinder, USB-only + dual VVT
 *
 * - Sequential spark / inject: 4 channels
 * - VVT1 / VVT2 cam solenoids (low-side or PWM-capable pins)
 * - Tuner: USB CDC on PA11/PA12 — CAN not used
 */
#ifndef ECU_PINS_H
#define ECU_PINS_H

#include "main.h"

/* ── Ignition 1–4 ───────────────────────────────────────────── */
#ifndef IGN1_Pin
#define IGN1_GPIO_Port   GPIOB
#define IGN1_Pin         GPIO_PIN_0
#define IGN2_GPIO_Port   GPIOB
#define IGN2_Pin         GPIO_PIN_1
#define IGN3_GPIO_Port   GPIOB
#define IGN3_Pin         GPIO_PIN_2
#define IGN4_GPIO_Port   GPIOB
#define IGN4_Pin         GPIO_PIN_3
#endif

/* ── Injectors 1–4 ──────────────────────────────────────────── */
#ifndef INJ1_Pin
#define INJ1_GPIO_Port   GPIOB
#define INJ1_Pin         GPIO_PIN_15  /* was PB4 (NJTRST) — now PB15 */
#define INJ2_GPIO_Port   GPIOB
#define INJ2_Pin         GPIO_PIN_5
#define INJ3_GPIO_Port   GPIOB
#define INJ3_Pin         GPIO_PIN_6
#define INJ4_GPIO_Port   GPIOB
#define INJ4_Pin         GPIO_PIN_7
#endif

/* ── VVT cam solenoids (was spare PA10 / PB14) ──────────────── */
#ifndef VVT1_Pin
#define VVT1_GPIO_Port   GPIOA
#define VVT1_Pin         GPIO_PIN_10   /* TIM1_CH3 — intake VVT */
#define VVT2_GPIO_Port   GPIOB
#define VVT2_Pin         GPIO_PIN_14   /* TIM1_CH2N — exhaust VVT */
#endif

/* ── Aux outputs ────────────────────────────────────────────── */
#ifndef BOOST_Pin
#define BOOST_GPIO_Port    GPIOB
#define BOOST_Pin          GPIO_PIN_8  /* TIM4_CH3 closed-loop boost */
#define ETB_PWM_GPIO_Port  GPIOA
#define ETB_PWM_Pin        GPIO_PIN_8  /* TIM1_CH1 closed-loop ETB */
#define ETB_DIR_GPIO_Port  GPIOA
#define ETB_DIR_Pin        GPIO_PIN_9
#define FAN_GPIO_Port      GPIOB
#define FAN_Pin            GPIO_PIN_9
#define FP_GPIO_Port       GPIOB
#define FP_Pin             GPIO_PIN_10
#define LED_GPIO_Port      GPIOC
#define LED_Pin            GPIO_PIN_13
#endif

/* ── Cam 2 (TIM3_CH1) ─────────────────────────────────────── */
#ifndef CAM2_Pin
#define CAM2_GPIO_Port   GPIOB
#define CAM2_Pin         GPIO_PIN_4   /* TIM3_CH1 AF2 */
#endif
/* PC14, PC15 still free (PB15 = INJ1) */

#define ECU_IGN_HI(n)  do { switch ((n)) { \
  case 1: HAL_GPIO_WritePin(IGN1_GPIO_Port, IGN1_Pin, GPIO_PIN_SET); break; \
  case 2: HAL_GPIO_WritePin(IGN2_GPIO_Port, IGN2_Pin, GPIO_PIN_SET); break; \
  case 3: HAL_GPIO_WritePin(IGN3_GPIO_Port, IGN3_Pin, GPIO_PIN_SET); break; \
  case 4: HAL_GPIO_WritePin(IGN4_GPIO_Port, IGN4_Pin, GPIO_PIN_SET); break; \
  default: break; \
  } } while (0)

#define ECU_IGN_LO(n)  do { switch ((n)) { \
  case 1: HAL_GPIO_WritePin(IGN1_GPIO_Port, IGN1_Pin, GPIO_PIN_RESET); break; \
  case 2: HAL_GPIO_WritePin(IGN2_GPIO_Port, IGN2_Pin, GPIO_PIN_RESET); break; \
  case 3: HAL_GPIO_WritePin(IGN3_GPIO_Port, IGN3_Pin, GPIO_PIN_RESET); break; \
  case 4: HAL_GPIO_WritePin(IGN4_GPIO_Port, IGN4_Pin, GPIO_PIN_RESET); break; \
  default: break; \
  } } while (0)

#define ECU_INJ_HI(n)  do { switch ((n)) { \
  case 1: HAL_GPIO_WritePin(INJ1_GPIO_Port, INJ1_Pin, GPIO_PIN_SET); break; \
  case 2: HAL_GPIO_WritePin(INJ2_GPIO_Port, INJ2_Pin, GPIO_PIN_SET); break; \
  case 3: HAL_GPIO_WritePin(INJ3_GPIO_Port, INJ3_Pin, GPIO_PIN_SET); break; \
  case 4: HAL_GPIO_WritePin(INJ4_GPIO_Port, INJ4_Pin, GPIO_PIN_SET); break; \
  default: break; \
  } } while (0)

#define ECU_INJ_LO(n)  do { switch ((n)) { \
  case 1: HAL_GPIO_WritePin(INJ1_GPIO_Port, INJ1_Pin, GPIO_PIN_RESET); break; \
  case 2: HAL_GPIO_WritePin(INJ2_GPIO_Port, INJ2_Pin, GPIO_PIN_RESET); break; \
  case 3: HAL_GPIO_WritePin(INJ3_GPIO_Port, INJ3_Pin, GPIO_PIN_RESET); break; \
  case 4: HAL_GPIO_WritePin(INJ4_GPIO_Port, INJ4_Pin, GPIO_PIN_RESET); break; \
  default: break; \
  } } while (0)

#define ECU_VVT1_HI() HAL_GPIO_WritePin(VVT1_GPIO_Port, VVT1_Pin, GPIO_PIN_SET)
#define ECU_VVT1_LO() HAL_GPIO_WritePin(VVT1_GPIO_Port, VVT1_Pin, GPIO_PIN_RESET)
#define ECU_VVT2_HI() HAL_GPIO_WritePin(VVT2_GPIO_Port, VVT2_Pin, GPIO_PIN_SET)
#define ECU_VVT2_LO() HAL_GPIO_WritePin(VVT2_GPIO_Port, VVT2_Pin, GPIO_PIN_RESET)

#define ECU_FAN_HI()  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET)
#define ECU_FAN_LO()  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET)
#define ECU_FP_HI()   HAL_GPIO_WritePin(FP_GPIO_Port, FP_Pin, GPIO_PIN_SET)
#define ECU_FP_LO()   HAL_GPIO_WritePin(FP_GPIO_Port, FP_Pin, GPIO_PIN_RESET)
#define ECU_LED_TOG() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim1;   /* VVT/ETB PWM TIM1 */
extern TIM_HandleTypeDef  htim4;   /* boost PWM TIM4 */
extern TIM_HandleTypeDef  htim5;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;   /* Cam2 PB4 TIM3_CH1 */
/* TIM1 PWM: CH3=VVT1 PA10, CH2N=VVT2 PB14 — see ECU_SetVVT() */

#define ECU_ADC_CH_MAP     ADC_CHANNEL_1
#define ECU_ADC_CH_TPS     ADC_CHANNEL_2
#define ECU_ADC_CH_CLT     ADC_CHANNEL_3
#define ECU_ADC_CH_IAT     ADC_CHANNEL_4
#define ECU_ADC_CH_O2      ADC_CHANNEL_5   /* PA5 */
#define ECU_ADC_CH_KNOCK   ADC_CHANNEL_6   /* PA6 conditioned knock */
#define ECU_ADC_CH_VBATT   ADC_CHANNEL_7   /* PA7 */
/* PA6 was fuel pressure — Option A: knock takes PA6 */
#define ECU_ADC_CH_PEDAL   ADC_CHANNEL_12

#endif
