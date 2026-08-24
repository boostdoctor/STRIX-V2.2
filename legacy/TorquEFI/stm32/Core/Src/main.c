/**
 * TorquEFI / STRIX — application main (CubeMX-compatible)
 *
 * USB "device descriptor failed" is almost always:
 *  - USB clock not exactly 48 MHz, or
 *  - crash before/during enumeration
 *
 * Black Pill F411: 25 MHz HSE → PLL → 96 MHz SYSCLK, USB 48 MHz.
 * If your board has no crystal, use SystemClock_Config_HSI() instead.
 */
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "usb_device.h"
#include "ecu_app.h"
#include "ecu_serial.h"
#include "ecu_pins.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void SystemClock_Config(void);
void SystemClock_Config_HSI(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();   /* HSE 25 MHz Black Pill — required for stable USB */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

  /* USB after clocks are stable */
  MX_USB_DEVICE_Init();

  {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin   = INJ1_Pin | INJ2_Pin | INJ3_Pin | INJ4_Pin;
    HAL_GPIO_Init(INJ1_GPIO_Port, &g);
    HAL_GPIO_WritePin(INJ1_GPIO_Port, g.Pin, GPIO_PIN_RESET);
  }

  /* Brief settle so host can finish enumeration before heavy work */
  HAL_Delay(50);

  /* Free PA15/PB3/PB4 from JTAG so TIM2/TIM3/GPIO can use them (keep SWD) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* SWJ_CFG is via DBGMCU on F4 — disable JTAG, keep SWD */
  DBGMCU->CR &= ~(DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY);
  /* AF remap of PA15 is done in TIM2 MSP; force SWD-only: */
#if defined(STM32F411xE) || defined(STM32F401xC) || defined(STM32F401xE)
  /* No AFIO on F4 — GPIO AFR in MSP is enough once pins are re-inited */
#endif

  ECU_Init();
  ECU_Serial_Init();

  /* Crank TIM5 PA0 */
  if (htim5.Instance != NULL) {
    HAL_TIM_Base_Start(&htim5);
    if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1) != HAL_OK)
      Error_Handler();
  }
  /* Cam1 TIM2 PA15 */
  if (htim2.Instance != NULL) {
    HAL_TIM_Base_Start(&htim2);
    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
      Error_Handler();
  }
  /* Cam2 TIM3 PB4 */
  if (htim3.Instance != NULL) {
    HAL_TIM_Base_Start(&htim3);
    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
      Error_Handler();
  }

  if (htim1.Instance != NULL) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  }
  if (htim4.Instance != NULL)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  while (1)
  {
    ECU_Loop();
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5) {
    ECU_CrankCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  } else if (htim->Instance == TIM2) {
    ECU_CamCapture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  } else if (htim->Instance == TIM3) {
    ECU_Cam2Capture(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));
  }
}

/**
 * Black Pill (WeAct) typical: 25 MHz HSE
 * PLL: M=25, N=192, P=2 → SYSCLK 96 MHz
 *      Q=4 → 48 MHz USB  (must be exact)
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  o.HSEState = RCC_HSE_ON;
  o.PLL.PLLState = RCC_PLL_ON;
  o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  o.PLL.PLLM = 25;
  o.PLL.PLLN = 192;
  o.PLL.PLLP = RCC_PLLP_DIV2;
  o.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&o) != HAL_OK) {
    /* No HSE? fall back to HSI (USB may be less reliable) */
    SystemClock_Config_HSI();
    return;
  }

  c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV2;
  c.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/** HSI fallback: 16 MHz HSI → 96 MHz SYSCLK, 48 MHz USB */
void SystemClock_Config_HSI(void)
{
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};

  o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  o.HSIState = RCC_HSI_ON;
  o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState = RCC_PLL_ON;
  o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  o.PLL.PLLM = 16;
  o.PLL.PLLN = 192;
  o.PLL.PLLP = RCC_PLLP_DIV2;
  o.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&o) != HAL_OK) {
    Error_Handler();
  }

  c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  c.AHBCLKDivider = RCC_SYSCLK_DIV1;
  c.APB1CLKDivider = RCC_HCLK_DIV2;
  c.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
