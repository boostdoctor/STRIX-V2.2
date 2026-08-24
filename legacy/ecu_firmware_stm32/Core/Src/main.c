/**
 * TorquEFI / STRIX — application main (CubeMX-compatible)
 * USB CDC init early so COM port enumerates even if later init fails.
 * Crank TIM5/PA0, Cam1 TIM2/PA15, Cam2 TIM3/PB4
 * INJ1 = PB15
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

/* Weak: only used if CubeMX did not generate MX_TIM3_Init */
__attribute__((weak)) void MX_TIM3_Init(void)
{
  /* no-op — enable TIM3 in CubeMX for Cam2 */
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();

  /* USB as early as practical so Windows/Linux see a COM port */
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

  ECU_Init();
  ECU_Serial_Init();

  /* Input capture — skip if handle not ready */
  if (htim5.Instance != NULL)
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  if (htim2.Instance != NULL)
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  if (htim3.Instance != NULL && htim3.Instance == TIM3)
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

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

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef o = {0};
  RCC_ClkInitTypeDef c = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  o.HSIState = RCC_HSI_ON;
  o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState = RCC_PLL_ON;
  o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  o.PLL.PLLM = 16;
  o.PLL.PLLN = 192;
  o.PLL.PLLP = RCC_PLLP_DIV2;
  o.PLL.PLLQ = 4; /* 48 MHz USB */
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
