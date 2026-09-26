/**
 * IWDG + output watchdog
 * LSI ~32 kHz, PR=32, RLR=249 → ~250 ms
 * Output clamp: coils/injectors forced off if held past max pulse.
 */
#include "main.h"
#include "ecu_watchdog.h"
#include "ecu_pins.h"
#include "ecu_runtime.h"
#include "ecu_config.h"

#if defined(STM32F411xE) || defined(STM32F4)
#include "stm32f4xx.h"
#endif

static uint8_t iwdg_on = 0;

#if defined(HAL_IWDG_MODULE_ENABLED)
extern IWDG_HandleTypeDef hiwdg;
static uint8_t hiwdg_valid(void)
{
  return (hiwdg.Instance == IWDG) ? 1u : 0u;
}
#endif

static void outputClamp(void)
{
  uint32_t now = micros();

  uint32_t cmax = dwellTargetUs ? (uint32_t)dwellTargetUs + 2000u : 6000u;
  if (cmax < 2500u) cmax = 2500u;
  if (cmax > 10000u) cmax = 10000u;

  for (uint8_t i = 1; i <= MAX_CYL; i++) {
    if (coilState[i] && coilStartUs[i] && (now - coilStartUs[i]) > cmax) {
      ECU_IGN_LO(i);
      coilState[i] = 0;
    }
    if (injOn[i]) {
      int32_t late = (int32_t)(now - injEndUs[i]);
      if (late >= 0 || late < -25000) {
        ECU_INJ_LO(i);
        injOn[i] = 0;
      }
    }
  }
}

void ECU_Watchdog_Init(void)
{
  if (iwdg_on)
    return;

#if defined(HAL_IWDG_MODULE_ENABLED)
  if (hiwdg_valid()) {
    iwdg_on = 1;
    HAL_IWDG_Refresh(&hiwdg);
    return;
  }
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload    = 249; /* ~250 ms */
  if (HAL_IWDG_Init(&hiwdg) == HAL_OK) {
    iwdg_on = 1;
    return;
  }
#endif

  IWDG->KR  = 0x5555u;
  IWDG->PR  = 0x03u;   /* /32 */
  IWDG->RLR = 249u;
  IWDG->KR  = 0xAAAAu;
  IWDG->KR  = 0xCCCCu;
  iwdg_on = 1;
}

void ECU_Watchdog_Kick(void)
{
  outputClamp();
  if (!iwdg_on)
    return;
#if defined(HAL_IWDG_MODULE_ENABLED)
  if (hiwdg_valid()) {
    (void)HAL_IWDG_Refresh(&hiwdg);
    return;
  }
#endif
  IWDG->KR = 0xAAAAu;
}

uint8_t ECU_Watchdog_IsEnabled(void)
{
  return iwdg_on;
}
