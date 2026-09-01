/**
 * IWDG: LSI ~32 kHz
 * Prescaler 64, reload 499 → timeout ≈ (499+1)*64/32000 ≈ 1.0 s
 *
 * Register path used so projects without HAL_IWDG_MODULE_ENABLED still work.
 * If HAL IWDG is enabled and hiwdg is inited by Cube, HAL refresh is also used.
 */
#include "main.h"
#include "ecu_watchdog.h"

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

void ECU_Watchdog_Init(void)
{
  /* IWDG off until CDC is proven — reset loops look like a dead COM */
  iwdg_on = 0;
  return;
  /* original follows if re-enabled */
}
void ECU_Watchdog_Init_DISABLED(void)
{
  if (iwdg_on)
    return;

#if defined(HAL_IWDG_MODULE_ENABLED)
  if (hiwdg_valid()) {
    /* Cube already started IWDG — just mark active */
    iwdg_on = 1;
    HAL_IWDG_Refresh(&hiwdg);
    return;
  }
  /* Start via HAL */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload    = 499; /* ~1 s @ 32 kHz LSI */
  if (HAL_IWDG_Init(&hiwdg) == HAL_OK) {
    iwdg_on = 1;
    return;
  }
#endif

  /* Register-level fallback (F4 IWDG) */
  /* Enable write access */
  IWDG->KR = 0x5555u;
  /* PR = 64 (0x04), RLR = 499 */
  IWDG->PR  = 0x04u;
  IWDG->RLR = 499u;
  /* Wait optional — LSI may need settle; reload & start */
  IWDG->KR = 0xAAAAu; /* reload */
  IWDG->KR = 0xCCCCu; /* start */
  iwdg_on = 1;
}

void ECU_Watchdog_Kick(void)
{
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
