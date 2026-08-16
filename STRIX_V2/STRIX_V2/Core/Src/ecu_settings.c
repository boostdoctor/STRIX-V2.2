/**
 * Settings pack/apply — weak stubs; strong versions live in ecu_app.c
 * so Cube projects that only add this file still link.
 */
#include "ecu_settings.h"
#include <string.h>

__attribute__((weak)) void ECU_Settings_Pack(EcuFlashSettings *out)
{
  if (out) memset(out, 0, sizeof(*out));
}

__attribute__((weak)) void ECU_Settings_Apply(const EcuFlashSettings *in)
{
  (void)in;
}
