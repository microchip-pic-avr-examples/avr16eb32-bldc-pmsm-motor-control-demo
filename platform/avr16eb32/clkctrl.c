#include <avr/io.h>
#include <avr/cpufunc.h>
#include "clkctrl.h"

/**
* Configures the Fuse bits.
*/
FUSES = 
{
  .BODCFG = ACTIVE_ENABLED_gc | LVL_BODLEVEL2_gc | SAMPFREQ_128HZ_gc | SLEEP_DISABLE_gc,
  .BOOTSIZE = 0x0,
  .CODESIZE = 0x0,
  .OSCCFG = OSCHFFRQ_20M_gc,
  .PDICFG = KEY_NOTACT_gc | LEVEL_BASIC_gc,
  .SYSCFG0 = CRCSEL_CRC16_gc | CRCSRC_NOCRC_gc | RSTPINCFG_NONE_gc | UPDIPINCFG_UPDI_gc,
  .SYSCFG1 = SUT_64MS_gc,
  .WDTCFG = PERIOD_OFF_gc | WINDOW_OFF_gc,
//  .reserved_1[0] = 0xFF,
//  .reserved_1[1] = 0xFF,
//  .reserved_2[0] = 0xFF
};

#if   (F_CPU==24000000UL) || (F_CPU==24000000)
#define CLK_SETTING  CLKCTRL_FRQSEL_24M_gc
#elif (F_CPU==20000000UL) || (F_CPU==20000000)
#define CLK_SETTING  CLKCTRL_FRQSEL_20M_gc
#elif (F_CPU==16000000UL) || (F_CPU==16000000)
#define CLK_SETTING  CLKCTRL_FRQSEL_16M_gc
#elif (F_CPU==12000000UL) || (F_CPU==12000000)
#define CLK_SETTING  CLKCTRL_FRQSEL_12M_gc
#elif (F_CPU==8000000UL) || (F_CPU==8000000)
#define CLK_SETTING  CLKCTRL_FRQSEL_8M_gc
#else
#define CLK_SETTING  CLKCTRL_FRQSEL_4M_gc
#endif


void CLKCTRL_Init(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (CLKCTRL.MCLKCTRLB & ~CLKCTRL_PEN_bm) | CLKCTRL_PBDIV_NONE_gc);
}

