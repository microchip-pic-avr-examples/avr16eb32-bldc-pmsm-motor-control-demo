#include <avr/io.h>
#include <stdbool.h>
#include "reset.h"


void ResetDevice(void)
{
    _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
    while(true)
    ;
}

