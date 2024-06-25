#include <avr/io.h>
#include "evsys.h"

void Evsys_Init(void)
{
    EVSYS.CHANNEL0 = EVSYS_CHANNEL_AC1_OUT_gc;
    EVSYS.USERWEXA = EVSYS_USER_CHANNEL0_gc;
}
