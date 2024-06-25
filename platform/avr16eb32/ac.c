#include <avr/io.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include "ac.h"


ac_irq_cb_t AC1_cb = NULL;


ISR(AC1_AC_vect)
{   
    /* Insert AC interrupt handling code here */
    AC1.STATUS = AC_CMPIF_bm; 
    if (AC1_cb != NULL)
    {
        AC1_cb();
    }    
}

void AC0_Start(void)
{
    AC0.CTRLA |= AC_ENABLE_bm;
}

void AC0_Stop(void)
{
    AC0.CTRLA &= ~AC_ENABLE_bm;
}

void AC0_HysteresisMode(AC_HYSMODE_t mode)
{
    uint8_t temp;
    temp = AC0.CTRLA;
    temp &= ~AC_HYSMODE_gm;
    temp |= mode;
    AC0.CTRLA = temp;
}

void AC0_PowerProfile(AC_POWER_t mode)
{
    uint8_t temp;
    temp = AC0.CTRLA;
    temp &= ~AC_POWER_gm;
    temp |= mode;
    AC0.CTRLA = temp;
}

void AC0_OutputEnable(void)
{
    AC0.CTRLA |= AC_OUTEN_bm;
}

void AC0_OutputDisable(void)
{
    AC0.CTRLA &= ~AC_OUTEN_bm;
}

void AC0_RunStandby(bool en)
{
    if(en == true)
        AC0.CTRLA |= AC_RUNSTDBY_bm;
    else
        AC0.CTRLA &= ~AC_RUNSTDBY_bm;        
}

void AC1_Start(void)
{
    AC1.CTRLA |= AC_ENABLE_bm;
}

void AC1_Stop(void)
{
    AC1.CTRLA &= ~AC_ENABLE_bm;
}

void AC1_HysteresisMode(AC_HYSMODE_t mode)
{
    uint8_t temp;
    temp = AC1.CTRLA;
    temp &= ~AC_HYSMODE_gm;
    temp |= mode;
    AC1.CTRLA = temp;
}

void AC1_PowerProfile(AC_POWER_t mode)
{
    uint8_t temp;
    temp = AC1.CTRLA;
    temp &= ~AC_POWER_gm;
    temp |= mode;
    AC1.CTRLA = temp;
}

void AC1_OutputEnable(void)
{
    AC1.CTRLA |= AC_OUTEN_bm;
}

void AC1_OutputDisable(void)
{
    AC1.CTRLA &= ~AC_OUTEN_bm;
}

void AC1_RunStandby(bool en)
{
    if(en == true)
        AC1.CTRLA |= AC_RUNSTDBY_bm;
    else
        AC1.CTRLA &= ~AC_RUNSTDBY_bm;        
}

void AC0_Initialize(void)
{
    AC0_Start();
}

void AC1_Initialize(void)
{
    AC1_HysteresisMode(AC_HYSMODE_LARGE_gc);
    AC1_PowerProfile(AC_POWER_PROFILE1_gc);
    AC1_Start();
}

void AC0_WindowSelectionMode(AC_WINSEL_t mode)
{
    uint8_t temp;
    temp = AC0.CTRLB;
    temp &= ~AC_WINSEL_gm;
    temp |= mode;
    AC0.CTRLB = temp;
}

void AC1_WindowSelectionMode(AC_WINSEL_t mode)
{
    uint8_t temp;
    temp = AC1.CTRLB;
    temp &= ~AC_WINSEL_gm;
    temp |= mode;
    AC1.CTRLB = temp;
}

void AC0_InvertAcOutput(bool inv)
{
    if(inv == true)
        AC0.MUXCTRL |= AC_INVERT_bm;
    else
        AC0.MUXCTRL &= ~AC_INVERT_bm;
}

void AC1_InvertAcOutput(bool inv)
{
    if(inv == true)
        AC1.MUXCTRL |= AC_INVERT_bm;
    else
        AC1.MUXCTRL &= ~AC_INVERT_bm;
}

void AC0_InitialValueHigh(bool en)
{
    if(en == true)
        AC0.MUXCTRL |= AC_INITVAL_bm;
    else
        AC0.MUXCTRL &= ~AC_INITVAL_bm;        
}

void AC1_InitialValueHigh(bool en)
{
    if(en == true)
        AC1.MUXCTRL |= AC_INITVAL_bm;
    else
        AC1.MUXCTRL &= ~AC_INITVAL_bm;        
}

void AC0_MUXSET(uint8_t mode)
{
    uint8_t temp;
    temp = AC0.MUXCTRL;
    temp &= ~(AC_MUXPOS_gm | AC_MUXNEG_gm);
    temp |= mode;
    AC0.MUXCTRL = temp;
}

void AC1_MUXSET(uint8_t mode)
{
    uint8_t temp;
    temp = AC1.MUXCTRL;
    temp &= ~(AC_MUXPOS_gm | AC_MUXNEG_gm);
    temp |= mode;
    AC1.MUXCTRL = temp;
}

void AC0_MUXPOS(AC_MUXPOS_t mode)
{
    uint8_t temp;
    temp = AC0.MUXCTRL;
    temp &= ~AC_MUXPOS_gm;
    temp |= mode;
    AC0.MUXCTRL = temp;
}

void AC1_MUXPOS(AC_MUXPOS_t mode)
{
    uint8_t temp;
    temp = AC1.MUXCTRL;
    temp &= ~AC_MUXPOS_gm;
    temp |= mode;
    AC1.MUXCTRL = temp;
}

void AC0_MUXNEG(AC_MUXNEG_t mode)
{
    uint8_t temp;
    temp = AC0.MUXCTRL;
    temp &= ~AC_MUXNEG_gm;
    temp |= mode;
    AC0.MUXCTRL = temp;
}

void AC1_MUXNEG(AC_MUXNEG_t mode)
{
    uint8_t temp;
    temp = AC1.MUXCTRL;
    temp &= ~AC_MUXNEG_gm;
    temp |= mode;
    AC1.MUXCTRL = temp;
}

void AC0_DACrefValue (uint8_t value)
{
    AC0.DACREF = value;
}

void AC1_DACRefValue (uint8_t value)
{
    VREF.ACREF = VREF_REFSEL_VDD_gc | VREF_ALWAYSON_bm; 
    AC1.DACREF = value;
}

bool AC0_GetComparatorState(void)
{
    return ((AC0.STATUS & AC_CMPSTATE_bm) != 0 );
}

bool AC1_GetComparatorState(void)
{
    return ((AC1.STATUS & AC_CMPSTATE_bm) != 0 );
}

uint8_t AC0_GetWindowState(void)
{
    return (AC0.STATUS & AC_WINSTATE_gm);
}

uint8_t AC1_GetWindowState(void)
{
    return (AC1.STATUS & AC_WINSTATE_gm);
}

void AC1_EnableInterrupts(void)
{
    AC1.INTCTRL = AC_CMP_bm | AC_INTMODE_NORMAL_POSEDGE_gc;
}

void AC1_HandlerRegister(ac_irq_cb_t comparator_cb)
{
    AC1_cb = comparator_cb;
}
