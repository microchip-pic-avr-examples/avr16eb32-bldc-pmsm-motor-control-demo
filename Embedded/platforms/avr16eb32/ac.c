/*
© [2024] Microchip Technology Inc. and its subsidiaries.
 
    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include <avr/io.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include "ac.h"


ac_irq_cb_t AC1_cb = NULL;


ISR(AC1_AC_vect)
{   
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
