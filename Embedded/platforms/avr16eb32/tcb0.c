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

#include <stddef.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "tcb0.h"



static TCB0_cb_t pfCallback = NULL;


void TCB0_CaptureCallbackRegister(TCB0_cb_t pf)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pfCallback = pf;
    }
}

ISR(TCB0_INT_vect)
{
    TCB0.INTFLAGS = TCB_CAPT_bm;
    if(pfCallback != NULL)
        pfCallback();
}

void TCB0_Initialize(void)
{
    pfCallback = NULL;

    //Compare or Capture
    TCB0.CCMP = 0xFFFF;

    //Count
    TCB0.CNT = 0x0;

    //ASYNC disabled; CCMPEN disabled; CCMPINIT disabled; CNTMODE INT; 
    TCB0.CTRLB = TCB_CNTMODE_INT_gc; //periodic interrupt
    
    //DBGRUN disabled; 
    TCB0.DBGCTRL = 0x0;

    //CAPTEI disabled; EDGE disabled; FILTER disabled; 
    TCB0.EVCTRL = 0x0;

    //CAPT enabled; OVF disabled; 
    TCB0.INTCTRL = TCB_CAPT_bm;

    //CAPT disabled; OVF disabled; 
    TCB0.INTFLAGS = 0x0;

    //Temporary Value
    TCB0.TEMP = 0x0;

    //CASCADE disabled; CLKSEL DIV1; ENABLE enabled; RUNSTDBY disabled; SYNCUPD disabled; 
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc;
}

void TCB0_Start(void)
{
    TCB0.CTRLA |= TCB_ENABLE_bm; /* Start Timer */
}

void TCB0_Stop(void)
{
    TCB0.CTRLA &= ~TCB_ENABLE_bm; /* Stop Timer */
}

void TCB0_Write(uint16_t timerVal)
{
    TCB0.CNT = timerVal;
}

uint16_t TCB0_Read(void)
{
    uint16_t readVal;

    readVal = TCB0.CNT;

    return readVal;
}

void TCB0_EnableCaptInterrupt(void)
{
    TCB0.INTCTRL |= TCB_CAPT_bm; /* Capture or Timeout: enabled */
}

void TCB0_EnableOvfInterrupt(void)
{
	TCB0.INTCTRL |= TCB_OVF_bm; /* Overflow Interrupt: enabled */
}

void TCB0_DisableCaptInterrupt(void)
{
    TCB0.INTCTRL &= ~TCB_CAPT_bm; /* Capture or Timeout: disabled */
}

void TCB0_DisableOvfInterrupt(void)
{
	TCB0.INTCTRL &= ~TCB_OVF_bm; /* Overflow Interrupt: disabled */

}

void TCB0_ClearCaptInterruptFlag(void)
{
    TCB0.INTFLAGS &= ~TCB_CAPT_bm;
}

void TCB0_ClearOvfInterruptFlag(void)
{
	TCB0.INTFLAGS &= ~TCB_OVF_bm;

}

bool TCB0_IsCaptInterruptEnabled(void)
{
    return ((TCB0.INTCTRL & TCB_CAPT_bm) > 0);
}

bool TCB0_IsOvfInterruptEnabled(void)
{
    return ((TCB0.INTCTRL & TCB_OVF_bm) > 0);
}

