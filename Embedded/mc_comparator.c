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

#include "clkctrl.h"
#include <util/delay.h>
#include <avr/io.h>
#include "mc_comparator.h"
#include "ac.h"


#ifdef AVR16EB32
#include "evsys.h"
#include "wex0.h"
#endif /* AVR16EB32 */



#ifdef AVR16EB32
void AC_Pins_Init(void)
{
    PORTA.DIRSET   = PIN7_bm;  // output pin for comparator test
    PORTD.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;   // PD4 -> Motor Phase A -> MUXPOS INPUT -> 0,AINP5
    PORTD.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;   // PD5 -> Motor Phase B -> MUXPOS INPUT -> 0,AINP6
    PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;   // PD6 -> Motor Phase C -> MUXPOS INPUT -> 0,AINP3
    PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;   // PD0 -> Motor Phase Neutral -> MUXNEG INPUT -> 0,AINN1
    PORTD.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;   // Fault Detection

    /* Disable pull-up resistors */
    PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm;
    PORTD.PIN5CTRL &= ~PORT_PULLUPEN_bm;
    PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
    PORTD.PIN0CTRL &= ~PORT_PULLUPEN_bm;
    PORTD.PIN3CTRL &= ~PORT_PULLUPEN_bm; //Fault Detection    
}
#endif /* AVR16EB32 */



#ifdef AVR16EB32 // Hardware Implementation with specific calls for AVR-EB 
#define _MC_COMP_INIT()                 AC0_Initialize()
#define _MC_COMP_FAULT_INIT()           do{AC_Pins_Init(); AC1_Initialize(); AC1_MUXSET(AC_MUXPOS_AINP1_gc | AC_MUXNEG_DACREF_gc | AC_INITVAL_LOW_gc); AC1_DACRefValue(255); _delay_us(10); Evsys_Init();}while(0)
#define _MC_COMP_MUX_SET                AC0_MUXPOS
#define _MC_COMP_MUX_S0                 (AC_MUXPOS_AINP5_gc | AC_MUXNEG_AINN1_gc)          
#define _MC_COMP_MUX_S1                 (AC_MUXPOS_AINP6_gc | AC_MUXNEG_AINN1_gc) 
#define _MC_COMP_MUX_S2                 (AC_MUXPOS_AINP3_gc | AC_MUXNEG_AINN1_gc)
#define _MC_COMP_INPUT_GET              AC0_GetComparatorState
#define _MC_COMP_FAULT_GET              AC1_GetComparatorState
#define _MC_COMP_HANDLER_REGISTER(X)    WEX0_FAULTIsrCallbackRegister(X)
#define _MC_COMP_REFERENCE_SET(X)       AC1_DACRefValue(X)
#define _MC_COMP_INT_ENABLE             WEX0_FaultEnable
#define _MC_COMP_INT_DISABLE            WEX0_FaultDisable
#endif /* AVR16EB32 */


void MC_Comparator_Initialize(void)
{
    _MC_COMP_INIT();
}

void MC_Comparator_FaultInitialize(void)
{
    _MC_COMP_FAULT_INIT();
}

void MC_Comparator_MuxSet(uint8_t step)
{
   switch(step)
   {
       case MC_PHASE_FLOAT_A: _MC_COMP_MUX_SET(_MC_COMP_MUX_S0); break;
       case MC_PHASE_FLOAT_B: _MC_COMP_MUX_SET(_MC_COMP_MUX_S1); break;
       case MC_PHASE_FLOAT_C: _MC_COMP_MUX_SET(_MC_COMP_MUX_S2); break;
       default: break;   
   }
}

bool MC_Comparator_Get(void)
{
    return _MC_COMP_INPUT_GET();
}

void MC_Comparator_HandlerRegister(mc_comparator_handler_t cb)
{
    _MC_COMP_HANDLER_REGISTER(cb);
}

void MC_Comparator_Reference(uint8_t dacRef)
{
    _MC_COMP_REFERENCE_SET(dacRef);
}

bool MC_Comparator_Fault_Get(void)
{
    return _MC_COMP_FAULT_GET();
}

void MC_Comparator_Int_Enable(void)
{
    _MC_COMP_INT_ENABLE();
}

void MC_Comparator_Int_Disable(void)
{
    _MC_COMP_INT_DISABLE();
}

