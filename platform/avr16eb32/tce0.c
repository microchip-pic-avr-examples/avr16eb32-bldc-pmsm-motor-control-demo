/*******************************************************************************
 *  Timer/Counter E Embedded Driver API Source File
 *
 * @file  tce0.c
 *
 * @defgroup tce0 TCE0
 *
 * @brief 	This document contains the implementation of public and private functions for the Timer Counter E module.
 *
 * @version TCE0 Driver Version 1.0.0
 
*******************************************************************************/

/*******************************************************************************
Copyright (c) [2023] released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

/**
  Section: Included files
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stddef.h>
#include "tce0.h"

/**
  Section: TCE0 Private data
*/

/**
 * @ingroup tce0
 * @brief Function pointers that store the callback addresses.
 */
static TCE0_cb_t TCE0_OVF_isr_cb = NULL;
static TCE0_cb_t TCE0_CMP0_isr_cb = NULL;
static TCE0_cb_t TCE0_CMP1_isr_cb = NULL;
static TCE0_cb_t TCE0_CMP2_isr_cb = NULL;
static TCE0_cb_t TCE0_CMP3_isr_cb = NULL;

/**
 * @ingroup tce0
 * @brief Boolean that is true when timer is active.
 */
static volatile bool timerActive = false;

/**
 * @ingroup tce0
 * @brief 8-bit variable that replicates TCE_WGMODE_t enum to reflect the active mode.
 */
static volatile uint8_t timerMode = TCE_WGMODE_FRQ_gc;


/**
 * @ingroup tce0
 * @brief Interrupt Service Routine (ISR) for the overflow (OVF) interrupt.
 * @param None.
 * @return None.
 */
ISR(TCE0_OVF_vect)
{
  TCE0.INTFLAGS = TCE_OVF_bm;
  if(TCE0_OVF_isr_cb != NULL)
  {
      TCE0_OVF_isr_cb();
  }
}

/**
 * @ingroup tce0
 * @brief ISR for the CMP0 interrupt.
 * @param None.
 * @return None.
 */
ISR(TCE0_CMP0_vect)
{
  TCE0.INTFLAGS = TCE_CMP0_bm;
  if(TCE0_CMP0_isr_cb != NULL)
  {
      TCE0_CMP0_isr_cb();
  }
}


/**
 * @ingroup tce0
 * @brief ISR for the CMP1 interrupt.
 * @param None.
 * @return None.
 */
ISR(TCE0_CMP1_vect)
{
  TCE0.INTFLAGS = TCE_CMP1_bm;
  if(TCE0_CMP1_isr_cb != NULL)
  {
      TCE0_CMP1_isr_cb();
  }
}


/**
 * @ingroup tce0
 * @brief ISR for the CMP2 interrupt.
 * @param None.
 * @return None.
 */
ISR(TCE0_CMP2_vect)
{
  TCE0.INTFLAGS = TCE_CMP2_bm;
  if(TCE0_CMP2_isr_cb != NULL)
  {
      TCE0_CMP2_isr_cb();
  }
}


/**
 * @ingroup tce0
 * @brief ISR for the CMP3 interrupt.
 * @param None.
 * @return None.
 */
ISR(TCE0_CMP3_vect)
{
  TCE0.INTFLAGS = TCE_CMP3_bm;
  if(TCE0_CMP3_isr_cb != NULL)
  {
      TCE0_CMP3_isr_cb();
  }
}

void TCE0_OverflowCallbackRegister(TCE0_cb_t cb)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCE0_OVF_isr_cb = cb;
    } 
}

void TCE0_Compare0CallbackRegister(TCE0_cb_t cb)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCE0_CMP0_isr_cb = cb;
    }
}

void TCE0_Compare1CallbackRegister(TCE0_cb_t cb)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCE0_CMP1_isr_cb = cb;
    }
}

void TCE0_Compare2CallbackRegister(TCE0_cb_t cb)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCE0_CMP2_isr_cb = cb;
    }
}

void TCE0_Compare3CallbackRegister(TCE0_cb_t cb)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        TCE0_CMP3_isr_cb = cb;
    }
}

void TCE0_Initialize(void)
{
  timerMode = TCE_WGMODE_NORMAL_gc;

  TCE0_OVF_isr_cb  = NULL;
  TCE0_CMP0_isr_cb = NULL;
  TCE0_CMP1_isr_cb = NULL;
  TCE0_CMP2_isr_cb = NULL;
  TCE0_CMP3_isr_cb = NULL;
  TCE0.CTRLA = 0x00;
  TCE0.INTCTRL = 0x00;

  TCE0.CTRLB = 0x00;
  TCE0.CTRLC = 0x00;
  TCE0.CTRLD = 0x00;
  TCE0.CTRLECLR = 0x00;
  TCE0.CTRLESET = 0x00;
  TCE0.CTRLFCLR = 0x00;
  TCE0.CTRLFSET = 0x00;
  TCE0.EVGENCTRL = 0x00;
  TCE0.EVCTRL = 0x00;
  TCE0.DBGCTRL = 0x00;
  TCE0.TEMP = 0x00;
  TCE0.CNT = 0x0000;
  TCE0.AMP = 0x0000;
  TCE0.OFFSET = 0x0000;
  TCE0.PER = 0xFFFF;
  TCE0.CMP0 = 0x0000;
  TCE0.CMP1 = 0x0000;
  TCE0.CMP2 = 0x0000;
  TCE0.CMP3 = 0x0000;
  TCE0.INTFLAGS = TCE0.INTFLAGS;
  TCE0.CTRLA = 0x00;
  timerActive = false;
}


void TCE0_Deinitialize(void)
{
  TCE0.CTRLA = 0x00;
  TCE0.INTCTRL = 0x00;

  TCE0.CTRLB = 0x00;
  TCE0.CTRLC = 0x00;
  TCE0.CTRLD = 0x00;
  TCE0.CTRLECLR = 0x00;
  TCE0.CTRLESET = 0x00;
  TCE0.CTRLFCLR = 0x00;
  TCE0.CTRLFSET = 0x00;
  TCE0.EVGENCTRL = 0x00;
  TCE0.EVCTRL = 0x00;
  TCE0.DBGCTRL = 0x00;
  TCE0.TEMP = 0x00;
  TCE0.CNT = 0x0000;
  TCE0.AMP = 0x0000;
  TCE0.OFFSET = 0x0000;
  TCE0.PER = 0xFFFF;
  TCE0.CMP0 = 0x0000;
  TCE0.CMP1 = 0x0000;
  TCE0.CMP2 = 0x0000;
  TCE0.CMP3 = 0x0000;

  TCE0_OVF_isr_cb  = NULL;
  TCE0_CMP0_isr_cb = NULL;
  TCE0_CMP1_isr_cb = NULL;
  TCE0_CMP2_isr_cb = NULL;
  TCE0_CMP3_isr_cb = NULL;

  timerActive = false;

  TCE0.INTFLAGS = TCE0.INTFLAGS;
}

void TCE0_Start(void)
{
  TCE0.CTRLA |=  TCE_ENABLE_bm;
  timerActive = true;
}

void TCE0_Stop(void)
{
  TCE0.CTRLA &= ~TCE_ENABLE_bm;
  timerActive = false;
}

TCE0_status_t TCE0_StatusGet(void)
{
  TCE0_status_t retval;
  if((TCE0.CTRLA & TCE_ENABLE_bm) != 0)
  {
    retval = TCE_STATUS_RUNNING;
  }
  else
  {
    retval = TCE_STATUS_IDLE;
  }
  return retval;
}

void TCE0_ModeSet(TCE_WGMODE_t mode)
{
  uint8_t temp;
  if((mode == TCE_WGMODE_NORMAL_gc) ||
     (mode == TCE_WGMODE_FRQ_gc) ||
     (mode == TCE_WGMODE_SINGLESLOPE_gc) ||
     (mode == TCE_WGMODE_DSTOP_gc) ||
     (mode == TCE_WGMODE_DSBOTH_gc) ||
     (mode == TCE_WGMODE_DSBOTTOM_gc))
  {
    if(timerActive)
    {
      TCE0.CTRLA &= ~TCE_ENABLE_bm;

      temp = (TCE0.CTRLB & ~TCE_WGMODE_gm) |
             (  mode     &  TCE_WGMODE_gm);
      TCE0.CTRLB = temp;
      timerMode = mode;

      TCE0.CTRLA |= TCE_ENABLE_bm;
    }
    else
    {
      temp = (TCE0.CTRLB & ~TCE_WGMODE_gm) |
             (  mode     &  TCE_WGMODE_gm);
      TCE0.CTRLB = temp;
      timerMode = mode;
    }
  }
}

void TCE0_Interrupts_Enable(uint8_t value)
{
  TCE0.INTCTRL |= value & (TCE_OVF_bm | TCE_CMP0_bm | TCE_CMP1_bm | TCE_CMP2_bm | TCE_CMP3_bm);
}

void TCE0_Interrupts_FlagsClear(uint8_t value)
{
  TCE0.INTFLAGS = value & (TCE_OVF_bm | TCE_CMP0_bm | TCE_CMP1_bm | TCE_CMP2_bm | TCE_CMP3_bm);
}

uint8_t TCE0_Interrupts_FlagsGet(void)
{
  return TCE0.INTFLAGS;
}

void TCE0_Interrupts_Disable(uint8_t value)
{
  TCE0.INTCTRL &= ~(value & (TCE_OVF_bm | TCE_CMP0_bm | TCE_CMP1_bm | TCE_CMP2_bm | TCE_CMP3_bm));
}

void TCE0_Event_OutputMode(uint8_t value)
{
  uint8_t temp;
  if(timerActive)
  {
    TCE0.CTRLA &= ~TCE_ENABLE_bm;

    temp = (TCE0.CTRLB & ~(TCE_CMP0EV_bm | TCE_CMP1EV_bm | TCE_CMP2EV_bm | TCE_CMP3EV_bm)) |
           ( value     &  (TCE_CMP0EV_bm | TCE_CMP1EV_bm | TCE_CMP2EV_bm | TCE_CMP3EV_bm));
    TCE0.CTRLB = temp;

    TCE0.CTRLA |= TCE_ENABLE_bm;
  }
  else
  {
    temp = (TCE0.CTRLB & ~(TCE_CMP0EV_bm | TCE_CMP1EV_bm | TCE_CMP2EV_bm | TCE_CMP3EV_bm)) |
           ( value     &  (TCE_CMP0EV_bm | TCE_CMP1EV_bm | TCE_CMP2EV_bm | TCE_CMP3EV_bm));
    TCE0.CTRLB = temp;
  }
}

void TCE0_Event_InputConfig(uint8_t value)
{
  if(timerActive)
  {
    TCE0.CTRLA &= ~TCE_ENABLE_bm;
    TCE0.EVCTRL = value;
    TCE0.CTRLA |= TCE_ENABLE_bm;
  }
  else
  {
    TCE0.EVCTRL = value;
  }
}

void TCE0_SoftwareCommand(TCE_CMD_t com)
{
  uint8_t temp;
  temp = (TCE0.CTRLESET & ~TCE_CMD_gm) |
         (   com        &  TCE_CMD_gm);
  TCE0.CTRLESET = temp;
}

void TCE0_StandBySleep(bool state)
{
  if(state == true)
  {
    TCE0.CTRLA |= TCE_RUNSTDBY_bm;
  }
  else
  {
    TCE0.CTRLA &= ~TCE_RUNSTDBY_bm;
  }
}

void TCE0_DebugRun(bool state)
{
  if(state == true)
  {
    TCE0.DBGCTRL |= TCE_DBGRUN_bm;
  }
  else
  {
    TCE0.DBGCTRL &= ~TCE_DBGRUN_bm;
  }
}

uint16_t TCE0_CounterGet(void)
{
  return TCE0.CNT;
}

void TCE0_CounterSet(uint16_t value)
{
  TCE0.CNT = value;
}

void TCE0_PrescalerSet(TCE_CLKSEL_t prescaler)
{
  uint8_t temp;
  temp = (TCE0.CTRLA & ~TCE_CLKSEL_gm) |
         ( prescaler &  TCE_CLKSEL_gm);
  TCE0.CTRLA = temp;
}

void TCE0_Compare0Set(uint16_t value)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP0 = value;
    }
}

void TCE0_Compare1Set(uint16_t value)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP1 = value;
    }
}

void TCE0_Compare2Set(uint16_t value)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP2 = value;
    }
}

void TCE0_Compare3Set(uint16_t value)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP3 = value;
    }
}

void TCE0_CompareAllChannelsSet(uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP0 = value0;
       TCE0.CMP1 = value1;
       TCE0.CMP2 = value2;
       TCE0.CMP3 = value3;
    }
}

void TCE0_CompareChannels012Set(uint16_t value0, uint16_t value1, uint16_t value2)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP0 = value0;
       TCE0.CMP1 = value1;
       TCE0.CMP2 = value2;
    }  
}

void TCE0_CompareChannels123Set(uint16_t value1, uint16_t value2, uint16_t value3)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.CMP1 = value1;
       TCE0.CMP2 = value2;
       TCE0.CMP3 = value3;
    }
}

void TCE0_CompareAllChannelsBufferedSet(uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3)
{
    TCE0.CMP0BUF = value0;
    TCE0.CMP1BUF = value1;
    TCE0.CMP2BUF = value2;
    TCE0.CMP3BUF = value3;
}

void TCE0_CompareChannels012BufferedSet(uint16_t value0, uint16_t value1, uint16_t value2)
{
    TCE0.CMP0BUF = value0;
    TCE0.CMP1BUF = value1;
    TCE0.CMP2BUF = value2;
}

void TCE0_CompareChannels123BufferedSet(uint16_t value1, uint16_t value2, uint16_t value3)
{
    TCE0.CMP1BUF = value1;
    TCE0.CMP2BUF = value2;
    TCE0.CMP3BUF = value3;
}

void TCE0_OutputsEnable(uint8_t value)
{
  uint8_t temp;
  temp = (TCE0.CTRLB & ~(TCE_CMP0EN_bm| TCE_CMP1EN_bm | TCE_CMP2EN_bm | TCE_CMP3EN_bm)) |
         (   value   &  (TCE_CMP0EN_bm| TCE_CMP1EN_bm | TCE_CMP2EN_bm | TCE_CMP3EN_bm));
  TCE0.CTRLB = temp;
}

void TCE0_OutputsValueSet(uint8_t value)
{
  TCE0.CTRLC = value;
}

uint8_t TCE0_OutputsValueGet(void)
{
  return TCE0.CTRLC;
}

void TCE0_HighResSet(TCE_HREN_t res)
{
  uint8_t temp;
  temp = (TCE0.CTRLD & ~TCE_HREN_gm) |
         (  res      &  TCE_HREN_gm);
  TCE0.CTRLD = temp;
}


void TCE0_ScaleModeSet(TCE_SCALEMODE_t mode)
{
  uint8_t temp;
  temp = (TCE0.CTRLD & ~TCE_SCALEMODE_gm) |
         (    mode   &  TCE_SCALEMODE_gm);
  TCE0.CTRLD = temp;
}

void TCE0_ScaleEnable(bool state)
{
  if(state == true)
  {
    TCE0.CTRLD |= TCE_SCALE_bm;
  }
  else
  {
    TCE0.CTRLD &= ~TCE_SCALE_bm;
  }
}

void TCE0_AmplitudeControlEnable(bool state)
{
  if(state == true)
  {
    TCE0.CTRLD |= TCE_AMPEN_bm;
  }
  else
  {
    TCE0.CTRLD &= ~TCE_AMPEN_bm;
  }
}

void TCE0_AmplitudeSet(uint16_t value)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      TCE0.AMP = value;
    }
}

uint16_t TCE0_AmplitudeGet(void)
{
  return TCE0.AMP;
}

void TCE0_OffsetSet(uint16_t value)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      TCE0.OFFSET = value;
    }    
}

uint16_t TCE0_OffsetGet(void)
{
  return TCE0.OFFSET;
}

void TCE0_PeriodSet(uint16_t period)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
       TCE0.PER = period;
    }
}

void TCE0_CountDirectionSet(void)
{
  TCE0.CTRLESET = TCE_DIR_bm;
}

void TCE0_CountDirectionClear(void)
{
  TCE0.CTRLECLR = TCE_DIR_bm;
}

void TCE0_LockUpdateSet(void)
{
  TCE0.CTRLESET = TCE_LUPD_bm;
}

void TCE0_LockUpdateClear(void)
{
  TCE0.CTRLECLR = TCE_LUPD_bm;
}

void TCE0_AutoLockUpdateSet(void)
{
  TCE0.CTRLB |= TCE_ALUPD_bm;
}

void TCE0_AutoLockUpdateClear(void)
{
  TCE0.CTRLB &= ~TCE_ALUPD_bm;
}

void TCE0_PWM_BufferedDutyCycle0Set(uint16_t value)
{
  TCE0.CMP0BUF = value;
}

void TCE0_PWM_BufferedDutyCycle1Set(uint16_t value)
{
  TCE0.CMP1BUF = value;
}

void TCE0_PWM_BufferedDutyCycle2Set(uint16_t value)
{
  TCE0.CMP2BUF = value;
}

void TCE0_PWM_BufferedDutyCycle3Set(uint16_t value)
{
  TCE0.CMP3BUF = value;
}

void TCE0_PeriodBufferSet(uint16_t value)
{
  TCE0.PERBUF = value;
}

