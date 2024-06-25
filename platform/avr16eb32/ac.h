/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef AC_H
#define	AC_H

#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

typedef void (*ac_irq_cb_t)(void);

void AC0_Start(void);
void AC0_Stop(void);
void AC0_HysteresisMode(AC_HYSMODE_t mode);
void AC0_PowerProfile(AC_POWER_t mode);
void AC0_OutputEnable(void);
void AC0_OutputDisable(void);
void AC0_RunStandby(bool en);

void AC1_Start(void);
void AC1_Stop(void);
void AC1_HysteresisMode(AC_HYSMODE_t mode);
void AC1_PowerProfile(AC_POWER_t mode);
void AC1_OutputEnable(void);
void AC1_OutputDisable(void);
void AC1_RunStandby(bool en);

void AC0_Initialize(void);
void AC1_Initialize(void);

void AC0_WindowSelectionMode(AC_WINSEL_t mode);
void AC1_WindowSelectionMode(AC_WINSEL_t mode);

void AC0_InvertAcOutput(bool inv);
void AC1_InvertAcOutput(bool inv);
void AC0_InitialValueHigh(bool en);
void AC1_InitialValueHigh(bool en);
void AC0_MUXSET(uint8_t mode);
void AC1_MUXSET(uint8_t mode);
void AC0_MUXPOS(AC_MUXPOS_t mode);
void AC1_MUXPOS(AC_MUXPOS_t mode);
void AC0_MUXNEG(AC_MUXNEG_t mode);
void AC1_MUXNEG(AC_MUXNEG_t mode);

void AC0_DACrefValue (uint8_t value);
void AC1_DACRefValue (uint8_t value);

bool AC0_GetComparatorState(void);
bool AC1_GetComparatorState(void);
uint8_t AC0_GetWindowState(void);
uint8_t AC1_GetWindowState(void);

void AC1_HandlerRegister(ac_irq_cb_t comparator_cb);
void AC1_EnableInterrupts(void);

#endif	/* AC_H */
