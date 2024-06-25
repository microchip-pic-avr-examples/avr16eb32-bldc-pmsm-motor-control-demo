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
 * File:   MC_Ramp.c
 * Author: M66878
 *
 * Created on November 10, 2022, 2:41 PM
 */

#include "mc_ramp.h"

#define NEGATIVE    true
#define POSITIVE    false

void MC_Ramp_Init(uint16_t begin, uint16_t end, uint16_t steps, mc_ramp_t* pData)
{
    if(pData == NULL) return;
    if(steps < 2)
    {
        pData->end_value = end;
        pData->accumulator.H16 = end;
        pData->accumulator.L16 = 0;
        pData->step_counter = 0;
        pData->step_size = 0;
        pData->step_sign = POSITIVE;
        pData->is_done = true;
        return;
    }

    if(begin < end)
    {
        pData->step_sign = POSITIVE;
        pData->step_size = end - begin;
    }
    else
    {
        pData->step_sign = NEGATIVE;
        pData->step_size = begin - end;
    }
    
    pData->step_counter = steps;
    steps--; 
    pData->step_size = ((pData->step_size)<<16)/steps;  
    pData->end_value = end;    
    pData->accumulator.H16 = begin;
    pData->accumulator.L16 = 0;    
    pData->is_done = false;
}

uint16_t MC_Ramp_Get(mc_ramp_t* pData)
{
    uint16_t ret_val;
    if(pData == NULL) return 0;
    if(pData->step_counter == 0)
    {
        pData->is_done = true;
        return (pData->accumulator.H16);
    }    
    
    ret_val = pData->accumulator.H16;
             
    pData->step_counter--;
    if(pData->step_counter == 0)
    {
        pData->accumulator.H16 = pData->end_value;
        pData->accumulator.L16 = 0;
        pData->is_done = true;
        ret_val = pData->accumulator.H16;
    }
    else
    {
        if(pData->step_sign == POSITIVE)
            pData->accumulator.W32 +=  pData->step_size;        
        else // pData->u16.step_sign == NEGATIVE
            pData->accumulator.W32 -=  pData->step_size;               
    }
    
    return ret_val;
}

