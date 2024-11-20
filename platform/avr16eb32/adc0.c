/**
 * ADC0 Generated Driver File
 *
 * @file adc0.c
 * 
 * @ingroup adc0 
 * 
 * @brief This file contains the driver code for ADC0 module.
 * 
 * @version ADC0 Driver Version 1.1.0
*/

/*
© [2023] Microchip Technology Inc. and its subsidiaries.

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
#include "adc0.h"

adc_irq_cb_t ADC0_SampleReadyCallback = NULL;
adc_irq_cb_t ADC0_ResultReadyCallback = NULL;
adc_irq_cb_t ADC0_ErrorCallback = NULL;

int8_t ADC0_Initialize(void)
{     
     //PRESC System clock divided by 2; 
    ADC0.CTRLB = 0x0;

    //CHOPPING ENABLE; FREERUN disabled; LEFTADJ disabled; SAMPNUM 16 samples accumulated; 
    ADC0.CTRLF = 0x44;

    //REFSEL VDD; 
    ADC0.CTRLC = 0x0;

    //WINCM No Window Comparison; WINSRC RESULT; 
    ADC0.CTRLD = 0x0;

    //SAMPDUR 11;
    ADC0.CTRLE = 0xB;

    //GAIN 1x gain; PGABIASSEL 0% BIAS current.; PGAEN disabled; 
    ADC0.PGACTRL = 0x00;

    //DBGRUN disabled; 
    ADC0.DBGCTRL = 0x0;

    //DIFF enabled; MODE BURST; START Start a conversion immediately. This will be set back to STOP when the first conversion is done, unless Free-Running mode is enabled; 
    ADC0.COMMAND = 0x0;

    //RESOVR disabled; RESRDY disabled; SAMPOVR disabled; SAMPRDY disabled; TRIGOVR disabled; WCMP disabled; 
    ADC0.INTCTRL = 0x0;

    //MUXPOS ADC input pin GND; VIA Inputs not connected via PGA; 
    ADC0.MUXPOS = ADC_MUXPOS_GND_gc;

    //MUXNEG ADC input pin GND; VIA Inputs not connected via PGA; 
    ADC0.MUXNEG = ADC_MUXNEG_GND_gc;

    // Window comparator high threshold 
    ADC0.WINHT = 0x0;

    // Window comparator low threshold 
    ADC0.WINLT = 0x0;

    //ENABLE enabled; LOWLAT enabled; RUNSTDBY disabled; 
    ADC0.CTRLA = 0x21;

    return 0;
}

void ADC0_Enable(void)
{
    ADC0.CTRLA |= ADC_ENABLE_bm;
}

void ADC0_Disable(void)
{
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
}

void ADC0_SetWindowHigh(adc_result_t high)
{
    ADC0.WINHT = high;
}

void ADC0_SetWindowLow(adc_result_t low)
{
    ADC0.WINLT = low;
}

void ADC0_SetWindowMode(ADC0_window_mode_t mode)
{
    ADC0.CTRLD = mode;
}

void ADC0_SetWindowChannel(adc_0_channel_t channel)
{
    ADC0.MUXPOS &= ADC_VIA_gm;
    ADC0.MUXPOS |= channel;
}


void ADC0_SetMux(ADC_MUXPOS_t channel)
{
    ADC0.MUXPOS = channel;
    ADC0.MUXNEG = ADC_MUXNEG_GND_gc;
}

void ADC0_SetMuxSingleEnded(ADC_MUXPOS_t channel)
{
    ADC0.MUXPOS = channel;
    ADC0.MUXNEG = ADC_MUXNEG_GND_gc;
}

void ADC0_SetMuxDifferential(ADC_MUXPOS_t channel_p,  ADC_MUXNEG_t channel_n)
{
    ADC0.MUXPOS = channel_p;
    ADC0.MUXNEG = channel_n;
}

void ADC0_StartConversion(void)
{
    ADC0.COMMAND = ADC_MODE_BURST_SCALING_gc | ADC_START_IMMEDIATE_gc;
}

void ADC0_StartSingleEndedConversion(void)
{
    ADC0.COMMAND = ADC_MODE_BURST_SCALING_gc | ADC_START_IMMEDIATE_gc;
}

void ADC0_StartDiffConversion(void)
{
    ADC0.COMMAND = ADC_MODE_BURST_SCALING_gc | ADC_START_IMMEDIATE_gc | ADC_DIFF_bm;
}

void ADC0_DiffMode(void)
{
    ADC0.COMMAND |= ADC_DIFF_bm;
}

void ADC0_SingleMode(void)
{
    ADC0.COMMAND &= ~ADC_DIFF_bm;
}

void ADC0_StopConversion(void)
{
    ADC0.COMMAND = ADC_START_STOP_gc;
}

bool ADC0_IsConversionDone(void)
{
    return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

adc_result_t ADC0_GetConversionResult(void)
{
    return (ADC0.RESULT);
}

diff_adc_result_t ADC0_GetDiffConversionResult(void)
{
    return (ADC0.RESULT);
}

adc_result_t ADC0_GetConversionSample(void)
{
    return (ADC0.SAMPLE);
}

bool ADC0_GetWindowResult(void)
{
    bool temp = (ADC0.INTFLAGS & ADC_WCMP_bm);
    ADC0.INTFLAGS = ADC_WCMP_bm; // Clear intflag if set
    return temp;
}

adc_result_t ADC0_GetConversion(adc_0_channel_t channel)
{
    adc_result_t res;
    ADC0_SetMux(channel);
    ADC0_StartConversion();
    while (!ADC0_IsConversionDone());
    res = ADC0_GetConversionResult();
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    return res;
}

diff_adc_result_t ADC0_GetDiffConversion(bool enablePGA, adc_0_channel_t channel, adc_0_muxneg_channel_t channel1)
{
    diff_adc_result_t res;
    ADC0_SetMuxDifferential(channel, channel1|(enablePGA<<6));
    ADC0_StartDiffConversion();
    while (!ADC0_IsConversionDone());
    res = ADC0_GetConversionResult();
    ADC0.INTFLAGS |= ADC_RESRDY_bm;
    return res;
}

uint8_t ADC0_GetResolution(void)
{
    return (ADC0.COMMAND & ADC_MODE_SINGLE_8BIT_gc) ? 8 : 12;
}

void ADC0_SampleReadyCallbackRegister(adc_irq_cb_t callback)
{
    ADC0_SampleReadyCallback = callback;
}

void ADC0_ResultReadyCallbackRegister(adc_irq_cb_t callback)
{
    ADC0_ResultReadyCallback = callback;
}

void ADC0_ErrorCallbackRegister(adc_irq_cb_t callback)
{
    ADC0_ErrorCallback = callback;
}

ISR(ADC0_SAMPRDY_vect)
{
    //Clear the interrupt flag
    ADC0.INTFLAGS = ADC_SAMPRDY_bm;

    if (ADC0_SampleReadyCallback != NULL)
    {
        ADC0_SampleReadyCallback();
    }
}

ISR(ADC0_RESRDY_vect)
{
    //Clear the interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    if (ADC0_ResultReadyCallback != NULL)
    {
        ADC0_ResultReadyCallback();
    }
}

ISR(ADC0_ERROR_vect)
{
    //Clear the interrupt flag
    ADC0.INTFLAGS = ADC_TRIGOVR_bm;

    //Clear the interrupt flag
    ADC0.INTFLAGS = ADC_SAMPOVR_bm;

    //Clear the interrupt flag
    ADC0.INTFLAGS = ADC_RESOVR_bm;

    if (ADC0_ErrorCallback != NULL)
    {
        ADC0_ErrorCallback();
    }
}
