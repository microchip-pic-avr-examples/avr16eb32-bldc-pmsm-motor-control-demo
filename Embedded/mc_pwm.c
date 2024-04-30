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

#include <avr/interrupt.h>
#include <util/atomic.h>
#include "mc_pwm.h"
#include "clkctrl.h"
#include "mc_config.h"

#ifdef AVR16EB32
#include "tce0.h"
#include "wex0.h"
#endif /* AVR16EB32 */


#ifdef AVR16EB32
static const uint8_t wex_settings[8] = 
{
    0,                                                                                                      /* 0b000 */
    WEX_PGMOVR0_bm | WEX_PGMOVR1_bm,                                                                        /* 0b001 */
    WEX_PGMOVR2_bm | WEX_PGMOVR3_bm,                                                                        /* 0b010 */
    WEX_PGMOVR0_bm | WEX_PGMOVR1_bm | WEX_PGMOVR2_bm | WEX_PGMOVR3_bm,                                      /* 0b011 */
    WEX_PGMOVR4_bm | WEX_PGMOVR5_bm,                                                                        /* 0b100 */
    WEX_PGMOVR0_bm | WEX_PGMOVR1_bm | WEX_PGMOVR4_bm | WEX_PGMOVR5_bm,                                      /* 0b101 */
    WEX_PGMOVR2_bm | WEX_PGMOVR3_bm | WEX_PGMOVR4_bm | WEX_PGMOVR5_bm ,                                     /* 0b110 */
    WEX_PGMOVR0_bm | WEX_PGMOVR1_bm | WEX_PGMOVR2_bm | WEX_PGMOVR3_bm | WEX_PGMOVR4_bm | WEX_PGMOVR5_bm,    /* 0b111 */
};
#define _MC_PWM_PINS_SET()            do{PORTMUX.TCEROUTEA = PORTMUX_TCE0_PORTA_gc;PORTA.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm;}while(0)
#define _MC_PWM_PERIOD()              TCE0_PER_US_TO_TICKS(PWM_PERIOD, F_CPU, 1)
#define _MC_PWM_PERIOD_SET()          do{TCE0_PeriodSet(_MC_PWM_PERIOD()); TCE0_Compare3Set((uint16_t)((1.05-MC_BEMF_SAMPLING_POINT) * TCE0_PER_US_TO_TICKS(PWM_PERIOD, F_CPU, 1)));}while(0)
#define _MC_PWM_TRAP_DCY(X)           do{stepped_data=(X); Map(MC_FP_TO_FIPu16(1.0),MC_FP_TO_FIPu16(0.0)); TCE0_CompareChannels012BufferedSet(scaled_dcy0,scaled_dcy1,scaled_dcy2); WEX0_PatternGenerationOverrideBufferSet(wex_settings[(X)&(MC_PHASE_FLOAT_A|MC_PHASE_FLOAT_B|MC_PHASE_FLOAT_C)]);}while(0)
#define _MC_PWM_SINE_DCY(X,Y,Z)       TCE0_CompareChannels012BufferedSet((X),(Y),(Z))
#define _MC_PWM_INIT                  MC_TCE0_Initialize
#define _MC_PWM_COMPLEMENTARY()       do{MC_WEX0_Initialize(); WEX0_DeadTimeHighSideSet(WEX0_NS_TO_TICKS(PWM_DTH, F_CPU, 1)); WEX0_DeadTimeLowSideSet(WEX0_NS_TO_TICKS(PWM_DTL, F_CPU, 1));}while(0)
#define _MC_PWM_DRIVE_REGISTER(X)     TCE0_OverflowCallbackRegister(X)
#define _MC_PWM_SENSE_REGISTER(X)     TCE0_Compare3CallbackRegister(X)
#define _MC_PWM_START()               TCE0_Start()
#define _MC_PWM_STOP()                TCE0_Stop()
#define _MC_PWM_AMPLITUDE_SET(X)      do{ATOMIC_BLOCK(ATOMIC_RESTORESTATE){frac_amplitude=(X);} TCE0_AmplitudeSet(X);}while(0)
#define _MC_PWM_OUTPUT_DISABLE()      do{TCE0_OutputsEnable(0); WEX0_OutputOverrideEnable(0);}while(0)
#define _MC_PWM_OUTPUT_ENABLE()       do{TCE0_OutputsEnable(TCE_CMP0EN_bm| TCE_CMP1EN_bm | TCE_CMP2EN_bm); WEX0_OutputOverrideEnable(WEX_OUTOVEN0_bm | WEX_OUTOVEN1_bm | WEX_OUTOVEN2_bm | WEX_OUTOVEN3_bm | WEX_OUTOVEN4_bm | WEX_OUTOVEN5_bm);}while(0)
#define _MC_PWM_FAULT_CLEAR()         do{WEX0_SoftwareCommand(WEX_CMD_FAULTCLR_gc); WEX0_SoftwareCommand(WEX_CMD_UPDATE_gc);}while(0)
#endif /* AVR16EB32 */


static mc_amplitude_t frac_amplitude;
static mc_int_dcy_t   scaled_dcy0, scaled_dcy1, scaled_dcy2;
static mc_stepped_t   stepped_data;

#if (MC_SCALE_MODE == MC_SCALE_BOTTOM) && (MC_DRIVE_MODE == MC_STEPPED_MODE)

static void Map(mc_int_dcy_t top, mc_int_dcy_t bot)
{
    if(stepped_data & MC_PHASE_HIGH_A)     scaled_dcy0 = top;
    else if(stepped_data & MC_PHASE_LOW_A) scaled_dcy0 = bot;
    else                                   scaled_dcy0 = 0;
    if(stepped_data & MC_PHASE_HIGH_B)     scaled_dcy1 = top;
    else if(stepped_data & MC_PHASE_LOW_B) scaled_dcy1 = bot;
    else                                   scaled_dcy1 = 0;
    if(stepped_data & MC_PHASE_HIGH_C)     scaled_dcy2 = top;
    else if(stepped_data & MC_PHASE_LOW_C) scaled_dcy2 = bot;
    else                                   scaled_dcy2 = 0;
}
#endif  /* (MC_SCALE_MODE == MC_SCALE_BOTTOM) && (MC_DRIVE_MODE == MC_STEPPED_MODE)  */



#ifdef AVR16EB32
void MC_TCE0_Initialize(void)
{ 
    TCE0_PrescalerSet(TCE_CLKSEL_DIV1_gc);
    TCE0_CountDirectionClear();
    TCE0_ModeSet(TCE_WGMODE_SINGLESLOPE_gc);
    TCE0_CounterSet(0);
    TCE0_ScaleEnable(true);
    TCE0_AmplitudeControlEnable(true);
#if MC_SCALE_MODE == MC_SCALE_BOTTOM
    TCE0_ScaleModeSet(TCE_SCALEMODE_BOTTOM_gc);
#endif /* MC_SCALE_MODE == MC_SCALE_BOTTOM */
    TCE0_Interrupts_Enable(TCE_OVF_bm | TCE_CMP3_bm);  
}

void MC_WEX0_Initialize(void)
{
    WEX0_InputMatrixSet(WEX_INMX_DIRECT_gc);
    WEX0_UpdateSourceSet(WEX_UPDSRC_TCPWM0_gc);
    WEX0_DeadTimeInsertionSet(WEX_DTI0EN_bm | WEX_DTI1EN_bm | WEX_DTI2EN_bm);
    WEX0_PatternGenerationOutputSet(0x00);
    WEX0_PatternGenerationMode(true);
    
    /* Fault Init */
    WEX0_FaultAEventInputEnable(true);
    WEX0_FaultAEventFilter(WEX_FILTER_SAMPLE7_gc);
    WEX0_FaultDetectionAction(WEX_FDACT_LOW_gc);
    WEX0_FaultDetectionRestartMode(WEX_FDMODE_CBC_gc); 
}
#endif /* AVR16EB32 */


/* Fault Action - Recovery*/
void MC_PWM_ForceStop(void)
{
    _MC_PWM_OUTPUT_DISABLE();
}

void MC_PWM_ForceStart(void)
{
    _MC_PWM_FAULT_CLEAR();
    _MC_PWM_OUTPUT_ENABLE();
}

void MC_PWM_Initialize(void)
{
    _MC_PWM_PERIOD_SET();
    _MC_PWM_INIT();
    _MC_PWM_COMPLEMENTARY();
    _MC_PWM_PINS_SET();
    _MC_PWM_START();
    sei();
}

void MC_PWM_Start(void)
{
    _MC_PWM_START();
}

void MC_PWM_Stop(void)
{
    _MC_PWM_STOP();
}

void MC_PWM_DriveHandlerRegister(mc_handler_t cb)
{
    _MC_PWM_DRIVE_REGISTER(cb);
}

void MC_PWM_SenseHandlerRegister(mc_handler_t cb)
{
    _MC_PWM_SENSE_REGISTER(cb);
}

void MC_PWM_ContinuousScale(mc_fip_dcy_t fch0, mc_fip_dcy_t fch1, mc_fip_dcy_t fch2)
{
    _MC_PWM_SINE_DCY(fch0, fch1, fch2);
}

void MC_PWM_SteppedScale(mc_stepped_t data)
{
    _MC_PWM_TRAP_DCY(data);
}

void MC_PWM_AmplitudeSet(mc_amplitude_t amplitude)
{
    _MC_PWM_AMPLITUDE_SET(amplitude);
}

mc_amplitude_t MC_PWM_AmplitudeGet(void)
{
    mc_amplitude_t retval;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        retval = frac_amplitude;
    }
    return retval;
}

