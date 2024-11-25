/*
� [2024] Microchip Technology Inc. and its subsidiaries.
 
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

#ifndef MC_PWM_H
#define	MC_PWM_H

#include "mc_internal_types.h"
#include "mc_config.h"

#define MC_SCALE_CENTER                 (1)
#define MC_SCALE_BOTTOM                 (2)

/* possible values:  MC_SCALE_CENTER, MC_SCALE_BOTTOM  */
#define MC_SCALE_MODE                   MC_SCALE_BOTTOM


#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE

#ifdef __AVR16EB32__
#define MC_BEMF_SAMPLING_POINT          0.03
#endif /* __AVR16EB32__ */

#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */

#if MC_DRIVE_MODE == MC_STEPPED_MODE
#if MC_SCALE_MODE == MC_SCALE_BOTTOM
#define MC_BEMF_SAMPLING_POINT          0.95
#endif /* MC_SCALE_MODE == MC_SCALE_BOTTOM */
#if MC_SCALE_MODE == MC_SCALE_CENTER
#define MC_BEMF_SAMPLING_POINT          0.15
#endif /* MC_SCALE_MODE == MC_SCALE_CENTER */
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */


void MC_PWM_Initialize(void);
void MC_PWM_Start(void);
void MC_PWM_Stop(void);
void MC_PWM_HandlerRegister(mc_handler_t);
void MC_PWM_ContinuousScale(mc_fip_dcy_t, mc_fip_dcy_t, mc_fip_dcy_t);
void MC_PWM_SteppedScale(mc_stepped_t);
void MC_PWM_AmplitudeSet(mc_amplitude_t);
mc_amplitude_t MC_PWM_AmplitudeGet(void);
void MC_TimerSample_Initialize(void);
void MC_PWM_ForceStop(void);
void MC_PWM_ForceStart(void);
void MC_PWM_FaultRecovery(void);

#endif	/* MC_PWM_H */
