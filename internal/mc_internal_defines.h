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

#ifndef MC_DEFINES_H
#define MC_DEFINES_H

#include "mc_internal_types.h"
#include "mc_config.h"
#include "mc_limits.h"

#define MC_MAX_SCALE_COMPARATOR              (256)
#define MC_MAX_SCALE_UNSIGNED                (65536.0)
#define MC_MAX_SCALE_SIGNED                  (32768.0)

#define PWM_PERIOD                           (uint8_t)(1000000.0 / (float)PWM_FREQUENCY + 0.5)      /* microseconds */

/* potentiometer conversion formulas */
#define MC_ADC_TO_PERCENT(X)                 ((float)(X) * 100.0 / MC_MAX_SCALE_UNSIGNED)
#define MC_PERCENT_TO_ADC(X)                 (uint16_t)((float)(X) * MC_MAX_SCALE_UNSIGNED / 100.0 + 0.5)

/* vbus  conversion formulas */
#define MC_ADC_TO_VOLTAGE(X)                 ((float)(X) * MC_VBUS_DIVIDER * MC_ADC_REFERENCE / MC_MAX_SCALE_UNSIGNED)
#define MC_VOLTAGE_TO_ADC(X)                 (uint16_t)((float)(X) * MC_MAX_SCALE_UNSIGNED / (MC_VBUS_DIVIDER * MC_ADC_REFERENCE) + 0.5)

/* temperature to Celsius degrees conversion formulas */
#define MC_ADC_TO_CELSIUS(X)                 ((float)((X) * (MC_ADC_REFERENCE * 1000.0 / MC_MAX_SCALE_UNSIGNED) - MC_TEMP_K1) / MC_TEMP_K2)
#define MC_CELSIUS_TO_ADC(X)                 (uint16_t)(((float)(X) * MC_TEMP_K2 + MC_TEMP_K1) * MC_MAX_SCALE_UNSIGNED / (MC_ADC_REFERENCE * 1000.0) + 0.5)

/* current to amps conversion formulas */
#define MC_ADC_TO_CURRENT(X)                 ((float)(X) * MC_ADC_REFERENCE / (MC_MAX_SCALE_SIGNED * MC_CURR_AMPLIFIER_GAIN * MC_SHUNT_RESISTANCE))
#define MC_CURENT_TO_ADC(X)                  (int16_t)((float)(X) * (MC_MAX_SCALE_SIGNED * MC_CURR_AMPLIFIER_GAIN * MC_SHUNT_RESISTANCE) / MC_ADC_REFERENCE + 0.5)

#define MC_CURR_TO_ILIM(X)                   (uint8_t)((MC_CURENT_TO_ADC(X) + MC_MAX_SCALE_SIGNED) / MC_MAX_SCALE_COMPARATOR)

/* mapping from 0 - 359.99 electrical degrees -> 0 - 65535 */
#define MC_DEG_TO_MCANGLE(DEG)               (mc_angle_t)(int32_t)( (float)(DEG) * 65536.0 / 360.0 + 0.5)
#define MC_MCANGLE_TO_DEG(MCANGLE)           ((float)(MCANGLE) * 360.0 / 65536.0 )

/* mapping from 0 - 359.99 electrical degrees -> 0 - 16777215 */
#define MC_DEG_TO_MCANGLE24(DEG)             (((int32_t)( (float)(DEG) * 65536.0 * 256.0 / 360.0 + 0.5)) & 16777215UL)
#define MC_MCANGLE24_TO_DEG(MCANGLE)         ((float)(MCANGLE) * 360.0 / (65536.0 * 256.0) )


#define MC_DT_COMPENSATE                     (fip_u16_t)(uint32_t)(32768.0 * ((float)PWM_DTH + (float)PWM_DTL) / (float)PWM_PERIOD + 0.5)                     

#define SCALE_FULL(VAL, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX)     (((VAL) - (IN_MIN)) * ((OUT_MAX) - (OUT_MIN)) / ((IN_MAX) - (IN_MIN)) + OUT_MIN)
#define SCALE_ZERO(VAL,         IN_MAX,          OUT_MAX)     ( (VAL)             *  (OUT_MAX)              /  (IN_MAX)                      )

#define MC_STARTUP_VOLTAGE                   ((MC_STARTUP_CURRENT * MC_MOTOR_PHASE_PHASE_RESISTANCE) + (MC_MOTOR_KV * MC_STARTUP_SPEED))

 #endif /* MC_DEFINES_H */

