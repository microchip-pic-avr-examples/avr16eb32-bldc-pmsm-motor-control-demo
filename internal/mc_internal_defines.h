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


/* potentiometer conversion formulas */
#define MC_POT_TO_PERCENT(X)                 ((float)(X) * 100.0 / MC_MAX_SCALE_UNSIGNED)
#define MC_PERCENT_TO_POT(X)                 (uint16_t)((float)(X) * MC_MAX_SCALE_UNSIGNED / 100.0 + 0.5)

/* vbus  conversion formulas */
#define MC_VBUS_TO_VOLTAGE(X)                ((float)(X) * MC_VBUS_DIVIDER * MC_VOLTAGE_REFFERENCE / MC_MAX_SCALE_UNSIGNED)
#define MC_VOLTAGE_TO_VBUS(X)                (uint16_t)((float)(X) * MC_MAX_SCALE_UNSIGNED / (MC_VBUS_DIVIDER * MC_VOLTAGE_REFFERENCE) + 0.5)

/* temperature to Celsius degrees conversion formulas */
#define MC_TEMP_TO_CELSIUS(X)                ((float)((X) * (MC_VOLTAGE_REFFERENCE * 1000.0 / MC_MAX_SCALE_UNSIGNED) - MC_TEMP_K1) / MC_TEMP_K2)
#define MC_CELSIUS_TO_TEMP(X)                (uint16_t)(((float)(X) * MC_TEMP_K2 + MC_TEMP_K1) * MC_MAX_SCALE_UNSIGNED / (MC_VOLTAGE_REFFERENCE * 1000.0) + 0.5)

/* current to amps conversion formulas */
#define MC_CURR_TO_AMPS(X)                   ((float)(X) * MC_VOLTAGE_REFFERENCE / (MC_MAX_SCALE_SIGNED * MC_CURR_SENSOR_GAIN * MC_SHUNT_RESISTANCE))
#define MC_AMPS_TO_CURR(X)                   (int16_t)((float)(X) * (MC_MAX_SCALE_SIGNED * MC_CURR_SENSOR_GAIN * MC_SHUNT_RESISTANCE) / MC_VOLTAGE_REFFERENCE + 0.5)

#define MC_CURR_TO_ILIM(X)                   (uint8_t) ((MC_AMPS_TO_CURR(X) + MC_MAX_SCALE_SIGNED) / MC_MAX_SCALE_COMPARATOR)

/* 0 - 359.99 degrees */
#define MC_DEG_TO_MCANGLE(DEG)               (mc_angle_t)(int32_t)( (float)(DEG) * 65536.0 / 360.0 + 0.5)
#define MC_MCANGLE_TO_DEG(MCANGLE)           ( (float)(MCANGLE) * 360.0 / 65536.0 )

 #endif /* MC_DEFINES_H */

