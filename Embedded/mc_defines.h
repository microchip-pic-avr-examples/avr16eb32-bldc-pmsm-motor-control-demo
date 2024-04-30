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

#include "mc_types.h"


#define MC_STEPPED_MODE                 (1)
#define MC_SENSORED_MODE                (2)
#define MC_SENSORLESS_MODE              (3)
#define MC_SCALE_BOTTOM                 (4)

#define MC_F_SAMPLING                   (1000000.0 / (float)PWM_PERIOD)

#define MC_MAX_SCALE                         65536.0


/* potentiometer conversion formulas */
#define MC_POT_TO_PERCENT(X)                 ((float)(X) * 100.0 / MC_MAX_SCALE)
#define MC_PERCENT_TO_POT(X)                 (uint16_t)((float)(X) * MC_MAX_SCALE / 100.0 + 0.5)
#define MC_POT_FGET(X)                       ({mc_analog_data_t _q = (X); MC_POT_TO_PERCENT(_q.H);})
#define MC_POT_NGET(X)                       ({mc_analog_data_t _q = (X); _q.H;})

/* vbus  conversion formulas */
#define MC_VBUS_TO_VOLTAGE(X)                ((float)(X) * MC_VBUS_DIVIDER * MC_VOLTAGE_REFFERENCE / MC_MAX_SCALE)
#define MC_VOLTAGE_TO_VBUS(X)                (uint16_t)((float)(X) * MC_MAX_SCALE / (MC_VBUS_DIVIDER * MC_VOLTAGE_REFFERENCE) + 0.5)
#define MC_VBUS_FGET(X)                      ({mc_analog_data_t _q = (X); MC_VBUS_TO_VOLTAGE(_q.H);})
#define MC_VBUS_NGET(X)                      ({mc_analog_data_t _q = (X); _q.H;})

/* temperature to Celsius degrees conversion formulas */
#define MC_TEMP_TO_CELSIUS(X)                ((float)((X) * (MC_VOLTAGE_REFFERENCE * 1000.0 / MC_MAX_SCALE) - MC_TEMP_K1) / MC_TEMP_K2)
#define MC_CELSIUS_TO_TEMP(X)                (uint16_t)(((float)(X) * MC_TEMP_K2 + MC_TEMP_K1) * MC_MAX_SCALE / (MC_VOLTAGE_REFFERENCE * 1000.0) + 0.5)
#define MC_TEMP_FGET(X)                      ({mc_analog_data_t _q = (X); MC_TEMP_TO_CELSIUS(_q.H);})
#define MC_TEMP_NGET(X)                      ({mc_analog_data_t _q = (X); _q.H;})

/* these macros must not be called with variable arguments within interrupt context */
#define MC_RPM_TO_MCSPEED(RPM)               (mc_speed_t)(((float)(RPM) * 65536.0 * (float)(MC_MOTOR_PAIR_POLES)) / ((float)(MC_F_SAMPLING) * 60.0) + 0.5)
#define MC_MCSPEED_TO_RPM(MCSPEED)           (float)(MC_F_SAMPLING) * (float)(MCSPEED) * 60.0 / ( 65536.0 * (float)(MC_MOTOR_PAIR_POLES) )

/* 0 - 359.99 degrees */
#define MC_DEG_TO_MCANGLE(DEG)               (mc_angle_t)( (float)(DEG) * 65536.0 / 360.0 + 0.5)
#define MC_MCANGLE_TO_DEG(MCANGLE)           (float)( (float)(MCANGLE) * 360.0 / 65536.0 )


#endif /* MC_DEFINES_H */

