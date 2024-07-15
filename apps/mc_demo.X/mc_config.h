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

#ifndef MC_CONFIG_H
#define	MC_CONFIG_H

#define MC_CONTINUOUS_MODE              (1)
#define MC_STEPPED_MODE                 (2)
#define MC_SENSORED_MODE                (3)
#define MC_SENSORLESS_MODE              (4)
#define MC_SCALE_CENTER                 (5)
#define MC_SCALE_BOTTOM                 (6)

/* possible values:  MC_SENSORED_MODE, MC_SENSORLESS_MODE  */
#define MC_CONTROL_MODE                 MC_SENSORLESS_MODE

/* possible values:  MC_CONTINUOUS_MODE, MC_STEPPED_MODE  */
#define MC_DRIVE_MODE                   MC_STEPPED_MODE

/* possible values:  MC_SCALE_CENTER, MC_SCALE_BOTTOM  */
#define MC_SCALE_MODE                   MC_SCALE_BOTTOM

/* motor specific settings */
#define MOTOR_HALL_DEVIATION            (0.00)   /* degrees */
#define MOTOR_HALL_INVERTED             true
#define MOTOR_PHASE_ADVANCE             (15.00)  /* degrees */
#define MC_MOTOR_PAIR_POLES             (4)      /* pole pairs */
#define MC_MIN_SPEED                    (400)    /* RPM - minimum speed */
#define MC_RAMP_UP_DURATION             (1000)   /* milliseconds */
#define MC_RAMP_DOWN_DURATION           (1000)   /* milliseconds */
#define MC_STARTUP_VOLTAGE              (1.0)    /* V peak */

/* PWM drive settings */
#define PWM_DTH                         (100)            /* nanoseconds */
#define PWM_DTL                         (100)            /* nanoseconds */  
#define PWM_PERIOD                      (50)             /* microseconds */

/* board specific settings */
#define MC_SHUNT_RESISTANCE             (0.01)   /* resistance in Ohm of the current shunt */
#define MC_CURR_SENSOR_GAIN             (7.5)
#define MC_VBUS_DIVIDER                 (16.0)
#define MC_VOLTAGE_REFFERENCE           (3.3)
#define MC_TEMP_K1                      (400.0)  /* mV - output voltage at 0 *C  */
#define MC_TEMP_K2                      (19.53)  /* mV / *C   */

/* application settings */
#define MC_DVRT_ENABLED                 false    /* MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED cannot be both true at the same time */
#define MC_PRINTOUT_ENABLED             true     /* MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED cannot be both true at the same time */
#define MC_PRINTOUT_REFRESH_INTERVAL    (1000)   /* milliseconds */

/* control functionality settings */
#define MC_STALL_EVENTS_THRESHOLD       (50)   /* number of phase errors before throwing a stall error, MAX 255 */
#define MC_STALL_ERROR_TOLERANCE        (60.0) /* angle error in degrees that conducts to a stall condition */
#define MC_SPEED_REGULATOR_EN           false  /* enables a basic speed regulator, not tunable */
#define MC_SYNCHRONIZED                 true   /* this enables the synchronization between stator and rotor, should be always enabled */
#define MC_FAULT_ENABLED                true   /* this enables the fault signalling and emergency stop, should be always enabled */


#endif	/* MC_CONFIG_H */

