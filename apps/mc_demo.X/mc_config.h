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
#define MC_WAVE_SINE                    (5)
#define MC_WAVE_SVM                     (6)
#define MC_WAVE_SADDLE                  (7)

/* possible values:  MC_SENSORED_MODE, MC_SENSORLESS_MODE  */
#define MC_CONTROL_MODE                 MC_SENSORLESS_MODE

/* possible values:  MC_CONTINUOUS_MODE, MC_STEPPED_MODE  */
#define MC_DRIVE_MODE                   MC_STEPPED_MODE

/* possible values:  MC_WAVE_SINE, MC_WAVE_SVM, MC_WAVE_SADDLE */
#define MC_WAVE_PROFILE                 MC_WAVE_SADDLE


/* motor specific settings */
#define MOTOR_PHASE_ADVANCE             (15.0)   /* degrees */
#define MC_MOTOR_PAIR_POLES             (4)      /* pole pairs */
#define MC_MOTOR_PHASE_PHASE_RESISTANCE (0.4)    /* ohm */
#define MC_MOTOR_KV                     (0.007)  /* volt/rpm */
#define MC_RAMP_UP_DURATION             (1000)   /* milliseconds */
#define MC_RAMP_DOWN_DURATION           (0)      /* milliseconds */
#define MC_STARTUP_CURRENT              (2.5)    /* initial current amplitude [amperes] */
#define MC_STARTUP_SPEED                (400)    /* switchover speed [rpm] */

/* PWM drive settings */
/* possible values:  15000 - 45000 Hz */
#define PWM_FREQUENCY                   (20000)  /* Hz */
#define PWM_DTH                         (100)    /* nanoseconds */
#define PWM_DTL                         (100)    /* nanoseconds */

/* board specific settings */
#define MC_SHUNT_RESISTANCE             (0.01)   /* resistance in Ohm of the current shunt */
#define MC_CURR_AMPLIFIER_GAIN          (7.5)
#define MC_VBUS_DIVIDER                 (16.0)
#define MC_ADC_REFERENCE                (3.3)    /* volt */
#define MC_TEMP_K1                      (400.0)  /* mV - output voltage at 0 *C  */
#define MC_TEMP_K2                      (19.53)  /* mV / *C   */

/* application settings */
#define MC_DVRT_ENABLED                 false    /* MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED cannot be both true at the same time */
#define MC_PRINTOUT_ENABLED             true     /* MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED cannot be both true at the same time */
#define MC_PRINTOUT_REFRESH_INTERVAL    (1000)   /* milliseconds */

/* control functionality settings */
#define MC_SPEED_REGULATOR_EN           true     /* enables a basic speed regulator, not tunable */
#define MC_SPEED_REGULATOR_MIN          (0500.0) /* min speed = pot min */
#define MC_SPEED_REGULATOR_MAX          (3500.0) /* max speed = pot max */
#define MC_SYNCHRONIZED                 true     /* this enables the synchronization between stator and rotor, should be always enabled */
#define MC_FAULT_ENABLED                true     /* this enables the fault signalling and emergency stop, should be always enabled */


#endif	/* MC_CONFIG_H */

