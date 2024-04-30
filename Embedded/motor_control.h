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


#ifndef MC_CONTROL_H
#define MC_CONTROL_H

#include "mc_types.h"


/*
 * Initialization function, needs to be called before any other
 * to be called only from main, not from interrupt context
 */
void MC_Control_Initialize(void);


/*
 * Function that starts the motor
 * Parameter: direction, either MC_DIR_CW or MC_DIR_CCW,
 * for clockwise respectively counterclockwise
 * to be called only from main, not from interrupt context
 */
void MC_Control_SoftStart(mc_direction_t dir);


/* 
 * Function that stops the motor leaving the power switches turned off,
 * to be called only from main, not from interrupt context
 */
void MC_Control_SoftStop(void);


/* 
 * Function that performs a delay using a timer in the background
 * the parameter specifies the number of milliseconds of delay
 * it is recommended to use this macro instead of direct call: MC_DELAY_MS
 * it is also recommended in the application to run this delay instead of
 * long loops or other system delays
 * to be called only from main, not from interrupt context
 */
void MC_Control_DelayMs(uint32_t delay_ms);


/* 
 * Function that sets the reference point
 * when MC_SPEED_REGULATOR_EN is true, it sets the reference for the speed control loop
 * when MC_SPEED_REGULATOR_EN is false, it sets the drive amplitude, the speed will vary,
 * depending on power supply voltage, mechanical load
 * parameter: 16 bit number used as a reference
 * to be called only from main, not from interrupt context
 */
void MC_Control_ReferenceSet(uint16_t ref);


/* 
 * Function that returns the various analog measurements
 * return value type: mc_analog_data_t that can be converted
 * into human-readable format using macros defined in mc_defines.h
 * parameter: mc_analog_id_t - id of the measurement to be read
 * to be called only from main, not from interrupt context
 */
mc_analog_data_t MC_Control_AnalogRead(mc_analog_id_t id);


/* 
 * Function that returns the rotational speed expressed in mc_speed_t
 * the data of type mc_speed_t can be converted into RPM
 * using macro MC_MCSPEED_TO_RPM
 * to be called only from main, not from interrupt context
 */
mc_speed_t MC_Control_SpeedGet(void);


/* 
 * Function that registers a user's function as a callback
 * the user's function has a prototype 'void function(void);'
 * the user's function will be called within MC_Control_DelayMs context, not from interrupt context
 * MC_Control_PeriodicHandlerRegister to be called only from main, not from interrupt context
 */
void MC_Control_PeriodicHandlerRegister(mc_handler_t phandler);


/* 
 * Function that registers a user's function as a callback
 * the user's function has a prototype 'void function(mc_fault_event_t);'
 * the data of type mc_fault_event_t is an enumeration described in mc_types.h
 * the user's function will be only called from interrupt context
 * MC_Control_FaultNotificationRegister to be called only from main, not from interrupt context
 */
void MC_Control_FaultNotificationRegister(mc_fault_handler_t fhandler);


#define MC_DELAY_MS    MC_Control_DelayMs 


#endif /*  MC_CONTROL_H */


