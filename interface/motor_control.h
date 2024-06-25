/**
 *  @defgroup mclib AVR® MCU Motor Control Library
 *
 *  @brief The AVR® MCU Motor Control Library is a library for the AVR-EB family
 *         of devices with support for spinning a BLDC/PMSM motor using either a
 *         sensored or sensorless setup. The library interface offers motor specific
 *         and power board customizations, as well as MCU settings and pinout
 *         functionality. The drive algorithm takes advantage of the MCU peripherals
 *         that ensures the CPU doesn't have a big overhead and optimizes the memory
 *         usage and resources consumption. User layer APIs are generated for a simple
 *         run-time configuration and control.
 **/

/**
 *  @file motor_control.h
 *
 *  @ingroup mclib
 *
 *  @brief This Header File contains the declarations of the functions public to
 *         the user.
 *
 *  @version AVR® MCU Motor Control Library v1.1.0
 *
 *  @copyright © 2024 Microchip Technology Inc. and its subsidiaries.
 *
 *  Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. You're responsible
 *  for complying with 3rd party license terms applicable to your use of 3rd
 *  party software (including open source software) that may accompany
 *  Microchip software.
 *
 *  SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 *  NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 *  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 *  WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 *  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 *  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
 *  RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 *  DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 **/


#ifndef MC_CONTROL_H
#define MC_CONTROL_H

#include "mc_public_types.h"

#define MC_F_SAMPLING                        (1000000.0 / (float)PWM_PERIOD)

/* these macros must not be called with variable arguments within interrupt context */
#define MC_RPM_TO_MCSPEED(RPM)               (mc_speed_t)(((float)(RPM) * 65536.0 * (float)(MC_MOTOR_PAIR_POLES)) / ((float)(MC_F_SAMPLING) * 60.0) + 0.5)
#define MC_MCSPEED_TO_RPM(MCSPEED)           (float)(MC_F_SAMPLING) * (float)(MCSPEED) * 60.0 / ( 65536.0 * (float)(MC_MOTOR_PAIR_POLES) )

/**
 * @ingroup mclib
 * @note This function must be called from normal context, not from interrupt context.
 *       This function must be called before any other function in the library.
 * @brief Function that initializes the AVR® MCU Motor Control Library.
 * @param None.
 * @return None.
 */
void MC_Control_Initialize(void);


/**
 * @ingroup mclib
 * @note This function must be called from normal context, not from interrupt context.
 *       It is recommended to use the MC_DELAY_MS macro instead of a direct call.
 *       Use this in the application to create delays instead of long loops or
 *       other system delays.
 * @brief Function that performs a delay using a timer in the background
 * @param[in] delay_ms The number of milliseconds of delay.
 * @return None.
 */
void MC_Control_DelayMs(uint32_t delay_ms);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context. Sets the reference point. 
 *       When the speed regulator is enabled, it sets
 *       the reference for the speed control loop. When the speed regulator
 *       is disabled, it sets the drive amplitude, and the speed will vary depending
 *       on the power supply voltage and mechanical load
 * @brief Function that sets the reference point.
 * @param[in] ref The reference value to be set.
 * @return None.
 */
void MC_Control_ReferenceSet(uint16_t ref);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context.
 * @brief Function that returns potentiometer value in percentage
 * @param None.
 * @return Potentiometer value in percentage.
 */
uint8_t MC_Control_PotentiometerRead(void);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context.
 * @brief Function that returns potentiometer value in uint16_t format
 * @param None.
 * @return Potentiometer value.
 */
uint16_t MC_Control_FastPotentiometerRead(void);


/**
 * @ingroup mclib
 * @note  Call this function only from main, not from interrupt context.
 * @brief Function that returns the voltage bus value in volts
 * @param None.
 * @return Voltage bus value.
 */
uint16_t MC_Control_VoltageBusRead(void);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context.
 * @brief Function that returns the MOSFET transistors temperature value in celsius degrees
 * @param None.
 * @return MOSFET transistors temperature value in celsius degrees.
 */
uint8_t MC_Control_TemperatureRead(void);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context.
 * @brief Function that returns the mean current value in milliamperes
 * @param None.
 * @return Mean current value.
 */
int16_t MC_Control_CurrentRead(void);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context. 
 *       The data of type mc_speed_t can be converted into RPM
 * using macro MC_MCSPEED_TO_RPM
 * @brief Function that returns the rotational speed expressed in mc_speed_t. 
 * @param None.
 * @return Rotational speed expressed.
 */
mc_speed_t MC_Control_SpeedGet(void);



/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context. 
 *       The user's function has a prototype 'void function(void);'. 
 *       The user's function will be called within MC_Control_DelayMs context, not from interrupt context
 * @brief Function that registers a user's function as a callback. 
 *        The user's function has a prototype 'void function(void);'
 * @param[in] phandler
 * @return None.
 */
void MC_Control_PeriodicHandlerRegister(mc_status_handler_t phandler);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context. 
 *       The user's function has a prototype 'void function(mc_fault_event_t);'. 
 *       The data of type mc_fault_event_t is an enumeration described in mc_public_types.h.
 *       user's function will be only called from interrupt context.
 * @brief Function that registers a user's function as a callback.
 * @param[in] fhandler
 * @return None.
 */
//void MC_Control_FaultNotificationRegister(mc_fault_handler_t fhandler);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context. 
 *       The data of type mc_status_t is an enumeration described in mc_public_types.h.
 * @brief Function that returns the motor status
 * @param None.
 * @return Motor status.
 */
mc_status_t MC_Control_StatusGet(void);


/**
 * @ingroup mclib
 * @note Call this function only from main, not from interrupt context. 
 *       The data of type mc_direction_t is an enumeration described in mc_public_types.h.
 * @brief Function that starts or stops the motor when the start/stop event is received
 * @param[in] dir The motor direction to be set.
 * @return None.
 */
void MC_Control_StartStop(mc_direction_t dir);

#define MC_DELAY_MS    MC_Control_DelayMs 


#endif /*  MC_CONTROL_H */


