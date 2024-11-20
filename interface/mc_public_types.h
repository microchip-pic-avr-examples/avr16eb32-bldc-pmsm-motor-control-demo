/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

#ifndef MC_PUBLIC_TYPES_H
#define	MC_PUBLIC_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**< Enum for motor direction. */
typedef enum
{
    MC_DIR_CW,
    MC_DIR_CCW
} mc_direction_t;

/**< Enum for fault events. */
typedef enum
{
    MC_FAULT_STALL_MASK        = (1 << 0),
    MC_FAULT_OVERCURRENT_MASK  = (1 << 1),
    MC_FAULT_UNDERVOLTAGE_MASK = (1 << 2),
    MC_FAULT_OVERHEAT_MASK     = (1 << 3),
    MC_FAULT_OVERVOLTAGE_MASK  = (1 << 4),
    MC_FAULT_HALL_MASK         = (1 << 5),
} mc_fault_flags_t;

/**< Union for motor status. */
 typedef union
{
    struct  
    {
        uint8_t state : 7;
        uint8_t direction : 1;
        mc_fault_flags_t flags;
    };
    uint16_t word;    
 } mc_status_t;

 
 /**< Enum for motor states. */
typedef enum
{   
    IDLE,        
    RUNNING,
    FAULT        
} mc_states_t;

/**< Pointer to a user's function with this prototype: void f(mc_status_t), called within main context */
typedef void     (*mc_status_handler_t)(mc_status_t);

/**< Speed as unsigned 16bit integer, angle_units/sample.
 * used with conversion macros: MC_RPM_TO_MCSPEED and MC_MCSPEED_TO_RPM */
typedef uint16_t   mc_speed_t;
typedef uint16_t   mc_position_t;

/**< Enum for HALL sensor scan errors. */
typedef enum
{
    HALL_NO_ERROR = 0,
    HALL_ERROR_TOO_EARLY = 1,
    HALL_ERROR_TOO_LATE = 2,
    HALL_ERROR_DISCONNECTED = 4,
    HALL_ERROR_WRONG_PATTERN = 8,
    HALL_ERROR_MOTOR_STOPPED = 16,
} mc_hall_error_t;


#endif	/* MC_PUBLIC_TYPES_H */
