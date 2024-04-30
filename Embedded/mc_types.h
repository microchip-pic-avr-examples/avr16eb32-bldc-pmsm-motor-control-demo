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

#ifndef MC_TYPES_H
#define	MC_TYPES_H

#include <stdbool.h>
#include <stdint.h>


/* fixed point formats, 16/8bit, signed/unsigned */
typedef int8_t     fip_i8_t;    /*  Q1.7  */
typedef uint8_t    fip_u8_t;    /* UQ1.7  */
typedef int16_t    fip_i16_t;   /*  Q1.15 */
typedef uint16_t   fip_u16_t;   /* UQ1.15 */ 

typedef uint16_t   mc_int_dcy_t;        /* duty cycle as integer, where max value is the period   */
typedef fip_u16_t  mc_fip_dcy_t;        /* duty cycle in fixed point format, in range 0.0 ... 1.0 */
typedef fip_u16_t  mc_amplitude_t;      /* amplitude in fixed point format, in range 0.0 ... 1.0  */
typedef uint16_t   mc_speed_t;          /* speed as unsigned 16bit integer, angle_units/sample      */ 
typedef uint16_t   mc_angle_t;          /* angle as unsigned 16bit integer, 1 angle_unit = 360/65536 degrees, range meaning: 0 ... 359.9945 degrees */


typedef union
{
    struct
    {
        uint8_t L;
        uint8_t H;
    };
    uint16_t W;
} split16_t;

typedef union
{
    struct
    {
        uint16_t L;
        uint16_t H;
    };
    uint32_t W;
} split32_t;


typedef enum
{
    MC_DIR_CW,
    MC_DIR_CCW
} mc_direction_t;

typedef enum
{
    MC_PHASE_FLOAT_A = 0x01,
    MC_PHASE_FLOAT_B = 0x02,
    MC_PHASE_FLOAT_C = 0x04,
    MC_PHASE_HIGH_A = 0x08,
    MC_PHASE_HIGH_B = 0x10,
    MC_PHASE_HIGH_C = 0x20,
    MC_PHASE_LOW_A = 0x40,
    MC_PHASE_LOW_B = 0x80,
    MC_PHASE_LOW_C = 0x100
} mc_stepped_t;

typedef enum
{    
    VOLTAGE,
    TEMPERATURE,
    POTENTIOMETER,
    ANALOG_ID_MAX
} mc_analog_id_t;


typedef union
{
    struct
    {
        uint8_t   L;
        uint16_t  H;
    };
    uint32_t  W;
} mc_analog_data_t;


typedef enum
{
    MC_FAULT_NO_EVENT,
    MC_FAULT_STALL_DETECTION_EVENT,
    MC_FAULT_NOT_ENOUGH_VOLTAGE_EVENT,
} mc_fault_event_t;


typedef struct
{
    mc_angle_t    stator;
    mc_angle_t    rotor;
    uint8_t       sensor;
} mc_hall_prof_t;


typedef void     (*mc_handler_t)(void); /* pointer to a function with this prototype: void f(void); */
typedef void     (*mc_fault_handler_t)(mc_fault_event_t);  /* pointer to a function that receives notification information */



/* floating point range is 0.0 ... 1.0 when unsigned , UQ1.15 and UQ1.7  */
/* floating point range is -1.0 ... 1.0 when signed     Q1.15 and Q1.7 */

/* Conversion Macros from floating point into fixed point  */ 
#define MC_FP_TO_FIPi8(X)   (fip_i8_t)(64.0*(X) + 0.5)
#define MC_FP_TO_FIPu8(X)   (fip_u8_t)(128.0*(X) + 0.5)
#define MC_FP_TO_FIPi16(X)  (fip_i16_t)(16384.0*(X) + 0.5)
#define MC_FP_TO_FIPu16(X)  (fip_u16_t)(32768.0*(X) + 0.5)

/*  Conversion Macros from fixed point into floating point  */
#define MC_FIPi8_TO_FP(X)   ((float)(X)/64.0)
#define MC_FIPu8_TO_FP(X)   ((float)(X)/128.0)
#define MC_FIPi16_TO_FP(X)  ((float)(X)/16384.0)
#define MC_FIPu16_TO_FP(X)  ((float)(X)/32768.0)

/*  Interval limits for all FIP (fixed point) types  */
#define MC_FIPi8_MAX         MC_FP_TO_FIPi8(1.0)
#define MC_FIPi8_MIN         MC_FP_TO_FIPi8(-1.0)
#define MC_FIPu8_MAX         MC_FP_TO_FIPu8(1.0)
#define MC_FIPu8_MIN         MC_FP_TO_FIPu8(0.0)
#define MC_FIPi16_MAX        MC_FP_TO_FIPi16(1.0)
#define MC_FIPi16_MIN        MC_FP_TO_FIPi16(-1.0)
#define MC_FIPu16_MAX        MC_FP_TO_FIPu16(1.0)        
#define MC_FIPu16_MIN        MC_FP_TO_FIPu16(0.0)

/* Scaling Macros */
#define MC_SCALE_FIPi8(S, X)        (fip_i8_t)((S)*(X))
#define MC_SCALE_FIPu8(S, X)        (fip_u8_t)((S)*(X))
#define MC_SCALE_FIPi16(S, X)       (fip_i16_t)((S)*(X))
#define MC_SCALE_FIPu16(S, X)       (fip_u16_t)((S)*(X))


#endif	/* MC_TYPES_H */
