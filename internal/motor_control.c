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

#include "clkctrl.h"
#include <util/delay.h>
#include <util/atomic.h>
#include "mc_internal_defines.h"
#include "mc_lookup_tables.h"
#include "mc_pwm.h"
#include "mc_sensing.h"
#include "motor_control.h"
#include "mc_ramp.h"
#include "mc_control_analog.h"
#include "mc_control_fault.h"
#include "mc_sm.h"

#if MC_DVRT_ENABLED == true
#include "DVRunTime.h"
#define MC_DVRT_PROCESS()  DVRT_Process()
#else /* MC_DVRT_ENABLED */
#define MC_DVRT_PROCESS()
#endif /* MC_DVRT_ENABLED */

#define MC_DRIVE_EN                     true
#define MC_STALL_EVENT_ENABLED          true

#if MC_CONTROL_MODE == MC_SENSORLESS_MODE
#define MC_SENSING_INTERPOLATION_ON     true
#else  /* MC_CONTROL_MODE */
#define MC_SENSING_INTERPOLATION_ON     false
#endif  /* MC_CONTROL_MODE */

#if (MC_CONTROL_MODE == MC_SENSORLESS_MODE) && (MC_DRIVE_MODE == MC_CONTINUOUS_MODE)
#error "MC_SENSORLESS_MODE and MC_CONTINUOUS_MODE are not compatible, please update mc_config.h"
#endif

#if (MC_DVRT_ENABLED == true) && (MC_PRINTOUT_ENABLED == true)
#error "MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED can't be both 'true' at the same time, please update mc_config.h"
#endif /* (MC_DVRT_ENABLED == true) && (MC_PRINTOUT_ENABLED == true) */

#if MC_SENSING_INTERPOLATION_ON == true
#define SENSING_INTERPOLATION_ANGLE     (30.0)
#else /* MC_SENSING_INTERPOLATION_ON */
#define SENSING_INTERPOLATION_ANGLE      (0.0)
#endif /* MC_SENSING_INTERPOLATION_ON */

#if MC_CONTROL_MODE == MC_SENSORED_MODE
#define MC_SENSOR_GET                   MC_Hall_IntGet
#define MC_SENSOR_SET
#if MC_DRIVE_MODE == MC_STEPPED_MODE
#define MC_ROTOR_OFFSET                 MC_DEG_TO_MCANGLE24(MOTOR_PHASE_ADVANCE - SENSING_INTERPOLATION_ANGLE + 180.0)
#else /* MC_DRIVE_MODE */
#define MC_ROTOR_OFFSET                 MC_DEG_TO_MCANGLE24(MOTOR_PHASE_ADVANCE - SENSING_INTERPOLATION_ANGLE + 120.0)
#endif/* MC_DRIVE_MODE */
#endif  /* MC_CONTROL_MODE == MC_SENSORED_MODE */

#if MC_CONTROL_MODE == MC_SENSORLESS_MODE
#if MC_DRIVE_MODE == MC_STEPPED_MODE
#define MC_SENSOR_GET                   MC_Bemf_IntGet
#define MC_SENSOR_SET                   MC_Bemf_Set
#define MC_ROTOR_OFFSET                 MC_DEG_TO_MCANGLE24(MOTOR_PHASE_ADVANCE - SENSING_INTERPOLATION_ANGLE + 30.0)
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */
#endif  /* MC_CONTROL_MODE == MC_SENSORLESS_MODE */

#if MC_WAVE_PROFILE == MC_WAVE_SINE
#define WAVE_LUT    sine_lookup_table
#elif MC_WAVE_PROFILE == MC_WAVE_SVM
#define WAVE_LUT    svm_lookup_table
#elif MC_WAVE_PROFILE == MC_WAVE_SADDLE
#define WAVE_LUT    saddle_lookup_table
#endif /* MC_WAVE_PROFILE */

static const uint32_t  sensor_data24[] = 
{
    MC_DEG_TO_MCANGLE24(240),
    MC_DEG_TO_MCANGLE24(300),
    MC_DEG_TO_MCANGLE24(0),
    MC_DEG_TO_MCANGLE24(60),
    MC_DEG_TO_MCANGLE24(120),
    MC_DEG_TO_MCANGLE24(180),
    MC_DEG_TO_MCANGLE24(240),
    MC_DEG_TO_MCANGLE24(300),
    MC_DEG_TO_MCANGLE24(0),
    MC_DEG_TO_MCANGLE24(60),
    MC_DEG_TO_MCANGLE24(120),
};

#if MC_DRIVE_MODE == MC_STEPPED_MODE
static const mc_stepped_t sectors_data[16] =
{
    MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C, /* unknown */
    MC_PHASE_FLOAT_A | MC_PHASE_HIGH_B  | MC_PHASE_LOW_C,   /* CW s1 */
    MC_PHASE_HIGH_A  | MC_PHASE_FLOAT_B | MC_PHASE_LOW_C,   /* CW s2 */
    MC_PHASE_HIGH_A  | MC_PHASE_LOW_B   | MC_PHASE_FLOAT_C, /* CW s3 */
    MC_PHASE_FLOAT_A | MC_PHASE_LOW_B   | MC_PHASE_HIGH_C,  /* CW s4 */
    MC_PHASE_LOW_A   | MC_PHASE_FLOAT_B | MC_PHASE_HIGH_C,  /* CW s5 */
    MC_PHASE_LOW_A   | MC_PHASE_HIGH_B  | MC_PHASE_FLOAT_C, /* CW s6 */
    MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C, /* unknown */
    MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C, /* unknown */
    MC_PHASE_FLOAT_A | MC_PHASE_LOW_B   | MC_PHASE_HIGH_C,  /* CCW s1 */
    MC_PHASE_HIGH_A  | MC_PHASE_LOW_B   | MC_PHASE_FLOAT_C, /* CCW s2 */
    MC_PHASE_HIGH_A  | MC_PHASE_FLOAT_B | MC_PHASE_LOW_C,   /* CCW s3 */
    MC_PHASE_FLOAT_A | MC_PHASE_HIGH_B  | MC_PHASE_LOW_C,   /* CCW s4 */
    MC_PHASE_LOW_A   | MC_PHASE_HIGH_B  | MC_PHASE_FLOAT_C, /* CCW s5 */
    MC_PHASE_LOW_A   | MC_PHASE_FLOAT_B | MC_PHASE_HIGH_C,  /* CCW s6 */
    MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C, /* unknown */
};
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */

static mc_status_handler_t          periodic_handler;
static volatile mc_direction_t      drive_direction;
static volatile mc_24_t             speed;
static volatile bool                delay_flag;
static volatile mc_amplitude_t      target_amplitude;
static volatile bool                sync_enabled;
static volatile mc_fault_flags_t    fault_flags; 
static int16_t                      current_offset;

static volatile mc_24_t        phase_a;
#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE
static volatile mc_24_t        phase_b, phase_c;
#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */

#if MC_CONTROL_MODE == MC_SENSORED_MODE
static mc_hall_error_t hall_error_code;
#endif /* MC_CONTROL_MODE == MC_SENSORED_MODE */



static inline void SET_TARGET_AMP(mc_amplitude_t x) {ATOMIC_BLOCK(ATOMIC_RESTORESTATE){target_amplitude = x;}}
static inline void SET_SPEED(mc_speed_t x)          {ATOMIC_BLOCK(ATOMIC_RESTORESTATE){speed.H16 = x; speed.L8 = 0;}}
static inline mc_speed_t GET_SPEED(void)            {mc_speed_t x; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){x = speed.H16;} return x;}
static inline uint32_t GET_SPEED24(void)            {uint32_t   x; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){x = speed.W24;} return x;}

static inline mc_angle_t GET_PHASE(void)            {mc_angle_t x; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){x = phase_a.H16;} return x;}
static inline mc_24_t GET_PHASE24(void)             {mc_24_t x; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){x = phase_a;} return x;}

static void _SoftStart(void);
static void _SoftStop(void);
static void _FaultHandler(mc_fault_event_t);


/* states list */
SMTF_STATES_BEGIN(mc_sm)
SMTF_STATES_DECLARE(MOTOR_IDLE)
SMTF_STATES_DECLARE(MOTOR_RUNNING)
SMTF_STATES_DECLARE(MOTOR_FAULT)
SMTF_STATES_END(mc_sm)

/* events list */
SMTF_EVENTS_BEGIN(mc_sm)
SMTF_EVENTS_DECLARE(START_STOP_EVENT)
SMTF_EVENTS_DECLARE(FAULT_SET_EVENT)
SMTF_EVENTS_DECLARE(FAULT_CLEAR_EVENT)
SMTF_EVENTS_END(mc_sm)

/* transitions table */
SMTF_TRANS_TABLE_BEGIN(mc_sm)        /* START_STOP_EVENT             FAULT_SET_EVENT      FAULT_CLEAR_EVENT                   STATES */
SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(MOTOR_RUNNING, _SoftStart,   MOTOR_FAULT, NULL,   MOTOR_IDLE, NULL)                /* MOTOR_IDLE  */
SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(MOTOR_IDLE,    _SoftStop,    MOTOR_FAULT, NULL,   MOTOR_RUNNING, NULL)             /* MOTOR_RUNNING */
SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(MOTOR_FAULT,   NULL,         MOTOR_FAULT, NULL,   MOTOR_IDLE, MC_Fault_Recovery)   /* MOTOR_FAULT */
SMTF_TRANS_TABLE_END()

SMTF_DEFINE(mc_sm, MOTOR_IDLE)

static void _RampUp(void)
{
    mc_ramp_t spd, amp;
    uint16_t end;
    uint16_t adc_vbus;
    adc_vbus = MC_Analog_Read(AID_VOLTAGE);

    end = MC_RPM_TO_MCSPEED(MC_STARTUP_SPEED);
    MC_Ramp_Init(0, end, MC_RAMP_UP_DURATION, &spd);

    end   = MC_FP_TO_FIPu16(MC_STARTUP_VOLTAGE / MC_ADC_TO_VOLTAGE(adc_vbus));
    MC_Ramp_Init(end/10, end, MC_RAMP_UP_DURATION, &amp);

    do
    {
        MC_PWM_AmplitudeSet(MC_Ramp_Get(&amp));
        SET_SPEED(MC_Ramp_Get(&spd));
        MC_Control_DelayMs(1);
    } while( (amp.is_done == false) || (spd.is_done == false) );
}

static void _RampDown(void)
{
#if MC_RAMP_DOWN_DURATION != 0
    mc_ramp_t spd, amp;
    uint16_t spd_init = GET_SPEED();
    uint16_t amp_init = MC_PWM_AmplitudeGet();
    
    MC_Ramp_Init(spd_init, spd_init / 2, MC_RAMP_DOWN_DURATION / 2, &spd);
    MC_Ramp_Init(amp_init, amp_init / 2, MC_RAMP_DOWN_DURATION / 2, &amp);
    do
    {
        MC_PWM_AmplitudeSet(MC_Ramp_Get(&amp));
        SET_SPEED(MC_Ramp_Get(&spd));
        MC_Control_DelayMs(1);
    } while( (amp.is_done == false) || (spd.is_done == false) );
#endif /* MC_RAMP_DOWN_DURATION != 0 */
}

static void _DirectionSet(mc_direction_t dir)
{
    if(GET_SPEED24() == 0)
    {
        drive_direction = dir;
#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE
        if(dir == MC_DIR_CW)
        {
            phase_a.W24 = MC_DEG_TO_MCANGLE24(   0.0);
            phase_b.W24 = MC_DEG_TO_MCANGLE24( 120.0);
            phase_c.W24 = MC_DEG_TO_MCANGLE24(-120.0);
        }
        else
        {
            phase_a.W24 = MC_DEG_TO_MCANGLE24(-120.0);
            phase_b.W24 = MC_DEG_TO_MCANGLE24( 120.0);
            phase_c.W24 = MC_DEG_TO_MCANGLE24(   0.0);
        }
#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */
#if MC_DRIVE_MODE == MC_STEPPED_MODE
    phase_a.W24 = MC_DEG_TO_MCANGLE24(0.0);
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */
    }
}

static mc_status_t _StatusGet(void)
{
    mc_status_t status = {0};
    status.state = SMTF_STATE_GET(mc_sm);   
    status.direction = drive_direction;    
    status.flags = fault_flags;
    return status;
}

static void _CurrentOffset(void)
{
    current_offset = (int16_t)MC_Analog_Read(AID_CURRENT);
}

#if MC_CONTROL_MODE == MC_SENSORED_MODE
static inline bool _SensorCheck(void)
{
    bool retval = false;
    static mc_sense_t old;
    mc_sense_t sns = MC_Sensing_Get();
    if(old.state != sns.state)
    {
        retval = true;
        old = sns;
    }
    return retval;
}

static inline mc_angle_t _WaitRollover(mc_angle_t init_angle, mc_angle_t *pSensorFound)
{
    mc_angle_t phi1, phi2;
    phi2 = init_angle;
    do
    {
        phi1 = phi2;
        if(_SensorCheck())
            *pSensorFound = phi1;
        phi2 = GET_PHASE();
    } while(phi1 <= phi2);
    return phi2;
}

static inline mc_angle_t _WaitAngle(mc_angle_t target_angle, mc_angle_t *pSensorFound)
{
    mc_angle_t phi1 = 0, phi2;
    phi2 = target_angle;
    do
    {
        if(_SensorCheck())
            *pSensorFound = phi1; 
        phi1 = GET_PHASE();
    } while(phi1 < phi2);
    return phi1;
}

static void _HallScan(void)
{
#define HPROF_STEPS (6*MC_MOTOR_PAIR_POLES)
    mc_hall_prof_t hall_prof[HPROF_STEPS];
    uint8_t* pHtable = MC_Sensing_HallTable_Get();
    uint8_t i, j, k;
    if ((sync_enabled == true) || (speed.W24 == 0))
    {
        hall_error_code = HALL_ERROR_MOTOR_STOPPED;
        return;
    }

    for (i = 0; i < 8; i++)
        pHtable[i] = 0;

    mc_angle_t phi = GET_PHASE();
    mc_angle_t theta, sensor_found = 0;

    for(i = 0; i < MC_MOTOR_PAIR_POLES; i++)
        phi = _WaitRollover(phi, &sensor_found);

    k = 0;
    for(j = 0; j < MC_MOTOR_PAIR_POLES; j++)
    {
        theta = MC_DEG_TO_MCANGLE(0.0);
        phi = _WaitRollover(phi, &sensor_found);

        hall_prof[k].sensor = MC_Sensing_Get();
        hall_prof[k].stator = phi;
        hall_prof[k].rotor  = sensor_found + MC_DEG_TO_MCANGLE(60.0);
        k++;
        for(i = 1; i < 6; i++)
        {
            theta += MC_DEG_TO_MCANGLE(60.0);
            phi = _WaitAngle(theta, &sensor_found);
            hall_prof[k].sensor = MC_Sensing_Get();
            hall_prof[k].stator = phi;
            hall_prof[k].rotor  = sensor_found + MC_DEG_TO_MCANGLE(60.0);
            k++;
        }
    }

    hall_error_code = HALL_NO_ERROR;
    j = 0;
    uint8_t arr[6] = {0};
    for(i = 0; i < HPROF_STEPS; i++)
    {
        if (hall_prof[i].rotor <= hall_prof[i].stator)
            hall_error_code |= HALL_ERROR_TOO_EARLY; /* hall sampling error - too early */
        if((hall_prof[i].rotor - hall_prof[i].stator) > (65536/6))
            hall_error_code |= HALL_ERROR_TOO_LATE; /* hall sampling error - too late */
        uint8_t sns = hall_prof[i].sensor.state;
        if((sns == 7) || (sns == 0))
            hall_error_code |= HALL_ERROR_DISCONNECTED; /* hall invalid combination */
        arr[j++] |= sns;    
        if (j == 6) j = 0;
    }

    uint8_t checker = 0;
    for(i = 0; i < 6; i++)
    {
        checker |= (1<<(arr[i]));
    }
    if(checker != 126)
        hall_error_code |= HALL_ERROR_WRONG_PATTERN;
    for(i = 0; i < 6; i++)
    {
        uint8_t index = arr[i];
        pHtable[index] = i + 1;
    }
    pHtable[0] = 0; pHtable[7] = 0;
#undef HPROF_STEPS
}
#endif /* MC_CONTROL_MODE == MC_SENSORED_MODE */


static void _SoftStart(void)
{
    MC_Control_DelayMs(500);
    _CurrentOffset();
    sync_enabled = false;
#if MC_DRIVE_EN == true
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
    MC_PWM_ForceStart();
    MC_Control_DelayMs(500);
    _RampUp();

#if MC_CONTROL_MODE == MC_SENSORED_MODE
    _HallScan();
#endif /* MC_CONTROL_MODE == MC_SENSORED_MODE */

#endif /* MC_DRIVE_EN */
    sync_enabled = MC_SYNCHRONIZED;
}

static void _SoftStop(void)
{
    sync_enabled = false;
    _RampDown();
    MC_PWM_AmplitudeSet(0);
    MC_PWM_ContinuousScale(0, 0, 0);
    SET_SPEED(0);
    SET_TARGET_AMP(0);
    MC_PWM_ForceStop();
    MC_Control_DelayMs(MC_RAMP_DOWN_DURATION/2);
}

static inline uint8_t _Upper8_Get(uint16_t data)
{
    uint8_t retval;
    split16_t split;

    split.W16 = data;
    retval = split.H8;
    return retval;
}

 static void _Drive(void)
{
    if(speed.W24 != 0)
    {
        phase_a.W24 += speed.W24;
#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE
        phase_b.W24 += speed.W24;
        phase_c.W24 += speed.W24;

        fip_u16_t ch0, ch1, ch2;
        ch0 = WAVE_LUT[phase_a.H8];
        ch1 = WAVE_LUT[phase_b.H8];
        ch2 = WAVE_LUT[phase_c.H8];
        MC_PWM_ContinuousScale(ch0, ch1, ch2);
#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */

#if MC_DRIVE_MODE == MC_STEPPED_MODE
        static uint8_t prev_sector = 0;
        uint8_t sector = sectors_mapping_table[phase_a.H8];
        if(drive_direction == MC_DIR_CCW) sector |= 8;
        static mc_stepped_t drive_data = 0;
        if(sector != prev_sector)
        {
            drive_data = sectors_data[sector];
            MC_PWM_SteppedScale(drive_data);
            prev_sector = sector;
#if MC_CONTROL_MODE == MC_SENSORLESS_MODE
            MC_SENSOR_SET(7 & sector, (uint8_t)drive_data & (MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C));
#endif /* MC_CONTROL_MODE == MC_SENSORLESS_MODE */
        }
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */
    }
    delay_flag = true;
}
  
#if MC_SENSING_INTERPOLATION_ON == false
static mc_24_t _Rotor_Position24_Get(uint8_t index)
{
    static mc_24_t   last_known_position;
    static uint8_t   prev_index;

    if(index == 0)
        return last_known_position;
    if(index != prev_index)
    {
        last_known_position.W24 = sensor_data24[index - 1];
        prev_index = index;
    }
    return last_known_position;
}
#endif /* MC_SENSING_INTERPOLATION_ON == false */

#if MC_SENSING_INTERPOLATION_ON == true
static mc_24_t _Rotor_Position24_Get(uint8_t index)
{
    static mc_24_t   estimated_position, last_known_position;
    static uint8_t   prev_index;

    if((index != 0) && (index != prev_index))
    {
        last_known_position.W24 = sensor_data24[index - 1];
        estimated_position = last_known_position;
        prev_index = index;
    }
    else
    {
        uint8_t diff;
        if(estimated_position.H8 > last_known_position.H8)
            diff = estimated_position.H8 - last_known_position.H8;
        else
            diff = last_known_position.H8 - estimated_position.H8;

        if(diff < 128) /* 180 deg */
            estimated_position.W24 += speed.W24;
    }
    return estimated_position;
}
#endif /* MC_SENSING_INTERPOLATION_ON == true */

static inline void _Rotor_Sense(mc_sense_t sensor_index)
{
    mc_24_t stator = phase_a;
    mc_24_t rotor  = _Rotor_Position24_Get(sensor_index.id);

    if(sync_enabled == true) do
    {
        #if MC_CONTROL_MODE == MC_SENSORED_MODE
        if(sensor_index.id == 0)
        {
            MC_Control_FaultForceOff();
            _FaultHandler(MC_FAULT_HALL_ERROR_EVENT);
            break;
        }
        #endif /* MC_CONTROL_MODE == MC_SENSORED_MODE */

        mc_24_t idiff, speed_adj;
        uint32_t x;
        int8_t   y;
        mc_amplitude_t actual_amp;

        x = rotor.W24 + MC_ROTOR_OFFSET;
        rotor.W24 = x;
        x = rotor.W24 - stator.W24;
        idiff.W24 = x;
        y = (int8_t)(idiff.H8);
        x = speed.W24;
        speed_adj.W24 = x + y;
        if (speed_adj.W24 < 128)
        #if MC_STALL_EVENT_ENABLED == true
        {
            MC_Control_FaultForceOff();
            _FaultHandler(MC_FAULT_STALL_DETECTION_EVENT);
            break;
        }
        else
        {
            speed.W24 = speed_adj.W24;
        }
        #else /* MC_STALL_EVENT_ENABLED */
        {
            speed_adj.W24 = 128;
        }
        speed.W24 = speed_adj.W24;
        #endif /* MC_STALL_EVENT_ENABLED */

        actual_amp = MC_PWM_AmplitudeGet();
        if(actual_amp != target_amplitude)
        {
            if(actual_amp > target_amplitude) actual_amp--;
            else                              actual_amp++;
            MC_PWM_AmplitudeSet(actual_amp);
        }
    } while(0);
}

static void _Motor_Handler(void)
{
    mc_sense_t sensor_index = MC_SENSOR_GET();
#if HIGH_FREQUENCY == true
    static bool state = false;
    if(state)
    {
        state = false;
#endif /* HIGH_FREQUENCY */
        MC_Analog_Run();
	    _Drive();    
#if HIGH_FREQUENCY == true
    }
    else
    {
        state = true;
#endif /* HIGH_FREQUENCY */
        _Rotor_Sense(sensor_index);
#if HIGH_FREQUENCY == true
    }
#endif /* HIGH_FREQUENCY */
}

#if MC_SPEED_REGULATOR_EN == true
static mc_speed_t     target_speed;
/* called every 1 ms */
static void _ControlLoop(void)
{
    static mc_amplitude_t amp = 0;
    if(GET_SPEED24() == 0)
    {
        amp = 0;
    }
    else
    {
        int16_t err = target_speed - GET_SPEED();
        amp = amp + (err>>2);
        if(err > 0) {amp++; if(amp > MC_FIPu16_MAX) amp = MC_FIPu16_MAX;}
        else        {amp--; if(amp > MC_FIPu16_MAX) amp = 1;}
    }

    SET_TARGET_AMP(amp);
}
#endif /* MC_SPEED_REGULATOR_EN == true */

static void _DelayWait(void)
{
    while(delay_flag == false);
    delay_flag = false;
}

/* called from interrupt context */
static void _FaultHandler(mc_fault_event_t ev)
{   
    switch(ev)
    {       
        case MC_FAULT_HIGH_TEMPERATURE_EVENT:   fault_flags |=  MC_FAULT_OVERHEAT_MASK;     break;
        case MC_FAULT_HIGH_TEMPERATURE_RESTORE: fault_flags &= ~MC_FAULT_OVERHEAT_MASK;     break;      
        case MC_FAULT_OVER_VOLTAGE_EVENT:       fault_flags |=  MC_FAULT_OVERVOLTAGE_MASK;  break;
        case MC_FAULT_OVER_VOLTAGE_RESTORE:     fault_flags &= ~MC_FAULT_OVERVOLTAGE_MASK;  break;      
        case MC_FAULT_UNDER_VOLTAGE_EVENT:      fault_flags |=  MC_FAULT_UNDERVOLTAGE_MASK; break;
        case MC_FAULT_UNDER_VOLTAGE_RESTORE:    fault_flags &= ~MC_FAULT_UNDERVOLTAGE_MASK; break;
        case MC_FAULT_OVER_CURRENT_EVENT:       fault_flags |=  MC_FAULT_OVERCURRENT_MASK;  break;
        case MC_FAULT_STALL_DETECTION_EVENT:    fault_flags |=  MC_FAULT_STALL_MASK;        break;
        case MC_FAULT_HALL_ERROR_EVENT:         fault_flags |=  MC_FAULT_HALL_MASK;         break;
        default: break;
    }
    if(fault_flags) SMTF_HANDLER_CALL(mc_sm, FAULT_SET_EVENT);
    else            SMTF_HANDLER_CALL(mc_sm, FAULT_CLEAR_EVENT);
}

/*** Functions called from other modules, but not main/user callable */

void MC_Control_FaultForceOff(void)
{
    sync_enabled = false;
    MC_PWM_ForceStop();
    MC_PWM_AmplitudeSet(0);
    target_amplitude = 0;
    speed.W24 = 0;
}


void MC_Control_FaultNotify(mc_fault_event_t fault_event)
{
    _FaultHandler(fault_event);
}

/******************************/
/*  PUBLIC FUNCTIONS          */
/******************************/

void MC_Control_Initialize(void)
{
    sync_enabled = false;
    speed.W24 = 0;
    target_amplitude = 0;
    MC_Analog_Initialize();
    MC_Sensing_Initialize();
    MC_PWM_Initialize();
    MC_PWM_AmplitudeSet(0);
    MC_PWM_HandlerRegister(_Motor_Handler);
    MC_Fault_Initialize();
    MC_Control_DelayMs(500);
    _CurrentOffset();
    MC_PWM_AmplitudeSet(0);
    MC_PWM_ContinuousScale(0, 0, 0);
    SET_SPEED(0);
    MC_PWM_ForceStop();
}

void MC_Control_DelayMs(uint32_t delay_ms)
{
    uint16_t counter;

    while(delay_ms--)
    {
        counter = (uint16_t)((float)MC_F_SAMPLING / 1000.0);
        while(counter--)
            _DelayWait();

        MC_DVRT_PROCESS();
        if(periodic_handler != NULL) periodic_handler(_StatusGet());
#if (MC_SPEED_REGULATOR_EN==true)
        _ControlLoop();
#endif  /* MC_SPEED_REGULATOR_EN==true */
    }
}

void MC_Control_PeriodicHandlerRegister(mc_status_handler_t handler)
{
    periodic_handler = handler;
}

void MC_Control_ReferenceSet(uint16_t ref)
{
#if (MC_SPEED_REGULATOR_EN == false)
    mc_amplitude_t amp;
    amp = (mc_amplitude_t)SCALE_ZERO(ref, MC_MAX_SCALE_UNSIGNED, MC_MAX_SCALE_SIGNED);
    if(GET_SPEED24() != 0)
        SET_TARGET_AMP(amp);
#endif /* MC_SPEED_REGULATOR_EN == false */
    
#if (MC_SPEED_REGULATOR_EN == true)
    target_speed = SCALE_FULL(ref, 0UL, (uint32_t)MC_MAX_SCALE_UNSIGNED, (uint32_t)MC_RPM_TO_MCSPEED(MC_SPEED_REGULATOR_MIN), (uint32_t)MC_RPM_TO_MCSPEED(MC_SPEED_REGULATOR_MAX));
#endif /* MC_SPEED_REGULATOR_EN == true */
}

uint8_t MC_Control_PotentiometerRead(void)
{
    return (uint8_t)MC_ADC_TO_PERCENT(MC_Analog_Read(AID_POTENTIOMETER));
}

uint16_t MC_Control_FastPotentiometerRead(void)
{
    return MC_Analog_Read(AID_POTENTIOMETER);
}

uint16_t MC_Control_VoltageBusRead(void)
{
    return (uint16_t)(1000.0 * MC_ADC_TO_VOLTAGE(MC_Analog_Read(AID_VOLTAGE)));
}

uint8_t MC_Control_TemperatureRead(void)
{
    return (uint8_t)MC_ADC_TO_CELSIUS(MC_Analog_Read(AID_TEMPERATURE));
}

int16_t MC_Control_CurrentRead(void)
{
    int32_t current;
    current = (int16_t)MC_Analog_Read(AID_CURRENT);
    current -= current_offset;
    return (int16_t)(1000.0 * MC_ADC_TO_CURRENT(current));
}

mc_speed_t MC_Control_SpeedGet(void)
{
    return GET_SPEED();
}

uint32_t MC_Control_Speed32Get(void)
{
    return GET_SPEED24();
}

mc_status_t MC_Control_StatusGet(void)
{   
    mc_status_t retVal = _StatusGet();
    if((retVal.flags & MC_FAULT_STALL_MASK) | (retVal.flags & MC_FAULT_OVERCURRENT_MASK) | (retVal.flags & MC_FAULT_HALL_MASK))
    {
        fault_flags &= ~MC_FAULT_STALL_MASK;
        fault_flags &= ~MC_FAULT_HALL_MASK;
        fault_flags &= ~MC_FAULT_OVERCURRENT_MASK;
        SMTF_HANDLER_CALL(mc_sm, FAULT_CLEAR_EVENT);
    }
    return retVal;
}

void MC_Control_StartStop(mc_direction_t dir)
{ 
    _DirectionSet(dir);    
    SMTF_HANDLER_CALL(mc_sm, START_STOP_EVENT);   
}

#if MC_CONTROL_MODE == MC_SENSORED_MODE
mc_hall_error_t MC_Control_HallError_Get(void)
{
    return hall_error_code;
}
#endif /* MC_CONTROL_MODE == MC_SENSORED_MODE */
