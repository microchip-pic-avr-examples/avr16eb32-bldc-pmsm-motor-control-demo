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

#if (MC_CONTROL_MODE == MC_SENSORLESS_MODE) && (MC_DRIVE_MODE == MC_CONTINUOUS_MODE)
#error "MC_SENSORLESS_MODE and MC_CONTINUOUS_MODE are not compatible, please update mc_config.h"
#endif

#if (MC_DVRT_ENABLED == true) && (MC_PRINTOUT_ENABLED == true)
#error "MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED can't be both 'true' at the same time, please update mc_config.h"
#endif /* (MC_DVRT_ENABLED == true) && (MC_PRINTOUT_ENABLED == true) */



#if MC_CONTROL_MODE == MC_SENSORED_MODE
#define MC_SENSOR_GET                   MC_Hall_IntGet
#define MC_SENSOR_SET
#define MC_ROTOR_OFFSET_CW              MC_DEG_TO_MCANGLE(-30.0 - MOTOR_HALL_DEVIATION)
#define MC_ROTOR_OFFSET_CCW             MC_DEG_TO_MCANGLE(-30.0 + MOTOR_HALL_DEVIATION)
#define MC_POSITION_ESTIMATION_ON       false
#endif  /* MC_CONTROL_MODE == MC_SENSORED_MODE */

#if MC_CONTROL_MODE == MC_SENSORLESS_MODE
#define MC_SENSOR_GET                   MC_Bemf_IntGet
#define MC_SENSOR_SET                   MC_Bemf_Set
#define MC_ROTOR_OFFSET_CW              MC_DEG_TO_MCANGLE(0.0)
#define MC_ROTOR_OFFSET_CCW             MC_DEG_TO_MCANGLE(0.0)
#define MC_POSITION_ESTIMATION_ON       true
#endif  /* MC_CONTROL_MODE == MC_SENSORLESS_MODE */

#if MC_SCALE_MODE == MC_SCALE_BOTTOM
#define WAVE_LUT  saddle_lookup_table
#endif /* MC_SCALE_MODE == MC_SCALE_BOTTOM */

#if MC_SCALE_MODE == MC_SCALE_CENTER
#define WAVE_LUT  svm_lookup_table
#endif /* MC_SCALE_MODE == MC_SCALE_CENTER */

#define MC_TIMEOUT_ANGLE                (120.0) // degrees
#define MC_MAX_SPEED                    MC_DEG_TO_MCANGLE(30.0) // degrees / sample
#define MC_MAX_AMPLITUDE                MC_FP_TO_FIPu16(1.0)

static const mc_angle_t  sensor_data[] = 
{
    MC_DEG_TO_MCANGLE(240),
    MC_DEG_TO_MCANGLE(300),
    MC_DEG_TO_MCANGLE(0),
    MC_DEG_TO_MCANGLE(60),
    MC_DEG_TO_MCANGLE(120),
    MC_DEG_TO_MCANGLE(180),
    MC_DEG_TO_MCANGLE(240),
    MC_DEG_TO_MCANGLE(300),
    MC_DEG_TO_MCANGLE(0),
    MC_DEG_TO_MCANGLE(60),
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
static volatile mc_speed_t          speed;
static volatile bool                delay_flag;
static volatile mc_amplitude_t      target_amplitude;
static volatile bool                sync_enabled, sync_status;
static volatile fault_flags_t       fault_flags; 
static int16_t                      current_offset;

static volatile mc_angle_t     phase_a;
#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE
static volatile mc_angle_t     phase_b, phase_c;
#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */
static volatile mc_angle_t     bemf_a;

static inline void SET_TARGET_AMP(mc_amplitude_t x) {ATOMIC_BLOCK(ATOMIC_RESTORESTATE){target_amplitude = x;}}
static inline void SET_SPEED(mc_speed_t x)          {ATOMIC_BLOCK(ATOMIC_RESTORESTATE){speed            = x;}}
static inline mc_speed_t GET_SPEED(void)            {mc_speed_t x; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){x = speed;} return x;}
static inline mc_angle_t GET_PHASE(void)            {mc_angle_t x; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){x = phase_a;} return x;}

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
SMTF_TRANS_TABLE_BEGIN(mc_sm)        /* START_STOP_EVENT            FAULT_SET_EVENT         FAULT_CLEAR_EVENT                   STATES */
SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(MOTOR_RUNNING, _SoftStart,  MOTOR_FAULT, NULL,      MOTOR_IDLE, NULL)                /* MOTOR_IDLE  */
SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(MOTOR_IDLE,  _SoftStop,     MOTOR_FAULT, NULL,      MOTOR_RUNNING, NULL)             /* MOTOR_RUNNING */
SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(MOTOR_FAULT, NULL,          MOTOR_FAULT, NULL,      MOTOR_IDLE, MC_Fault_Recovery)   /* MOTOR_FAULT */

SMTF_TRANS_TABLE_END()

SMTF_DEFINE(mc_sm, MOTOR_IDLE)


static void _RampUp(void)
{
    mc_ramp_t spd, amp;
    uint16_t end;
    uint16_t vbus = MC_Analog_Read(AID_VOLTAGE);

    end = MC_RPM_TO_MCSPEED(MC_MIN_SPEED);
    MC_Ramp_Init(0, end, MC_RAMP_UP_DURATION, &spd);
    end   = MC_FP_TO_FIPu16(2.0 * MC_STARTUP_VOLTAGE / MC_VBUS_TO_VOLTAGE(vbus));
    MC_Ramp_Init(0, end, MC_RAMP_UP_DURATION, &amp);
    do
    {
        MC_PWM_AmplitudeSet(MC_Ramp_Get(&amp));
        SET_SPEED(MC_Ramp_Get(&spd));
        MC_Control_DelayMs(1);
    } while( (amp.is_done == false) || (spd.is_done == false) );
}

static void _RampDown(void)
{
    mc_ramp_t spd, amp;
    uint16_t spd_init = GET_SPEED();
    uint16_t amp_init = MC_PWM_AmplitudeGet();
    
    MC_Ramp_Init(spd_init, 3UL * spd_init / 4, MC_RAMP_DOWN_DURATION / 2, &spd);
    MC_Ramp_Init(amp_init, 3UL * amp_init / 4, MC_RAMP_DOWN_DURATION / 2, &amp);
    do
    {
        MC_PWM_AmplitudeSet(MC_Ramp_Get(&amp));
        SET_SPEED(MC_Ramp_Get(&spd));
        MC_Control_DelayMs(1);
    } while( (amp.is_done == false) || (spd.is_done == false) );
}

static void _DirectionSet(mc_direction_t dir)
{
    if(speed == 0)
    {
        drive_direction = dir;
#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE
        if(dir == MC_DIR_CW)
        {
            phase_a = MC_DEG_TO_MCANGLE(0.0);
            phase_b = MC_DEG_TO_MCANGLE(120.0);
            phase_c = MC_DEG_TO_MCANGLE(240.0);
        }
        else
        {
            phase_a = MC_DEG_TO_MCANGLE(240.0);
            phase_b = MC_DEG_TO_MCANGLE(120.0);
            phase_c = MC_DEG_TO_MCANGLE(0.0);
        }
#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */
#if MC_DRIVE_MODE == MC_STEPPED_MODE
    phase_a = MC_DEG_TO_MCANGLE(0.0);
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

static void _SoftStart(void)
{
    MC_Control_DelayMs(100);
    _CurrentOffset();
    sync_enabled = false;
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
    MC_PWM_ForceStart();
    _RampUp();
    sync_enabled = MC_SYNCHRONIZED;
}

static void _SoftStop(void)
{
    sync_enabled = false;
    _RampDown();
    MC_PWM_ForceStop();
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
    MC_Control_DelayMs(MC_RAMP_DOWN_DURATION/2);
}

static inline uint8_t _Upper8_Get(uint16_t data)
{
    uint8_t retval;
    split16_t split;

    split.W = data;
    retval = split.H;
    return retval;
}

static void _Drive(void)
{
    mc_amplitude_t actual_amp;
    phase_a += speed;
#if MC_DRIVE_MODE == MC_CONTINUOUS_MODE
    phase_b += speed;
    phase_c += speed;
    fip_u16_t ch0, ch1, ch2;
    ch0 = WAVE_LUT[_Upper8_Get(phase_a)];
    ch1 = WAVE_LUT[_Upper8_Get(phase_b)];
    ch2 = WAVE_LUT[_Upper8_Get(phase_c)];
    MC_PWM_ContinuousScale(ch0, ch1, ch2);
#endif /* MC_DRIVE_MODE == MC_CONTINUOUS_MODE */

#if MC_DRIVE_MODE == MC_STEPPED_MODE
    static uint8_t prev_sector = 0;
    uint8_t sector = sectors_mapping_table[_Upper8_Get(phase_a)];
    if(drive_direction == MC_DIR_CCW) sector |= 8;
    if(sector != prev_sector)
    {
        mc_stepped_t drive_data = sectors_data[sector];
        MC_PWM_SteppedScale(drive_data);
        prev_sector = sector;
#if MC_CONTROL_MODE == MC_SENSORLESS_MODE
        uint8_t mixed_data = (sector << 4) | (uint8_t)(drive_data & (MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C));
        MC_SENSOR_SET(mixed_data);
#endif /* MC_CONTROL_MODE == MC_SENSORLESS_MODE */
    }
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */
    if(sync_status == true)
    {
        actual_amp = MC_PWM_AmplitudeGet();
        if(actual_amp != target_amplitude)
        {
            if(actual_amp > target_amplitude) actual_amp--;
            else                              actual_amp++;
            MC_PWM_AmplitudeSet(actual_amp);
        }
    }
    delay_flag = true;
}

static void _Stall_Handler(bool stall_event)
{
    static uint16_t stall_counter = 0;
    if(stall_event)
    {
        if(stall_counter < 65535)  stall_counter ++;
    }
    else
    {
        if(stall_counter > 0)      stall_counter--;
    }
    
    if(_Upper8_Get(stall_counter) > MC_STALL_EVENTS_THRESHOLD)
    {
        stall_counter = 0;
        sync_enabled = false;
        MC_PWM_ForceStop();
        MC_PWM_AmplitudeSet(0);
        SET_SPEED(0);
        _FaultHandler(MC_FAULT_STALL_DETECTION_EVENT);
    }
}

static mc_angle_t _Rotor_Position_Get(uint8_t sensor)
{
    static mc_angle_t   estimated_position, last_known_position;
    static uint8_t      prev_sensor;

    if(sensor != prev_sensor)
    {
        last_known_position = sensor_data[sensor];
        estimated_position = last_known_position;
        prev_sensor = sensor;
    }
#if MC_POSITION_ESTIMATION_ON == true
    else
    {
        uint8_t temp = _Upper8_Get(estimated_position - last_known_position);
        if(temp < _Upper8_Get(MC_DEG_TO_MCANGLE(MC_TIMEOUT_ANGLE)))
            estimated_position += speed;
        else if(sync_enabled == true)
            _Stall_Handler(true);
    }
#endif /* MC_POSITION_ESTIMATION_ON == true */
    return estimated_position;
}

/* returns 'true' if abs(val) > threshold */
static inline bool _AbsCompare(int16_t val, int16_t threshold)
{
    if (val < 0) val = 0 - val;
    if(val > threshold) return true;
    else                return false;
}

static void _Motor_Handler(void)
{
    mc_sense_t sensor = MC_SENSOR_GET();
    MC_Analog_Run();
    _Drive();
    mc_angle_t stator = phase_a;
    
    static split32_t high_precision_speed;
    uint8_t id = sensor.id;
    int16_t idiff;
  
    mc_angle_t rotor;
    rotor = _Rotor_Position_Get(id);
    mc_angle_t rotor_offset = (drive_direction == MC_DIR_CW) ? (MC_ROTOR_OFFSET_CW) : (MC_ROTOR_OFFSET_CCW);
    rotor += rotor_offset;
    idiff = rotor - stator;
    idiff += MC_DEG_TO_MCANGLE(MOTOR_PHASE_ADVANCE);
        
    if(sync_enabled == true)
    {
        high_precision_speed.W += ((int32_t)idiff);
        if(high_precision_speed.H < MC_RPM_TO_MCSPEED(MC_MIN_SPEED)) { high_precision_speed.H = MC_RPM_TO_MCSPEED(MC_MIN_SPEED); high_precision_speed.L = 0;}
        if(high_precision_speed.H >                   MC_MAX_SPEED)  { high_precision_speed.H =                   MC_MAX_SPEED;  high_precision_speed.L = 0;}
        speed = high_precision_speed.H;
        bool bTemp = _AbsCompare(idiff, MC_DEG_TO_MCANGLE(MC_STALL_ERROR_TOLERANCE));
        _Stall_Handler(bTemp);
        sync_status = ! bTemp;
    }
    else
    {
        sync_status = false;
        high_precision_speed.H = speed;
        high_precision_speed.L = 0;
    }
}

#if MC_SPEED_REGULATOR_EN == true
static mc_speed_t     target_speed;
/* called every 1 ms */
static void _ControlLoop(void)
{
    static mc_amplitude_t amp = 0;
    int16_t err = target_speed - GET_SPEED();
    amp = amp + (err>>3);
    if(err > 0) {amp++; if(amp > MC_MAX_AMPLITUDE) amp = MC_MAX_AMPLITUDE;}
    else        {amp--; if(amp > MC_MAX_AMPLITUDE) amp = 1;}

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
    SET_SPEED(0);
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
    speed = 0;
    target_amplitude = 0;
    MC_Analog_Initialize();
    MC_Sensing_Initialize();
    MC_PWM_Initialize();
    MC_PWM_AmplitudeSet(0);
    MC_PWM_ContinuousScale(0, 0, 0);
    MC_PWM_SenseHandlerRegister(_Motor_Handler);
    MC_Fault_Initialize();
    MC_Control_DelayMs(500);
    _CurrentOffset();
    MC_PWM_ForceStop();
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
}

void MC_Control_DelayMs(uint32_t delay_ms)
{
    uint16_t counter;

    while(delay_ms--)
    {
        counter = (uint16_t)(1000.0 / (float)PWM_PERIOD);
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
    ref >>= 1;
    amp = (mc_amplitude_t)ref;
    SET_TARGET_AMP(amp);
#endif /* MC_SPEED_REGULATOR_EN == false */
    
#if (MC_SPEED_REGULATOR_EN == true)
    target_speed = (mc_speed_t)(ref>>6);
    if(target_speed < MC_RPM_TO_MCSPEED(MC_MIN_SPEED))
        target_speed = MC_RPM_TO_MCSPEED(MC_MIN_SPEED);
#endif /* MC_SPEED_REGULATOR_EN == true */
}

uint8_t MC_Control_PotentiometerRead(void)
{
    return (uint8_t)MC_POT_TO_PERCENT(MC_Analog_Read(AID_POTENTIOMETER));
}

uint16_t MC_Control_FastPotentiometerRead(void)
{
    return MC_Analog_Read(AID_POTENTIOMETER);
}

uint16_t MC_Control_VoltageBusRead(void)
{
    return (uint16_t)(1000.0 * MC_VBUS_TO_VOLTAGE(MC_Analog_Read(AID_VOLTAGE)));
}

uint8_t MC_Control_TemperatureRead(void)
{
    return (uint8_t)MC_TEMP_TO_CELSIUS(MC_Analog_Read(AID_TEMPERATURE));
}

int16_t MC_Control_CurrentRead(void)
{
    int32_t current;
    current = (int16_t)MC_Analog_Read(AID_CURRENT);
    current -= current_offset;
    return (1000.0 * MC_CURR_TO_AMPS(current));
}

mc_speed_t MC_Control_SpeedGet(void)
{
    return GET_SPEED();
}

mc_status_t MC_Control_StatusGet(void)
{   
    mc_status_t retVal = _StatusGet();
    if((retVal.flags & MC_FAULT_STALL_MASK) | (retVal.flags & MC_FAULT_OVERCURRENT_MASK))
    {
        fault_flags &= ~MC_FAULT_STALL_MASK;
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

