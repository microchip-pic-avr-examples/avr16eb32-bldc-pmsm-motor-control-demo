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
#include <stddef.h>
#include <util/atomic.h>
#include "mc_config.h"
#include "mc_lookup_tables.h"
#include "mc_pwm.h"
#include "mc_sensing.h"
#include "motor_control.h"
#include "mc_control_analog.h"


#if MC_DVRT_ENABLED == true
#include "DVRunTime.h"
#define MC_DVRT_PROCESS()  DVRT_Process()
#else /* MC_DVRT_ENABLED */
#define MC_DVRT_PROCESS()
#endif /* MC_DVRT_ENABLED */


#define SET_TARGET_AMP(X)    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){target_amplitude = (X);}
#define SET_SPEED(X)         ATOMIC_BLOCK(ATOMIC_RESTORESTATE){speed            = (X);}
#define GET_SPEED()          ({mc_speed_t _q; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){_q = speed;}_q;})
#define GET_PHASE()          ({mc_angle_t _q; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){_q = phase_a;}_q;})


#if MC_CONTROL_MODE == MC_SENSORED_MODE
#define MC_SENSOR_GET                   MC_Hall_Get
#define MC_SENSOR_SET
#define MC_ROTOR_OFFSET_CW              MC_DEG_TO_MCANGLE(30.0 + MOTOR_HALL_DEVIATION_CW)
#define MC_ROTOR_OFFSET_CCW             MC_DEG_TO_MCANGLE(30.0 + MOTOR_HALL_DEVIATION_CCW)
#endif  /* MC_CONTROL_MODE == MC_SENSORED_MODE */

#if MC_CONTROL_MODE == MC_SENSORLESS_MODE
#define MC_SENSOR_GET                   MC_Bemf_Get
#define MC_SENSOR_SET                   MC_Bemf_Set
#define MC_ROTOR_OFFSET_CW              MC_DEG_TO_MCANGLE(0.0)
#define MC_ROTOR_OFFSET_CCW             MC_DEG_TO_MCANGLE(0.0)
#endif  /* MC_CONTROL_MODE == MC_SENSORLESS_MODE */


#define MC_TIMEOUT_ANGLE                (120.0) // degrees
#define MC_MAX_SPEED                    MC_DEG_TO_MCANGLE(30.0) // degrees / sample
#define MC_MAX_AMPLITUDE                MC_FP_TO_FIPu16(1.0)

static const mc_angle_t  sensor_data[9] = 
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
};


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

static mc_handler_t            periodic_handler;
static mc_fault_handler_t      fault_notify_handler;
static volatile mc_direction_t drive_direction;
static volatile mc_speed_t     speed;
static volatile bool           delay_flag;
static volatile mc_amplitude_t target_amplitude;
static volatile bool           sense_sync;

#if MC_DRIVE_MODE == MC_STEPPED_MODE
static volatile mc_angle_t     phase_a;
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */
static volatile mc_angle_t     bemf_a;

static void _DirectionSet(mc_direction_t dir)
{
    if((sense_sync == false) && (speed == 0))
    {
        drive_direction = dir;
#if MC_DRIVE_MODE == MC_STEPPED_MODE
    phase_a = MC_DEG_TO_MCANGLE(0.0);
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */
    }
}

static inline uint8_t _Upper8_PGet(volatile uint16_t *pData)
{
    uint8_t retval;
    split16_t *pSplit;

    pSplit = (split16_t *)pData;
    retval = pSplit -> H;
    return retval;
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
#if MC_DRIVE_MODE == MC_STEPPED_MODE
    phase_a += speed;
#endif /* MC_DRIVE_MODE == MC_STEPPED_MODE */

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
    if(sense_sync == true)
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

static inline void _StatorRolloverWait(void)
{
    uint8_t prev, actual = 0;
    do
    {
        prev = actual;
        actual = _Upper8_PGet(&phase_a);
        _delay_us(10);
    } while(actual >= prev);
}

/*  return values: true = fail, false = OK  */
static bool _RampUp(void)
{
    uint16_t vbus = MC_VBUS_NGET(MC_Analog_Read(VOLTAGE));
    if(vbus < MC_VOLTAGE_TO_VBUS(MC_STARTUP_VOLTAGE * 2.0))
        return true;
    else
    {
        uint32_t sp = 0;
        uint32_t amp = 0;
        uint32_t sp_target = MC_RPM_TO_MCSPEED(MC_MIN_SPEED);
        uint32_t start_amp = MC_FP_TO_FIPu16(MC_STARTUP_VOLTAGE / MC_VBUS_TO_VOLTAGE(vbus));
        uint32_t amp_target = start_amp;
        sp_target <<= 8;
        amp_target <<= 8;
        uint32_t sp_step = (uint32_t)((float)sp_target / (float)MC_RAMP_UP_DURATION + 1);
        uint32_t amp_step = (uint32_t)((float)amp_target / (float)MC_RAMP_UP_DURATION + 1);
        while((sp < sp_target) && (amp < amp_target))
        {
            MC_PWM_AmplitudeSet(amp >> 8);
            SET_SPEED(sp >> 8);
            amp+= amp_step;
            sp += sp_step;
            MC_Control_DelayMs(1);
        }
        MC_PWM_AmplitudeSet(start_amp);
        SET_SPEED(MC_RPM_TO_MCSPEED(MC_MIN_SPEED));
        return false;
    }
}

/*static*/ void _RampDown(void)
{
    uint32_t sp = GET_SPEED();
    uint32_t amp = MC_PWM_AmplitudeGet();
    uint32_t sp_target = MC_RPM_TO_MCSPEED(MC_MIN_SPEED);
    uint32_t amp_target = MC_FP_TO_FIPu16(MC_STARTUP_VOLTAGE / MC_VBUS_FGET(MC_Analog_Read(VOLTAGE)));
    if((sp <= sp_target) || (amp <= amp_target))
        return;
    uint32_t sp_step = (uint32_t)((float)((sp - sp_target) << 8) / (float)MC_RAMP_DOWN_DURATION + 1);
    uint32_t amp_step = (uint32_t)((float)((amp - amp_target) << 8) / (float)MC_RAMP_DOWN_DURATION + 1);
    sp <<= 8;
    amp <<= 8;
    sp_target <<= 8;
    amp_target <<= 8;
    while((sp > sp_target) && (amp > amp_target))
    {
        MC_PWM_AmplitudeSet(amp >> 8);
        SET_SPEED(sp >> 8);
        amp -= amp_step;
        sp -= sp_step;
        MC_Control_DelayMs(1);
    }
}

#define STALL_RATIO  5
static void _Stall_Handler(bool stall_event)
{
    static uint8_t stall_counter = 0;
    if(stall_event)
    {
        if(stall_counter < 250)
            stall_counter += STALL_RATIO;
    }
    else
    {
        if(stall_counter > 0)
            stall_counter--;
    }
    
    if(stall_counter > STALL_RATIO*(MC_STALL_EVENTS_THRESHOLD))
    {
        stall_counter = 0;
        sense_sync = false;
        MC_PWM_ForceStop();
        MC_PWM_AmplitudeSet(0);
        SET_SPEED(0);
        if(fault_notify_handler != NULL)
            fault_notify_handler(MC_FAULT_STALL_DETECTION_EVENT);
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
    else
    {
        uint8_t temp = _Upper8_Get(estimated_position - last_known_position);
        if(temp < _Upper8_Get(MC_DEG_TO_MCANGLE(MC_TIMEOUT_ANGLE)))
            estimated_position += speed;
        else if(sense_sync == true)
            _Stall_Handler(true);
    }
    return estimated_position;
}

static void _Motor_Handler(void)
{
    static split32_t high_precision_speed;
    uint8_t sensor = MC_SENSOR_GET();
    mc_angle_t stator = phase_a;
    _Drive();

    mc_angle_t rotor_offset = (drive_direction == MC_DIR_CW) ? (MC_ROTOR_OFFSET_CW) : (MC_ROTOR_OFFSET_CCW);
    mc_angle_t rotor = _Rotor_Position_Get(sensor) + rotor_offset;

    if(sense_sync == true)
    {
        int32_t ldiff = rotor - stator + MC_DEG_TO_MCANGLE(MOTOR_PHASE_ADVANCE);
        int16_t idiff = (int16_t) ldiff;
        if(idiff != 0)
        {
            high_precision_speed.W += ((int32_t)idiff);
            if(high_precision_speed.H < MC_RPM_TO_MCSPEED(MC_MIN_SPEED)) { high_precision_speed.H = MC_RPM_TO_MCSPEED(MC_MIN_SPEED); high_precision_speed.L = 0;}
            if(high_precision_speed.H >                   MC_MAX_SPEED)  { high_precision_speed.H =                   MC_MAX_SPEED;  high_precision_speed.L = 0;}
            speed = high_precision_speed.H;
            uint8_t local_diff = _Upper8_Get(MC_DEG_TO_MCANGLE(90.0) + idiff);
            if(local_diff > _Upper8_Get(MC_DEG_TO_MCANGLE(90.0 + MC_TIMEOUT_ANGLE))) _Stall_Handler(true);
            else                                                                     _Stall_Handler(false);
        }
    }
    else
    {
        high_precision_speed.H = speed;
        high_precision_speed.L = 0;
    }

    MC_Analog_Run();
}


#if MC_SPEED_REGULATOR_EN == true
static mc_speed_t     target_speed;
// called every 1 ms
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

/*** Functions called from other modules, but not main/user callable */

void MC_Control_FaultForceOff(void)
{
    sense_sync = false;
    MC_PWM_ForceStop();
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
}

void MC_Control_FaultNotify(mc_fault_event_t fault_event)
{
    if(fault_notify_handler != NULL)
        fault_notify_handler(fault_event);
}



/******************************/
/*  PUBLIC FUNCTIONS          */
/******************************/

void MC_Control_Initialize(void)
{
    sense_sync = false;
    speed = 0;
    target_amplitude = 0;

    MC_Analog_Initialize();
    MC_Sensing_Initialize();
    MC_PWM_Initialize();
    MC_PWM_AmplitudeSet(0);
    MC_PWM_ContinuousScale(0, 0, 0);
    MC_PWM_SenseHandlerRegister(_Motor_Handler);
}

void MC_Control_SoftStart(mc_direction_t dir)
{
    sense_sync = false;
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
    _DirectionSet(dir);
    MC_PWM_ForceStart();
    bool error_state = _RampUp();
    if(error_state == false)
        sense_sync = MC_SYNCHRONIZED;
    else if(fault_notify_handler != NULL)
        fault_notify_handler(MC_FAULT_NOT_ENOUGH_VOLTAGE_EVENT);
}

void MC_Control_SoftStop(void)
{
    sense_sync = false;
    MC_PWM_ForceStop();
    MC_PWM_AmplitudeSet(0);
    SET_SPEED(0);
    MC_Control_DelayMs(MC_RAMP_DOWN_DURATION);
}

void MC_Control_DelayMs(uint32_t delay_ms)
{
    uint16_t counter;

    while(delay_ms--)
    {
        counter = (uint16_t)(1000.0 / (float)PWM_PERIOD);
        while(counter--)
        {
            _DelayWait();
        }        
        MC_DVRT_PROCESS();
        if(periodic_handler != NULL) periodic_handler();
#if (MC_SPEED_REGULATOR_EN==true)
        _ControlLoop();
#endif  /* MC_SPEED_REGULATOR_EN==true */
    }
}

void MC_Control_PeriodicHandlerRegister(mc_handler_t handler)
{
    periodic_handler = handler;
}

void MC_Control_FaultNotificationRegister(mc_fault_handler_t fhandler)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        fault_notify_handler = fhandler;
    }
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

mc_analog_data_t MC_Control_AnalogRead(mc_analog_id_t id)
{
    return MC_Analog_Read(id);
}

mc_speed_t MC_Control_SpeedGet(void)
{
    return GET_SPEED();
}

