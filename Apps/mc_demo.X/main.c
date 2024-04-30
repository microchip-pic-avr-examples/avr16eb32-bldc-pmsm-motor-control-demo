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
#include "mc_config.h"
#include "motor_control.h"
#include "mc_sm.h"

#include "reset.h"
#include "button_led.h"


static volatile mc_fault_event_t fault_event_id;

uint16_t volatile my_speed;
uint8_t  volatile my_pot;

#define MC_COMM_INIT()
#define PRINT_OUT(...)


#if MC_DVRT_ENABLED == true
#include "usart.h"
#include "DVRunTime.h"
#undef MC_COMM_INIT 
#define MC_COMM_INIT()     ({USART_Initialize();  DVRT_Initialize();})
#endif /* MC_DVRT_ENABLED */

#if MC_PRINTOUT_ENABLED == true
#include "usart.h"
#undef MC_COMM_INIT
#define MC_COMM_INIT()      USART_Initialize()
#undef PRINT_OUT
#define PRINT_OUT(...)      printf(__VA_ARGS__)
#endif /* MC_PRINTOUT_ENABLED */

#if (MC_DVRT_ENABLED == true) && (MC_PRINTOUT_ENABLED == true)
#error "MC_DVRT_ENABLED and MC_PRINTOUT_ENABLED can't be both 'true' at the same time, please update mc_config.h"
#endif /* (MC_DVRT_ENABLED == true) && (MC_PRINTOUT_ENABLED == true) */


static void Main_Start(void);
static void Main_Stop(void);
static void Main_Reset_Idle(void);
static void Main_Reset_Running(void);
static void Main_Clear(void);
static void Main_PeriodicPrint(void);


/* application's states list */
SMTF_STATES_BEGIN(mc_sm)
SMTF_STATES_DECLARE(MOTOR_IDLE)    
SMTF_STATES_DECLARE(MOTOR_RUN)  
SMTF_STATES_END(mc_sm)


/* events list */
SMTF_EVENTS_BEGIN(mc_sm)
SMTF_EVENTS_DECLARE(BSHORT_EVENT)           
SMTF_EVENTS_DECLARE(BLONG_EVENT)           
SMTF_EVENTS_DECLARE(FAULT_EVENT)           
SMTF_EVENTS_DECLARE(PERIODIC_EVENT)           
SMTF_EVENTS_END(mc_sm)

/* transitions table */
SMTF_TRANS_TABLE_BEGIN(mc_sm)        /* BSHORT_EVENT            BLONG_EVENT                     FAULT_EVENT              PERIODIC_EVENT                     states:       */
SMTF_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(MOTOR_RUN, Main_Start,  MOTOR_IDLE, Main_Reset_Idle,    MOTOR_IDLE, NULL,        MOTOR_IDLE, NULL)               /* MOTOR_IDLE    */
SMTF_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(MOTOR_IDLE,Main_Stop,   MOTOR_RUN, Main_Reset_Running,  MOTOR_IDLE, Main_Clear,  MOTOR_RUN, Main_PeriodicPrint)  /* MOTOR_RUNNING */
SMTF_TRANS_TABLE_END()
    
SMTF_DEFINE(mc_sm, MOTOR_IDLE)


static void PrintConfig(void)
{
    PRINT_OUT("\n\rDrive mode: %s", (MC_DRIVE_MODE==MC_STEPPED_MODE)? "trapezoidal" : "sinusoidal");
    PRINT_OUT("\n\rScale mode: %s", (MC_SCALE_MODE==MC_SCALE_BOTTOM)?    "bottom" : "center");
    PRINT_OUT("\n\rSense mode: %s", (MC_CONTROL_MODE==MC_SENSORLESS_MODE)? "sensorless" : "sensored");
    PRINT_OUT("\n\rPole-pair number: %u", MC_MOTOR_PAIR_POLES);
}

static void PrintButton(void)
{
    PRINT_OUT("\n\rShort-press the button to start/stop the motor");
}

static void FaultEventNotification(void)
{
    switch(fault_event_id)
    {
        case MC_FAULT_STALL_DETECTION_EVENT:
                PRINT_OUT("\n\rStall condition occurred");
                break;
        case MC_FAULT_NOT_ENOUGH_VOLTAGE_EVENT:
                PRINT_OUT("\n\rInsufficient voltage to start");
                PRINT_OUT("\n\rIncrease the power supply voltage above %uV", (uint16_t)(0.5 + (MC_STARTUP_VOLTAGE * 2.0)));
                break;
        default: PRINT_OUT("\n\rUnknown fault occurred"); 
                break;
    }
    fault_event_id = MC_FAULT_NO_EVENT;
    PRINT_OUT("\n\r");
}

/* called from interrupt context */
static void FaultHandler(mc_fault_event_t ev)
{ 
    fault_event_id = ev;
}

/* Called on main program context from MC_DELAY_MS macro every 1 ms */
static void PeriodicHandler(void)
{
    mc_analog_data_t pot = MC_Control_AnalogRead(POTENTIOMETER);
    MC_Control_ReferenceSet(MC_POT_NGET(pot));
    my_pot = (uint8_t)MC_POT_FGET(pot);
    my_speed = (uint16_t)MC_MCSPEED_TO_RPM(MC_Control_SpeedGet());
    static uint16_t periodic_event_counter = 0;
    periodic_event_counter ++;
    if(periodic_event_counter == MC_PRINTOUT_REFRESH_INTERVAL)
    {
        periodic_event_counter = 0;
        SMTF_HANDLER_CALL(mc_sm, PERIODIC_EVENT);
    }  
}

static void Main_Start(void)
{
    static mc_direction_t direction = MC_DIR_CW;
    
    LedControl(true);
    PRINT_OUT("\n\rRamping up");
    MC_Control_SoftStart(direction);
    PRINT_OUT("\n\rRamp-up finished, motor running ");
    if(direction == MC_DIR_CW) 
    {
        PRINT_OUT("CW");
        direction = MC_DIR_CCW; 
    }
    else
    {
        PRINT_OUT("CCW");
        direction = MC_DIR_CW;
    }
}

static void Main_Stop(void)
{
    PRINT_OUT("\n\rRamping down");
    MC_Control_SoftStop();
    PRINT_OUT("\n\rMotor idle");
    LedControl(false);
    PrintButton();
}

static void Main_Reset_Idle(void)
{
    PRINT_OUT("\n\r============= REBOOT ================ ");
    ResetDevice(); 
}

static void Main_Reset_Running(void)
{
    MC_Control_SoftStop();
    Main_Reset_Idle();
}

static void Main_Clear(void)
{
    FaultEventNotification();
    uint8_t counter = 5;
    while(counter--)
    {
        LedControl(true);
        MC_DELAY_MS(250);
        LedControl(false);
        MC_DELAY_MS(250); 
    }
    PrintButton();
}

static void Main_PeriodicPrint(void)
{
    PRINT_OUT("\n\rSpeed: %u rpm, Potentiometer: %u %%      ", my_speed, my_pot);
}

int main(void)
{
    CLKCTRL_Init();
    ButtonLedInit();
    _delay_ms(1000);
    MC_COMM_INIT();
    PRINT_OUT("\n\r============= START ================= ");
    MC_Control_Initialize();
    PrintConfig();
    PRINT_OUT("\n\rLong-press the button to reboot");
    MC_Control_PeriodicHandlerRegister(PeriodicHandler);
    MC_Control_FaultNotificationRegister(FaultHandler);
    MC_DELAY_MS(500);
    PrintButton();
    while(1)
    {
        MC_DELAY_MS(BUTTON_TIME_STEP);
        button_state_t b_state = ButtonGet();

        if(b_state == BUTTON_SHORT_PRESS)                          SMTF_HANDLER_CALL(mc_sm, BSHORT_EVENT);
        else if (b_state == BUTTON_LONG_PRESS)                     SMTF_HANDLER_CALL(mc_sm, BLONG_EVENT);
        if(fault_event_id != MC_FAULT_NO_EVENT)                    SMTF_HANDLER_CALL(mc_sm, FAULT_EVENT);
    }
}

