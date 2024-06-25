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
#include "mc_config.h"
#include "motor_control.h"
#include "reset.h"
#include "button_led.h"

uint16_t volatile my_speed;
uint8_t  volatile my_pot;

#define MC_COMM_INIT()
#define PRINT_OUT(...)

#if MC_DVRT_ENABLED == true
#include "usart.h"
#include "DVRunTime.h"
#undef MC_COMM_INIT 
#define MC_COMM_INIT()     do{USART_Initialize(); DVRT_Initialize();} while(0)
#endif /* MC_DVRT_ENABLED */

#if MC_PRINTOUT_ENABLED == true
#include "usart.h"
#undef MC_COMM_INIT
#define MC_COMM_INIT()      USART_Initialize()
#undef PRINT_OUT
#define PRINT_OUT(...)      printf(__VA_ARGS__)
#endif /* MC_PRINTOUT_ENABLED */


#define LED_BLINK_DURATION     200 // ms
#define LED_BLINK_NUMBER       5

static void PrintConfig(void)
{
    PRINT_OUT("\n\rDrive mode: %s", (MC_DRIVE_MODE==MC_STEPPED_MODE)? "trapezoidal" : "sinusoidal");
    PRINT_OUT("\n\rScale mode: %s", (MC_SCALE_MODE==MC_SCALE_BOTTOM)?    "bottom" : "center");
    PRINT_OUT("\n\rSense mode: %s", (MC_CONTROL_MODE==MC_SENSORLESS_MODE)? "sensorless" : "sensored");
    PRINT_OUT("\n\rPole-pair number: %u", MC_MOTOR_PAIR_POLES);
    PRINT_OUT("\n\rLong-press the button to reboot");
    PRINT_OUT("\n\rShort-press the button to start or stop the motor");
}

static void Main_ShowFaults(mc_status_t status)
{
#if MC_PRINTOUT_ENABLED == true
    if(status.flags)
    {
        const char *str_uv = "";
        const char *str_ov = "";
        const char *str_stall = "";
        const char *str_hot = "";
        const char *str_oc = "";
        if(status.flags & MC_FAULT_UNDERVOLTAGE_MASK) str_uv = "Undervoltage ";
        if(status.flags & MC_FAULT_OVERVOLTAGE_MASK)  str_ov = "Overvoltage ";
        if(status.flags & MC_FAULT_STALL_MASK)     str_stall = "Stall ";
        if(status.flags & MC_FAULT_OVERHEAT_MASK)    str_hot = "Hot ";
        if(status.flags & MC_FAULT_OVERCURRENT_MASK)  str_oc = "Overcurrent ";

        PRINT_OUT("\n\rFaults: %s%s%s%s%s", str_uv, str_ov, str_stall, str_hot, str_oc);
    }
#else /* MC_PRINTOUT_ENABLED == true */
    (void)status;
#endif /* MC_PRINTOUT_ENABLED == true */
}

static void Main_PeriodicPrint(void)
{
    PRINT_OUT("\n\rSpeed: %uRPM, Current: %dmA, VBUS: %umV, Temperature: %u°C, Pot: %u%%      ",
                my_speed,
                MC_Control_CurrentRead(),
                MC_Control_VoltageBusRead(),
                MC_Control_TemperatureRead(),
                my_pot
                );
}

/* Called on main program context from MC_DELAY_MS macro every 1 ms */
static void PeriodicHandler(mc_status_t status)
{
    MC_Control_ReferenceSet(MC_Control_FastPotentiometerRead());
    my_pot = MC_Control_PotentiometerRead();
    my_speed = (uint16_t)MC_MCSPEED_TO_RPM(MC_Control_SpeedGet());
    static uint16_t periodic_event_counter = 0;
    if(status.state == RUNNING)
    {
        periodic_event_counter ++;
        if(periodic_event_counter == MC_PRINTOUT_REFRESH_INTERVAL)
        {
            Main_PeriodicPrint();
            periodic_event_counter = 0;
        }   
    }
}

static void LedBlink(uint8_t counter)
{
    while(counter--)
    {
        LedControl(true);
        MC_DELAY_MS(LED_BLINK_DURATION);
        LedControl(false);
        MC_DELAY_MS(LED_BLINK_DURATION); 
    }
}

int main(void)
{
    CLKCTRL_Init();
    ButtonLedInit();
    _delay_ms(1000);
    MC_COMM_INIT();
    PRINT_OUT("\n\r============= START ================= ");
    MC_Control_Initialize();
    MC_Control_PeriodicHandlerRegister(PeriodicHandler);  
    MC_DELAY_MS(500);
    PrintConfig();
    
    mc_status_t prev_motor_state; prev_motor_state.state = -1;
    mc_direction_t direction  =  MC_DIR_CW;
    
    while(1)
    {
        MC_DELAY_MS(BUTTON_TIME_STEP);
        button_state_t b_state = ButtonGet();
        mc_status_t motor_state = MC_Control_StatusGet();
        
        if(motor_state.state == FAULT) LedBlink(1);

        if((b_state != BUTTON_IDLE) || (motor_state.word != prev_motor_state.word))
        {
            if (b_state == BUTTON_LONG_PRESS)
            {
                if(motor_state.state == RUNNING)
                  MC_Control_StartStop(0);
                PRINT_OUT("\n\r============= REBOOT ================ ");
                MC_DELAY_MS(10);
                ResetDevice();
            }
            switch(motor_state.state)
            {
                case IDLE:      if(b_state == BUTTON_SHORT_PRESS)
                                {
                                    PRINT_OUT("\n\rRamping-up ... %s", (direction==MC_DIR_CW)? "CW":"CCW");
                                    LedControl(true);
                                    MC_Control_StartStop(direction);
                                    if(direction == MC_DIR_CW) direction = MC_DIR_CCW;
                                    else                       direction = MC_DIR_CW;
                                }
                                else
                                {
                                    PRINT_OUT("\n\rMotor idle");
                                }
                                break;
                case RUNNING:   if(b_state == BUTTON_SHORT_PRESS) 
                                {
                                    PRINT_OUT("\n\rRamping-down");
                                    MC_Control_StartStop(0);
                                    LedControl(false);
                                }
                                break;
                case FAULT:     Main_ShowFaults(motor_state);
                                LedBlink(LED_BLINK_NUMBER);
                                break;
                default: break;        
            }
            prev_motor_state = motor_state;
        }
    }
}
