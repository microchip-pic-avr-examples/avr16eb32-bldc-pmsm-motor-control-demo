#include "mc_config.h"
#include "mc_internal_defines.h"
#include "mc_comparator.h"
#include "mc_pwm.h"
#include "mc_control_fault.h"


#define MC_UNDER_VOLTAGE_LIMIT         ((MC_UNDER_VOLTAGE_EVENT_LEVEL   > (MC_STARTUP_VOLTAGE * 2.0)) ? MC_UNDER_VOLTAGE_EVENT_LEVEL   : (MC_STARTUP_VOLTAGE * 2.0))
#define MC_UNDER_VOLTAGE_LIMIT_RESTORE ((MC_UNDER_VOLTAGE_RESTORE_LEVEL > (MC_STARTUP_VOLTAGE * 2.0)) ? MC_UNDER_VOLTAGE_RESTORE_LEVEL : (MC_STARTUP_VOLTAGE * 2.0 + 1.0))


#if MC_FAULT_ENABLED == true
static void FastHandler(void)
{   
    MC_Control_FaultForceOff();
    MC_Control_FaultNotify(MC_FAULT_OVER_CURRENT_EVENT);
}
#endif /* MC_FAULT_ENABLED == true */

void MC_Fault_Initialize(void)
{
#if MC_FAULT_ENABLED == true
    MC_Comparator_FaultInitialize(); 
    MC_Comparator_HandlerRegister(FastHandler);
    MC_Comparator_Reference(MC_CURR_TO_ILIM(MC_OVER_CURRENT_PEAK_EVENT_LEVEL));
    MC_Comparator_Int_Enable();
#else /* MC_FAULT_ENABLED == true */
    MC_Comparator_Int_Disable();
#endif /* MC_FAULT_ENABLED == true */
}

void MC_Fault_Recovery(void)
{   
    MC_PWM_FaultRecovery();
}

void MC_Fault_LimitsCheck(mc_analog_id_t id, uint16_t res)
{
#if MC_FAULT_ENABLED == true
    static bool temp_status = false;
    static bool ov_status = false;
    static bool uv_status = false;
    static uint16_t ovoltage = MC_VOLTAGE_TO_VBUS(MC_OVER_VOLTAGE_EVENT_LEVEL);
    static uint16_t uvoltage = MC_VOLTAGE_TO_VBUS(MC_UNDER_VOLTAGE_LIMIT);
    static uint16_t otemp = MC_CELSIUS_TO_TEMP(MC_OVER_TEMPERATURE_LEVEL);
    mc_fault_event_t fault_event = MC_FAULT_NO_EVENT;
    int16_t ires;

    switch(id)
    {
        case AID_VOLTAGE:   if(res > ovoltage) 
                            {
                                if(ov_status == false)
                                {
                                    ov_status = true;
                                    ovoltage = MC_VOLTAGE_TO_VBUS(MC_OVER_VOLTAGE_RESTORE_LEVEL);
                                    fault_event = MC_FAULT_OVER_VOLTAGE_EVENT;
                                }
                            } 
                            else
                            {
                                if(res < uvoltage)
                                {
                                    if(uv_status == false)
                                    {
                                        uv_status = true;
                                        uvoltage = MC_VOLTAGE_TO_VBUS(MC_UNDER_VOLTAGE_LIMIT_RESTORE);
                                        fault_event = MC_FAULT_UNDER_VOLTAGE_EVENT;
                                    }
                                }
                                else
                                {
                                    if(ov_status == true)
                                    {
                                        ov_status = false;
                                        ovoltage = MC_VOLTAGE_TO_VBUS(MC_OVER_VOLTAGE_EVENT_LEVEL);
                                        fault_event = MC_FAULT_OVER_VOLTAGE_RESTORE;
                                    }
                                    else
                                        if(uv_status == true)
                                        {
                                            uv_status = false;
                                            uvoltage = MC_VOLTAGE_TO_VBUS(MC_UNDER_VOLTAGE_LIMIT);
                                            fault_event = MC_FAULT_UNDER_VOLTAGE_RESTORE;
                                        }
                                }
                            }
                            break;
        case AID_TEMPERATURE:
                            if(res >= otemp)
                            {
                                if(temp_status == false)
                                {
                                    temp_status = true;
                                    otemp = MC_CELSIUS_TO_TEMP(MC_TEMPERATURE_RESTORE_LEVEL);
                                    fault_event = MC_FAULT_HIGH_TEMPERATURE_EVENT;
                                }
                            }
                            else
                            {
                                if(temp_status == true)
                                {
                                    temp_status = false;
                                    otemp = MC_CELSIUS_TO_TEMP(MC_OVER_TEMPERATURE_LEVEL);
                                    fault_event = MC_FAULT_HIGH_TEMPERATURE_RESTORE;
                                }
                            }
                            break;
        case AID_CURRENT:   ires = (int16_t)res;
                            if(ires < 0)
                                ires = 0 - ires;
                            if(ires >= MC_AMPS_TO_CURR(MC_OVER_CURRENT_AVG_EVENT_LEVEL))
                                fault_event = MC_FAULT_OVER_CURRENT_EVENT;
                            break;
        default: break;
    }
    
    if(fault_event != MC_FAULT_NO_EVENT)
    {
        MC_Control_FaultForceOff();
        MC_Control_FaultNotify(fault_event);
    }
#else /* MC_FAULT_ENABLED == true */
    (void)id;
    (void)res;
#endif /* MC_FAULT_ENABLED == true */
}
