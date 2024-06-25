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
#include "mc_pins.h"
#include "adc0.h"

#include "mc_control_analog.h"
#include "mc_control_fault.h"


static void ADC_Pins_Init(void)
{
    /* Select ADC channel input for VBUS_MONITOR */
    VBUS_PORT.VBUS_CTRL &= ~PORT_ISC_gm;
    VBUS_PORT.VBUS_CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    VBUS_PORT.VBUS_CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel input for POTENTIOMETER */
    POT_PORT.POT_CTRL &= ~PORT_ISC_gm; 
    POT_PORT.POT_CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    POT_PORT.POT_CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel input for TEMPERATURE */
    TEMP_PORT.TEMP_CTRL &= ~PORT_ISC_gm;
    TEMP_PORT.TEMP_CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    TEMP_PORT.TEMP_CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel input for CRT_SNS */
    CRT_SNS_PORT.CRT_SNS_CTRL &= ~PORT_ISC_gm;
    CRT_SNS_PORT.CRT_SNS_CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    CRT_SNS_PORT.CRT_SNS_CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel input for CRT_REF */
    CRT_REF_PORT.CRT_REF_CTRL &= ~PORT_ISC_gm;
    CRT_REF_PORT.CRT_REF_CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    CRT_REF_PORT.CRT_REF_CTRL &= ~PORT_PULLUPEN_bm; 
}

#define _MC_ANALOG_TEMPERATURE()    do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(TEMP_ADC_PIN);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)
#define _MC_ANALOG_CURRENT()        do{ADC0_SetMuxDifferential(ADC_MUXPOS_GND_gc,   ADC_MUXNEG_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxDifferential(CRT_SNS_ADC_PIN,  CRT_REF_ADC_PIN);\
                                       ADC0_StartDiffConversion();\
                                      }while(0)
#define _MC_ANALOG_VOLTAGE()        do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(VBUS_ADC_PIN);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)
#define _MC_ANALOG_POTENTIOMETER()  do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(POT_ADC_PIN);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)
#define _MC_ANALOG_BEMF_A()         do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(PHASE_A_ADC_PIN);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)

#define _MC_ANALOG_INIT()           do{ADC_Pins_Init(); ADC0_Initialize(); }while(0)
#define _MC_ANALOG_RESRDY()         ADC0_IsConversionDone()
#define _MC_ANALOG_GET_RES()        ADC0_GetConversionResult()

static mc_analog_id_t measurement_id;
static volatile mc_analog_data_t adc_filters[AID_MAX];

static inline void             FilterClear(mc_analog_id_t id)                       {adc_filters[id].W = 0;}
static inline void             FilterUnsigned(mc_analog_id_t id, adc_result_t data) {mc_analog_data_t x = adc_filters[id]; uint32_t y =           x.W;  uint16_t z =           x.H;  adc_filters[id].W =            y - z + (uint16_t)data;}
static inline void             FilterSigned(mc_analog_id_t id,   adc_result_t data) {mc_analog_data_t x = adc_filters[id]; int32_t  y = (int32_t)(x.W);  int16_t z = (int16_t)(x.H); adc_filters[id].W = (uint32_t)(y - z +  (int16_t)data);}
static inline uint16_t         FilterRead (mc_analog_id_t id)                       {return adc_filters[id].H;}


void MC_Analog_Initialize(void)
{
    _MC_ANALOG_INIT();
    FilterClear(AID_CURRENT);
    FilterClear(AID_VOLTAGE);
    FilterClear(AID_TEMPERATURE);
    FilterClear(AID_POTENTIOMETER);
    measurement_id = AID_CURRENT;
    _MC_ANALOG_CURRENT();
}

/* called from interrupt context */
void MC_Analog_Run(void)
{
    if(_MC_ANALOG_RESRDY())
    {
        mc_analog_id_t prev_id = measurement_id;
        uint16_t res = _MC_ANALOG_GET_RES();

        switch(measurement_id)
        {   
            case AID_VOLTAGE:       FilterUnsigned(measurement_id, res); measurement_id = AID_POTENTIOMETER;   _MC_ANALOG_POTENTIOMETER();  break;
            case AID_CURRENT:       FilterSigned(measurement_id, res);   measurement_id = AID_VOLTAGE;         _MC_ANALOG_VOLTAGE();        break;                                
            case AID_TEMPERATURE:   FilterUnsigned(measurement_id, res); measurement_id = AID_CURRENT;         _MC_ANALOG_CURRENT();        break;
            case AID_POTENTIOMETER: FilterUnsigned(measurement_id, res); measurement_id = AID_TEMPERATURE;     _MC_ANALOG_TEMPERATURE();    break;
            default: break;
        }
        MC_Fault_LimitsCheck(prev_id, FilterRead(prev_id));
    }
}

uint16_t MC_Analog_Read(mc_analog_id_t index)
{
    uint16_t retVal;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        retVal = FilterRead(index);
    }
    return retVal;
}


