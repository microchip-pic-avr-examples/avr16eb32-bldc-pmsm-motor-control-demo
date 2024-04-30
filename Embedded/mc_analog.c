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
#include <avr/io.h>
#include <util/atomic.h>
#include "mc_types.h"
#include "adc0.h"
#include "mc_control_analog.h"

#ifdef AVR16EB32

static void ADC_Pins_Init(void)
{
    /* ADC pins*/
    
    /* Select ADC channel AIN7 <-> PD7 <-> CRT_REF */
    PORTD.PIN7CTRL &= ~PORT_ISC_gm;
    PORTD.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTD.PIN7CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel AIN1 <-> PD1 <-> POTENTIOMETER */
    PORTD.PIN1CTRL &= ~PORT_ISC_gm; 
    PORTD.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTD.PIN1CTRL &= ~PORT_PULLUPEN_bm;
    
     /* Select ADC channel AIN20 <-> PF4 <-> VBUS_MONITOR */
    PORTF.PIN4CTRL &= ~PORT_ISC_gm;
    PORTF.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTF.PIN4CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel AIN3 <-> PD3 <-> CRT_1 */
    PORTD.PIN3CTRL &= ~PORT_ISC_gm;
    PORTD.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTD.PIN3CTRL &= ~PORT_PULLUPEN_bm;
    
    /* Select ADC channel AIN28 <-> PC0 <-> TEMPERATURE */
    PORTC.PIN0CTRL &= ~PORT_ISC_gm;
    PORTC.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN0CTRL &= ~PORT_PULLUPEN_bm;
	
	/* Select ADC channel AIN4 <-> PD4 <-> BEMF_A */
    PORTD.PIN4CTRL &= ~PORT_ISC_gm;
    PORTD.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm; 
}
#endif  /* AVR16EB32 */



#ifdef AVR16EB32

#define _MC_ANALOG_INIT()           do{ADC_Pins_Init(); ADC0_Initialize(); }while(0)
#define _MC_ANALOG_TEMPERATURE()    do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(ADC_MUXPOS_AIN28_gc);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)
#define _MC_ANALOG_VOLTAGE()        do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(ADC_MUXPOS_AIN20_gc);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)
#define _MC_ANALOG_POTENTIOMETER()  do{ADC0_SetMuxSingleEnded(ADC_MUXPOS_GND_gc); _delay_us(1);\
                                       ADC0_SetMuxSingleEnded(ADC_MUXPOS_AIN1_gc);\
                                       ADC0_StartSingleEndedConversion();\
                                      }while(0)
#define _MC_ANALOG_RESRDY()          ADC0_IsConversionDone()
#define _MC_ANALOG_GET_RES()         ADC0_GetConversionResult()

#endif  /* AVR16EB32 */


static volatile mc_analog_data_t adc_results[ANALOG_ID_MAX];
static inline void             AnalogDataClear(mc_analog_id_t id)                    {adc_results[id].W = 0;}
static inline void             AnalogDataStore(mc_analog_id_t id, adc_result_t data) {mc_analog_data_t x = adc_results[id]; uint32_t y = x.W; uint16_t z = x.H; adc_results[id].W = y - z + (uint16_t)data;}
static inline mc_analog_data_t AnalogDataLoad (mc_analog_id_t id)                    {return adc_results[id];}


static mc_analog_id_t measurement_id;

void MC_Analog_Initialize(void)
{
    _MC_ANALOG_INIT();
    _MC_ANALOG_VOLTAGE();
    AnalogDataClear(VOLTAGE);
    AnalogDataClear(TEMPERATURE);
    AnalogDataClear(POTENTIOMETER);
    measurement_id = VOLTAGE;
}

/* called from interrupt context */
void MC_Analog_Run(void)
{
    if(_MC_ANALOG_RESRDY())
    {
        uint16_t res = _MC_ANALOG_GET_RES();
        switch(measurement_id)
        {   
            case VOLTAGE:       AnalogDataStore(measurement_id, res); measurement_id = POTENTIOMETER; _MC_ANALOG_POTENTIOMETER();  break;
            case TEMPERATURE:   AnalogDataStore(measurement_id, res); measurement_id = VOLTAGE;       _MC_ANALOG_VOLTAGE();        break;
            case POTENTIOMETER: AnalogDataStore(measurement_id, res); measurement_id = TEMPERATURE;   _MC_ANALOG_TEMPERATURE();    break;
            default: break;
        }
    }
}

mc_analog_data_t MC_Analog_Read(mc_analog_id_t index)
{
    mc_analog_data_t retVal;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        retVal = AnalogDataLoad(index);
    }
    return retVal;
}
