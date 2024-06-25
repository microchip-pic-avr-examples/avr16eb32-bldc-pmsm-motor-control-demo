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

#ifndef MC_PINS_H
#define MC_PINS_H

#include <avr/io.h>

/** LED and BUTTON PINS definitions **/

#ifdef __AVR16EB32__
#define LED_PORT            PORTF
#define LED_PIN             PIN5_bm
#define BUTTON_PORT         PORTF
#define BUTTON_PIN          PIN6_bm
#define BUTTON_CTRL         PIN6CTRL
#endif /* __AVR16EB32__ */

/** HALL PORTS and PINS definitions **/

#ifdef __AVR16EB32__
#define HA_PORT             PORTF
#define HA_PIN              PIN1_bm
#define HB_PORT             PORTF
#define HB_PIN              PIN2_bm
#define HC_PORT             PORTF
#define HC_PIN              PIN3_bm
#endif  /* __AVR16EB32__ */

/** BEMF PORTS and PINS definitions **/

#ifdef __AVR16EB32__
#define PHASE_A_PORT        PORTD
#define PHASE_A_CTRL        PIN4CTRL
#define PHASE_A_ADC_PIN     ADC_MUXPOS_AIN4_gc
#define PHASE_A_AC_PIN      AC_MUXPOS_AINP5_gc
#define PHASE_B_PORT        PORTD
#define PHASE_B_CTRL        PIN5CTRL
#define PHASE_B_AC_PIN      AC_MUXPOS_AINP6_gc
#define PHASE_C_PORT        PORTD
#define PHASE_C_CTRL        PIN6CTRL
#define PHASE_C_AC_PIN      AC_MUXPOS_AINP3_gc
#define PHASE_N_PORT        PORTD
#define PHASE_N_CTRL        PIN0CTRL
#define PHASE_N_AC_PIN      AC_MUXNEG_AINN1_gc
#define CMP_OUT_PORT        PORTA
#define CMP_OUT_PIN         PIN7_bm
#endif  /* __AVR16EB32__ */

/** ANALOG PARAMETERS PORTS and PINS definitions **/

#ifdef __AVR16EB32__
#define VBUS_PORT           PORTD
#define VBUS_CTRL           PIN4CTRL
#define VBUS_ADC_PIN        ADC_MUXPOS_AIN20_gc
#define POT_PORT            PORTD
#define POT_CTRL            PIN1CTRL
#define POT_ADC_PIN         ADC_MUXPOS_AIN1_gc
#define TEMP_PORT           PORTD
#define TEMP_CTRL           PIN0CTRL
#define TEMP_ADC_PIN        ADC_MUXPOS_AIN28_gc
#define CRT_SNS_PORT        PORTD
#define CRT_SNS_CTRL        PIN3CTRL
#define CRT_SNS_ADC_PIN     ADC_MUXPOS_AIN3_gc
#define CRT_REF_PORT        PORTD
#define CRT_REF_CTRL        PIN7CTRL
#define CRT_REF_ADC_PIN     ADC_MUXPOS_AIN7_gc
#define CRT_FAULT_AC_PIN    AC_MUXPOS_AINP1_gc
#endif  /* __AVR16EB32__ */


/** PWM PORTS and PINS definitions **/

#ifdef __AVR16EB32__
#define PWM_AH_PORT         PORTA
#define PWM_AH_PIN          PIN0_bm
#define PWM_AL_PORT         PORTA
#define PWM_AL_PIN          PIN1_bm
#define PWM_BH_PORT         PORTA
#define PWM_BH_PIN          PIN2_bm
#define PWM_BL_PORT         PORTA
#define PWM_BL_PIN          PIN3_bm
#define PWM_CH_PORT         PORTA
#define PWM_CH_PIN          PIN4_bm
#define PWM_CL_PORT         PORTA
#define PWM_CL_PIN          PIN5_bm
#endif  /* __AVR16EB32__ */

#endif /* MC_PINS_H */
