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

#include "mc_pins.h"
#include "button_led.h"
#include <stdbool.h>

#define LED_LOW(PORT, PIN)      PORT.OUTCLR = PIN
#define LED_HIGH(PORT, PIN)     PORT.OUTSET = PIN
#define LED_TOGGLE(PORT, PIN)   PORT.OUTTGL = PIN
#define BUTTON_READ(PORT, PIN)  (((PORT.IN & PIN) == 0) ? true:false)

#define LED_BLINK_NUMBER(X)     ((X) * LED_BLINK_DURATION * 2)

void ButtonLedInit(void)
{
    BUTTON_PORT.DIRCLR = BUTTON_PIN;
    BUTTON_PORT.BUTTON_CTRL = PORT_PULLUPEN_bm;
    LED_HIGH(LED_PORT, LED_PIN);
    LED_PORT.DIRSET = LED_PIN;
}

/* this is called every BUTTON_TIME_STEP ms */
button_state_t ButtonGet(void)
{
    button_state_t retVal = BUTTON_IDLE;
    static uint16_t counter = 0;
    static bool prev_state = 0;

    bool actual_state = BUTTON_READ(BUTTON_PORT, BUTTON_PIN);
    if((actual_state == false) && (prev_state == true)) 
    {   
        if(counter < BUTTON_TIME_LONG)
            retVal = BUTTON_SHORT_PRESS;  
        counter = 0;
    }
    if(actual_state)
    {
        counter ++;
        if(counter == BUTTON_TIME_LONG)
            retVal = BUTTON_LONG_PRESS;
    }
    prev_state = actual_state;
    return retVal;
}

/* pass LED_ON, LED_OFF or LED_BLINK */
void LedControl(led_ctrl_t state)
{
    static uint8_t  toggle_counter = 0;
    static uint16_t blinks_counter = LED_BLINK_NUMBER(LED_FAULT_BLINKS);
    static bool fault_flag = false;
    if(state == LED_BLINK)  fault_flag = true;
    if(fault_flag == true)
    {
        if(blinks_counter > 0)
        {
            if(toggle_counter == LED_BLINK_DURATION - 1)   
            {
                LED_TOGGLE(LED_PORT, LED_PIN);
                toggle_counter = 0;
            }
            else toggle_counter ++;
            blinks_counter --;
        }
        else
        {
            fault_flag = false;
            blinks_counter = LED_BLINK_NUMBER(LED_FAULT_BLINKS);
            toggle_counter = 0;
            LED_TOGGLE(LED_PORT, LED_PIN);
        }
    }
    else
    {
        if(state == LED_ON)    LED_LOW (LED_PORT, LED_PIN); 
        if(state == LED_OFF)   LED_HIGH(LED_PORT, LED_PIN);   
    }
}
