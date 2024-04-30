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

#include <avr/io.h>
#include "button_led.h"


#ifdef AVR16EB32
#define LED_PORT     PORTF
#define LED_PIN      PIN5_bm
#define BUTTON_PORT  PORTF
#define BUTTON_PIN   PIN6_bm
#define BUTTON_CTRL  PIN6CTRL
#endif


#define LED_LOW(PORT, PIN)      PORT.OUTCLR = PIN
#define LED_HIGH(PORT, PIN)     PORT.OUTSET = PIN
#define BUTTON_READ(PORT, PIN)  (((PORT.IN & PIN) == 0) ? true:false)

void ButtonLedInit(void)
{
    BUTTON_PORT.DIRCLR = BUTTON_PIN;
    BUTTON_PORT.BUTTON_CTRL = PORT_PULLUPEN_bm;
    LED_HIGH(LED_PORT, LED_PIN);
    LED_PORT.DIRSET = LED_PIN;
}

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


/* pass 'true' to turn on the LED */
void LedControl(bool state)
{
    if(state == true) LED_LOW (LED_PORT, LED_PIN);
    else              LED_HIGH(LED_PORT, LED_PIN);
}

