/**
  @Company
    Microchip Technology Inc.

  @Description
    This Source file provides APIs.
    Generation Information :
    Driver Version    :   1.0.0
*/
/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/


#include <avr/io.h>
#include "clkctrl.h"
#include "usart0.h"
#include "mc_config.h"

#if MC_PRINTOUT_ENABLED == true

#if defined(__GNUC__)

int USART0_printCHAR(char character, FILE *stream)
{
    (void)stream;
    USART0_Write(character);
    return 0;
}

FILE USART0_stream = FDEV_SETUP_STREAM(USART0_printCHAR, NULL, _FDEV_SETUP_WRITE);

#elif defined(__ICCAVR__)

int putchar(int outChar)
{
    USART0_Write(outChar);
    return outChar;
}
#endif

void USART0_Initialize()
{    
    PORTMUX.USARTROUTEA = PORTMUX_USART0_ALT4_gc;
    PORTC.DIRSET = PIN1_bm;
   
    //set baud rate register
    USART0.BAUD = USART0_BAUD_RATE(460800);
	
    //RXCIE disabled; TXCIE disabled; DREIE disabled; RXSIE disabled; LBME disabled; ABEIE disabled; RS485 DISABLE; 
    USART0.CTRLA = 0x00;
	
    //RXEN disabled; TXEN enabled; SFDEN disabled; ODME disabled; RXMODE NORMAL; MPCM disabled; 
    USART0.CTRLB = 0x40;
	
    //CMODE ASYNCHRONOUS; PMODE DISABLED; SBMODE 1BIT; CHSIZE 8BIT; UDORD disabled; UCPHA disabled; 
    USART0.CTRLC = 0x03;
	
    //DBGCTRL_DBGRUN
    USART0.DBGCTRL = 0x00;
	
    //EVCTRL_IREI
    USART0.EVCTRL = 0x00;
	
    //RXPLCTRL_RXPL
    USART0.RXPLCTRL = 0x00;
	
    //TXPLCTRL_TXPL
    USART0.TXPLCTRL = 0x00;
    

    
	

#if defined(__GNUC__)
    stdout = &USART0_stream;
#endif
    USART0_Enable();
}

void USART0_Enable()
{
    USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

void USART0_EnableRx()
{
    USART0.CTRLB |= USART_RXEN_bm;
}

void USART0_EnableTx()
{
    USART0.CTRLB |= USART_TXEN_bm;
}

void USART0_Disable()
{
    USART0.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

bool USART0_IsTxReady()
{
    return (USART0.STATUS & USART_DREIF_bm);
}

bool USART0_IsRxReady()
{
    return (USART0.STATUS & USART_RXCIF_bm);
}

bool USART0_IsTxBusy()
{
    return (!(USART0.STATUS & USART_TXCIF_bm));
}

bool USART0_IsTxDone()
{
    return (USART0.STATUS & USART_TXCIF_bm);
}

uint8_t USART0_Read()
{
    while (!(USART0.STATUS & USART_RXCIF_bm))
            ;
    return USART0.RXDATAL;
}

void USART0_Write(const uint8_t data)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
            ;
    USART0.TXDATAL = data;
}
#endif /* MC_PRINTOUT_ENABLED */

