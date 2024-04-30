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

#ifndef MC_COMPARATOR_H
#define MC_COMPARATOR_H
 
#include <stdint.h>
#include <stdbool.h>
#include "mc_types.h"

typedef void     (*mc_comparator_handler_t)(void); 

void MC_Comparator_Initialize(void);
void MC_Comparator_FaultInitialize(void);
bool MC_Comparator_Get(void);
void MC_Comparator_MuxSet(uint8_t);

void MC_Comparator_HandlerRegister(mc_handler_t cb);
void MC_Comparator_Reference(uint8_t dacRef);
bool MC_Comparator_Fault_Get(void);
void MC_Comparator_Int_Enable(void);
void MC_Comparator_Int_Disable(void);

#endif  /* MC_COMPARATOR_H */

