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

#ifndef TCB0_H_INCLUDED
#define TCB0_H_INCLUDED

#include <stdint.h>
#include <avr/io.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//extern const struct TMR_INTERFACE TCB0_Interface;

/**
 * @ingroup tcb0
 * @typedef void *TCB0_cb_t
 * @brief Function pointer to callback function called by the TCB. The default value is set to NULL which means that no callback function will be used.
 */
typedef void (*TCB0_cb_t)(void);
/**
 * @ingroup tcb0
 * @brief Registers a callback function to be called at capture event.
 * @param TCB0_cb_t cb - Callback function for capture event.
 * @return None.
 */
void TCB0_CaptureCallbackRegister(TCB0_cb_t);
/**
 * @ingroup tcb0
 * @brief Initializes the TCB module
 * @param None.
 * @return None.
 */
void TCB0_Initialize(void);
/**
 * @ingroup tcb0
 * @brief Starts the TCB counter.
 * @param None.
 * @return None.
 */
void TCB0_Start(void);
/**
 * @ingroup tcb0
 * @brief Stops the TCB counter.
 * @param None.
 * @return None.
 */
void TCB0_Stop(void);
/**
 * @ingroup tcb0
 * @brief Enables the capture interrupt for the TCB.
 * @param None.
 * @return None.
 */
void TCB0_EnableCaptInterrupt(void);
/**
 * @ingroup tcb0
 * @brief Disables the capture interrupt for the TCB.
 * @param None.
 * @return None.
 */
void TCB0_DisableCaptInterrupt(void);
/**
 * @ingroup tcb0
 * @brief Enables the overflow interrupt for the TCB.
 * @param None.
 * @return None.
 */
void TCB0_EnableOvfInterrupt(void);
/**
 * @ingroup tcb0
 * @brief Disables the overflow interrupt for the TCB.
 * @param None.
 * @return None.
 */
void TCB0_DisableOvfInterrupt(void);
/**
 * @ingroup tcb0
 * @brief Reads the 16-bit timer value of the TCB.
 * @param None.
 * @return uint16_t
 */
uint16_t TCB0_Read(void);
/**
 * @ingroup tcb0
 * @brief Writes the 16-bit timer value to the TCB. 
 * @param uint16_t timerVal - 16-bit Timer value to write for TCB interface.
 * @return None.
 */
void TCB0_Write(uint16_t timerVal);
/**
 * @ingroup tcb0
 * @brief Clears the Capture Interrupt flag.
 * @param None.
 * @return None.
 */
void TCB0_ClearCaptInterruptFlag(void);
/**
 * @ingroup tcb0
 * @brief Checks if the capture interrupt is enabled.
 * @param None.
 * @return None.
 */
bool TCB0_IsCaptInterruptEnabled(void);
/**
 * @ingroup tcb0
 * @brief Clears the Overflow Interrupt flag.
 * @param None.
 * @return None.
 */
void TCB0_ClearOvfInterruptFlag(void);
/**
 * @ingroup tcb0
 * @brief Checks if the overflow interrupt is enabled.
 * @param None.
 * @return None.
 */
bool TCB0_IsOvfInterruptEnabled(void);



#ifdef __cplusplus
}
#endif

#endif /* TCB0_H_INCLUDED */

