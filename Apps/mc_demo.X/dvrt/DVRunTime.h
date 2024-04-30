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

#ifdef DEFINE_VAR_DVREALTIME
#define EXTERN						/* nothing */
#else
#define EXTERN	extern
#endif /* DEFINE_VARIABLES */

#ifndef DVRUNTIME_H
#define DVRUNTIME_H

/**
  Section: Included Files
*/

#include <stdbool.h>
#include <stdint.h>
#include "DVRunTime_interface.h"
#include "DVRunTime_types.h"
#include "DVRunTime_config.h"


#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif


/**
 Section: Data Type Definitions
 */

/**
* @ingroup dvruntime
* @brief Declaration of the DVRT interface and its function pointers.
*/
extern const DVRT_interface_t DVRT;


/**
 * @ingroup dvruntime
 * @brief Initializes the DVRT driver. 
 * Sets the variable values to their initial values and registers error callback functions for UART errors.
 * @param None.
 * @return None.
 */
void DVRT_Initialize(void);

/**
 * @ingroup dvruntime
 * @brief Processes the DVRT driver. 
 * Checks the UART for incoming data and processes the data if it is available. 
 * It also checks for time-out conditions and triggers the periodic sending of data if required. Finally, it executes one-shot readings and ping commands.
 * @param None.
 * @return None.
 */
void DVRT_Process(void);


#ifdef __cplusplus
}
#endif

#endif //DVRUNTIME_H