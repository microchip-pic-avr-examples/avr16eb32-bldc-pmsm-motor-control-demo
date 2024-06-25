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

#ifndef DVRT_TYPES_H
#define DVRT_TYPES_H

/**
  Section: Included Files
*/

#include <stdbool.h>
#include <stdint.h>
#include "usart.h"
#include "DVRunTime_config.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/**
 * @ingroup dvruntime
 * @enum DVRT_commands
 * @brief Enum containing a list of commands used to implement the DV Run Time protocol. 
 */
typedef enum DVRT_commands{ 
    UPDATE_VARIABLE_POINTER_TABLE=0,  
    UPDATE_VARIABLE_VALUE=1,     
    UPDATE_STREAMING_TICK=2,     
    TURN_STREAMING_OFF=3,        
    TURN_STREAMING_ON=4,         
    ONE_SHOT_READ=5,             
    PING=6                     
} DVRT_commands;	


/**
 * @ingroup dvruntime
 * @struct DVRT_VariablePointerTableEntry
 * @brief Defines one entry of the Variable Pointer Table.
 * @brief Defines one entry of the Variable Pointer Table.
 */
typedef struct __attribute__((packed)) DVRT_VariablePointerTableEntry {
    uint8_t size;                             /* Size of address */
    uint8_t * address;			              /* Pointer to address of Variable Pointer Table */	 
} DVRT_VariablePointerTableEntry_t;

/**
 * @ingroup dvruntime
 * @struct DVRT_StreamUpdates
 * @brief Defines the data structure for updating the stream of variables sent to the Data Visualizer.
 */
typedef struct __attribute__((packed)) DVRT_StreamUpdates {
    uint8_t startOfFrame;                     /* Start of frame byte.  */
    uint8_t command;                          /* Byte indicating the command type. This should be set to UPDATE_VARIABLE_POINTER_TABLE (0) for this command. */
    uint8_t size;                             /* Byte indicating the size of variable pointer table */
    DVRT_VariablePointerTableEntry_t DVPMT[DYNAMIC_VAR_PTR_COUNT];   /* Array of DVRT_VariablePointerTableEntry_t structs representing the addresses of the variable pointer table. */            
    uint8_t endOfFrame;                       /* End of frame byte. */
} DVRT_StreamUpdates_t;

/**
 * @ingroup dvruntime
 * @struct DVRT_VariableUpdate
 * @brief Defines the data structure needed to update the Variable Pointer Table (DVPMT).
 */
typedef struct __attribute__((packed)) DVRT_VariableUpdate {
    uint8_t startOfFrame;                     /* Start of frame */
    uint8_t command;                          /* Byte indicating the command type. This should be set to UPDATE_VARIABLE_VALUE (1) for this command. */
    uint8_t variablePointerTableSize;         /* Size of the variable pointer table */
    uint8_t * variableAddress;                /* Address of the variable to add to the pointer table */
    uint8_t variableValue[5];                 /* Value of the variable */
    uint8_t endOfFrame;                       /* End of frame */
} DVRT_VariableUpdate_t;

/**
 * @ingroup dvruntime
 * @struct DVRT_StreamIntervalUpdate
 * @brief Defines the data structure for updating the streaming period of frames sent from the DVRT to the Data Visualizer.
 */
typedef struct __attribute__((packed)) DVRT_StreamIntervalUpdate {
    uint8_t startOfFrame;                     /* Start of frame byte. */
    uint8_t command;                          /* Byte indicating the command type. This should be set to UPDATE_STREAMING_TICK (2) for this command. */
    uint16_t period;                          /* The streaming interval period, in milliseconds. */
    uint8_t endOfFrame;                       /* End of frame byte. */
} DVRT_StreamIntervalUpdate_t;

/**
 * @ingroup dvruntime
 * @struct DVRT_CommandTemplate
 * @brief Typedef for struct containing the elements needegit d for a generic DVRT command.  
 */
typedef struct DVRT_CommandTemplate{
    uint8_t startOfFrame;                    /* Start of frame */
    uint8_t command;                         /* DVRT Command */
    uint8_t data;				             /* Data payload of command */		
    uint8_t endOfFrame;                      /* End of frame */
} DVRT_CommandTemplate_t;

/**
 * @ingroup dvruntime
 * @struct DVPMTs
 * @brief Typedef for element of the Dynamic Variable Pointer Monitor Table (DVPMT).
 */
typedef struct DVPMTs {				 
	uint8_t * address;		                /* Size of address */		 
    uint8_t size;                           /* Pointer to address of Variable Pointer Table */	
} DVPMT_t;


typedef size_t DVRT_error_t;                /* Type used for catching UART errors */

#ifdef __cplusplus
}
#endif

#endif /* DVRT_TYPES_H */
