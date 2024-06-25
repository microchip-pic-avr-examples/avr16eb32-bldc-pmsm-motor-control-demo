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

/**
  Section: Included Files
*/
#include "DVRunTime.h"
#include "usart.h"
#include "mc_config.h"

#if MC_DVRT_ENABLED == true
/**
  Section: Macro Declarations
*/

#ifdef __AVR16EB32__
static const uart_drv_interface_t *UART = &UART0;
#endif /* __AVR16EB32__ */

DVRT_error_t error;

/**
  Section: Driver Interface
*/
void DVRT_UART_RX_CallBack(void);
void DVRT_HandleCommand(void);
void DVRT_UART_WriteByte(uint8_t);
void DVRT_WritePacket(void);
void DVRT_Error_Callback(void);

const DVRT_interface_t DVRT = {
	.Initialize = DVRT_Initialize,
	.Process = DVRT_Process,
};


//------------------------------------------------------------------------------
/**
  Section: Variables
*/

DVRT_VariablePointerTableEntry_t DVPMT[DYNAMIC_VAR_PTR_COUNT];		/* Dynamic Variable Pointer Monitor Array */

volatile union DVCmds {
    DVRT_StreamUpdates_t stream;
    DVRT_VariableUpdate_t Var;
    DVRT_StreamIntervalUpdate_t interval;
	DVRT_CommandTemplate_t generic;
    uint8_t DVCmdArray[sizeof (DVRT_StreamUpdates_t)];
} DVRT_ReceivedCmd;								

volatile const uint16_t dvIdFw = DV_FW_CODE;

volatile uint8_t rxBufPtr, tickCounter;

uint16_t DVStreamInterval, DVStreamInterval_Counter;
uint16_t DVCmdInterval, DVCmdInterval_Counter;

struct flagS{
    unsigned streamOn	: 1;	/* Streaming On */
	unsigned osr		: 1;	/* One shot reading */
	unsigned ping		: 1;	/* Ping target microcontroller */
}DVflag;


/**
 * @ingroup dvruntime
 * @brief This blocking function writes a byte of data to selected UART.
 * @pre The transfer status is checked to see if the transmitter is ready to accept a byte before UART->Write()
 * @param txData - Data byte to write to the TX FIFO.
 * @return None.
 */
void DVRT_UART_WriteByte(uint8_t txData){
	while(!UART->IsTxReady()){
	};
	UART->Write(txData);
}

void DVRT_WritePacket(void){
	uint8_t i, k;
	DVRT_UART_WriteByte(DV_START_OF_FRAME);
	DVRT_UART_WriteByte(tickCounter++);
	if(DVflag.ping){
		DVRT_UART_WriteByte((uint8_t)(DV_FW_CODE));
		DVRT_UART_WriteByte((uint8_t)(DV_FW_CODE>>8));
	}
	else{
		for (i = 0; i < DYNAMIC_VAR_PTR_COUNT; i++) {
			for (k = 0; k < DVPMT[i].size; k++) {
				DVRT_UART_WriteByte(*(DVPMT[i].address + k));
			}		
		}		
	}		
	DVRT_UART_WriteByte(DV_END_OF_FRAME);
}


void DVRT_Initialize(void) {
	uint8_t i;
	for (i = 0; i < DYNAMIC_VAR_PTR_COUNT; i++) {
        DVPMT[i].address = 0;
        DVPMT[i].size = 0;
    }
	
    DVflag.streamOn = 1;
	DVStreamInterval = DV_STREAM_TIME;
	DVCmdInterval = DV_RX_CMD_TIMEOUT;

	error = 0;
	UART->FramingErrorCallbackRegister(&DVRT_Error_Callback);
	UART->OverrunErrorCallbackRegister(&DVRT_Error_Callback);
}


void DVRT_Error_Callback(void)
{
	error = 1;
}


void DVRT_Process(){

	if(error){
		USART_ErrorGet();
		rxBufPtr = 0;
		error = 0;
	}
	else{
		while(UART->IsRxReady()) {
		DVRT_ReceivedCmd.DVCmdArray[rxBufPtr++] = UART->Read();
		}
	} 	

	if(DVCmdInterval_Counter++ >= DVCmdInterval){
		DVCmdInterval_Counter = 0;
		DVRT_HandleCommand();		    
	}	
	
	if(DVStreamInterval_Counter++ >= DVStreamInterval){
		DVStreamInterval_Counter = 0;
		if(DVflag.streamOn){				
			DVRT_WritePacket();
		}
	}

	if(DVflag.osr || DVflag.ping){			// One shot reading or ping command execution
		DVflag.osr = 0;
		DVflag.ping = 0;
		DVflag.streamOn = 0;				// stop streaming
	}	
	
}

void DVRT_HandleCommand(void) {
    uint8_t VARcount;
	if(rxBufPtr >= DV_RX_CMD_MIN_SIZE){
		if((DVRT_ReceivedCmd.DVCmdArray[0] == DV_START_OF_FRAME) && (DVRT_ReceivedCmd.DVCmdArray[rxBufPtr-1] == DV_END_OF_FRAME)){
			switch (DVRT_ReceivedCmd.stream.command) {
				case UPDATE_VARIABLE_POINTER_TABLE:
				{
					VARcount = 0;
					while (VARcount < DVRT_ReceivedCmd.stream.size) {
						DVPMT[VARcount].size = DVRT_ReceivedCmd.stream.DVPMT[VARcount].size;
						DVPMT[VARcount].address = DVRT_ReceivedCmd.stream.DVPMT[VARcount].address;
						VARcount++;
					}
					break;
				}
				case UPDATE_VARIABLE_VALUE:
				{
					VARcount = 0;
					while (VARcount < DVRT_ReceivedCmd.Var.variablePointerTableSize) {
						*(uint8_t *)(DVRT_ReceivedCmd.Var.variableAddress + VARcount) = DVRT_ReceivedCmd.Var.variableValue[VARcount];
						VARcount++;
					}
					break;				
				}	
				case UPDATE_STREAMING_TICK:
				{
						DVStreamInterval = DVRT_ReceivedCmd.interval.period;
						DVStreamInterval_Counter = 0;
						break;
				}
				case TURN_STREAMING_OFF:
				{
						DVflag.streamOn = 0;
						break;
				}
				case TURN_STREAMING_ON:
				{
						DVflag.streamOn = 1;
						DVStreamInterval_Counter = DVStreamInterval;	 
						break;
				}
				case ONE_SHOT_READ:
				{
						DVflag.osr = 1;
						DVflag.streamOn = 1;
						DVStreamInterval_Counter = DVStreamInterval;	 
						break;
				}
				case PING:
				{
						DVflag.ping = 1;
						DVflag.streamOn = 1;
						DVStreamInterval_Counter = DVStreamInterval;	 
						break;
				}	
			}
		} 
	}
	rxBufPtr = 0;
}
#endif /* MC_DVRT_ENABLED */