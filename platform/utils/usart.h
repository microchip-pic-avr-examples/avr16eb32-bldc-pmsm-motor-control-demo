#ifndef USART_H
#define USART_H

#include "mc_config.h"

#ifdef __AVR16EB32__
#if MC_DVRT_ENABLED == true
#include "uart_drv_interface.h"
#include "usart0_dvrt.h"
#define USART_ErrorGet     USART0_ErrorGet
#endif /* MC_DVRT_COMM */
#if MC_PRINTOUT_ENABLED == true
#include "usart0.h"
inline uint8_t USART_Get(void) {if(USART0_IsRxReady()) return USART0_Read(); else return 0;}
#endif /* MC_PRINTOUT_ENABLED */
#define USART_Initialize   USART0_Initialize
#endif /* __AVR16EB32__ */

#endif /*  USART_H  */

