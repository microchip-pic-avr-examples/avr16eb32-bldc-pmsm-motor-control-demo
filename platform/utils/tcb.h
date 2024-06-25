#ifndef TCB_H
#define TCB_H

#ifdef __AVR16EB32__
#include "tcb0.h"
#define TCB_Initialize                      TCB0_Initialize                      
#define TCB_StartTimer                      TCB0_Start                           
#define TCB_CallbackRegister                TCB0_CaptureCallbackRegister         
#endif /* __AVR16EB32__ */

#endif /*  TCB_H  */
