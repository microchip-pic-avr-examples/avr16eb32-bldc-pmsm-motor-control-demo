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


#ifndef DVRT_CONFIG_H
#define DVRT_CONFIG_H

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

#define DYNAMIC_VAR_PTR_COUNT  8       /* Number of Dynamic Variables Pointers in the Monitor Table */ 	
#define DV_STREAM_TIME		   20      /* Streaming time interval (time dependent on main loop tick) */
#define DV_RX_CMD_TIMEOUT	   200     /* Command timeout: Timeout after number of times DVRT_process() is called */
#define DV_FW_CODE			   (uint16_t)261 /* Project Firmware Code */

#define DV_START_OF_FRAME	0x03       /* Start Frame Byte */
#define DV_END_OF_FRAME		0xFC       /* End Frame Byte */
#define DV_RX_CMD_MIN_SIZE	3          /* Minimun number of byte in a RX command */

/** @} */ 
#ifdef __cplusplus
}
#endif

#endif //DVRT_CONFIG_H