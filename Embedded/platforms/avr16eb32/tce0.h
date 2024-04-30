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

#ifndef TCE0_H
#define TCE0_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>


/*******************************************************************************
Macros defined for features not supported in the device
*******************************************************************************/


/**
*@ingroup tce0
*@brief
*Defines the macro associated with the conversion from us(microseconds) to clock increments for setting the period.
*/
#define TCE0_PER_US_TO_TICKS(US, F_CLOCK, TCE0_PRESCALER) (uint16_t)(((float)(F_CLOCK) * (float)(US)) / (1000000.0 * (float)(TCE0_PRESCALER)) + 0.5) - 1


/**
 *@ingroup tce0
 *@brief
 *Defines the macro associated with the conversion from ms(milliseconds) to clock increments for setting the period.
 */
#define TCE0_PER_MS_TO_TICKS(MS, F_CLOCK, TCE0_PRESCALER) (uint16_t)(((float)(F_CLOCK) * (float)(MS)) / (1000.0 * (float)(TCE0_PRESCALER)) + 0.5) - 1


/**
*@ingroup tce0
*@brief
*Defines the macro associated with the conversion from s(seconds) to clock increments for setting the period.
*/
#define TCE0_PER_S_TO_TICKS(S, F_CLOCK, TCE0_PRESCALER)  (uint16_t)(((float)(F_CLOCK) * (float)(S)) / (1.0 * (float)(TCE0_PRESCALER)) + 0.5) - 1


/**
*@ingroup tce0
*@brief
*Defines the macro associated with the conversion from us(microseconds) to clock increments for setting the compare registers.
*/
#define TCE0_CMP_US_TO_TICKS(US, F_CLOCK, TCE0_PRESCALER) (uint16_t)(((float)(F_CLOCK) * (float)(US)) / (1000000.0 * (float)(TCE0_PRESCALER)) + 0.5)


/**
 *@ingroup tce0
 *@brief
 *Defines the macro associated with the conversion from ms(milliseconds) to clock increments for setting the compare registers.
 */
#define TCE0_CMP_MS_TO_TICKS(MS, F_CLOCK, TCE0_PRESCALER) (uint16_t)(((float)(F_CLOCK) * (float)(MS)) / (1000.0 * (float)(TCE0_PRESCALER)) + 0.5)


/**
*@ingroup tce0
*@brief
*Defines the macro associated with the conversion from s(seconds) to clock increments for setting the compare registers.
*/
#define TCE0_CMP_S_TO_TICKS(S, F_CLOCK, TCE0_PRESCALER)  (uint16_t)(((float)(F_CLOCK) * (float)(S)) / (1.0 * (float)(TCE0_PRESCALER)) + 0.5)


/**
*@ingroup tce0
*@brief
*Defines the macro associated with the conversion from Hz(Hertz) to clock increments.  Used in Frequency mode.
*/
#define TCE0_HZ_TO_CLOCKS_FREQUENCYMODE(HZ, F_CLOCK, TCE0_PRESCALER)(uint16_t)((float)(F_CLOCK) / (2 * (float)(HZ) * (float)(TCE0_PRESCALER)) - 1)


/**
 * @ingroup tce0
 * @brief Pointer to a function to be used as a callback handler when an overflow interrupt event occurs.
 * @param None.
 * @return None.
 */
typedef void (*TCE0_cb_t)(void);


/**
@ingroup tce0
@brief Lists the possible states of the TCE0.
 * - @c TCE_STATUS_IDLE = 0 @c      -   The timer is not running
 * - @c TCE_STATUS_RUNNING = 1 @c   -   The timer is running
 */
typedef enum
{
    TCE_STATUS_IDLE =       0,          /* The timer is not running*/
    TCE_STATUS_RUNNING =    1,           /* The timer is running */
} TCE0_status_t;


/**
 * @ingroup tce0
 * @brief Setter function for the TCE0 overflow callback.
 * @pre None.
 * @param CallbackHandler - Pointer to custom callback.
 * @return None.
 */
void TCE0_OverflowCallbackRegister(TCE0_cb_t cb);


/**
 * @ingroup tce0
 * @brief Setter function for the TCE0 Capture-Compare0 callback.
 * @pre None.
 * @param CallbackHandler - Pointer to custom callback.
 * @return None.
 */
void TCE0_Compare0CallbackRegister(TCE0_cb_t cb);


/**
 * @ingroup tce0
 * @brief Setter function for the TCE0 Capture-Compare1 callback.
 * @pre None.
 * @param CallbackHandler - Pointer to custom callback.
 * @return None.
 */
void TCE0_Compare1CallbackRegister(TCE0_cb_t cb);


/**
 * @ingroup tce0
 * @brief Setter function for the TCE0 Capture-Compare2 callback.
 * @pre None.
 * @param CallbackHandler - Pointer to custom callback.
 * @return None.
 */
void TCE0_Compare2CallbackRegister(TCE0_cb_t cb);


/**
 * @ingroup tce0
 * @brief Setter function for the TCE0 Capture-Compare3 callback.
 * @pre None.
 * @param CallbackHandler - Pointer to custom callback.
 * @return None.
 */
void TCE0_Compare3CallbackRegister(TCE0_cb_t cb);


/**
 * @ingroup tce0
 * @brief Initializes the TCE0 module. This routine must be called before other TCE0 APIs.
 * @pre None.
 * @param None.
 * @return None.
 */
void TCE0_Initialize(void);


/**
 * @ingroup tce0
 * @brief Deinitializes the TCE0 module.
 * @pre None.
 * @param None.
 * @return None.
 */
 void TCE0_Deinitialize(void);


 /**
 * @ingroup tce0
 * @brief Starts the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TCE0_Start(void);


/**
 * @ingroup tce0
 * @brief Stops the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 *
 */
void TCE0_Stop(void);


 /**
 * @ingroup tce0
 * @brief Returns the status of the TCE0 module.
 * @param None.
 * @return enum TCE_status_t:
 *  - @c TCE_STATUS_IDLE = 0 @c      - The timer is not running
 *  - @c TCE_STATUS_RUNNING = 1 @c   - The timer is running
 */
TCE0_status_t TCE0_StatusGet(void);


/**
 * @ingroup tce0
 * @brief Sets the Wave mode for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param TCE_WGMODE_t mode:
 * - @c TCE_WGMODE_NORMAL_gc      @c  -  	Normal Mode
 * - @c TCE_WGMODE_FRQ_gc         @c  -     Frequency Generation Mode
 * - @c TCE_WGMODE_SINGLESLOPE_gc @c  -     Single Slope PWM
 * - @c TCE_WGMODE_DSTOP_gc       @c  -     Dual Slope PWM, overflow on TOP
 * - @c TCE_WGMODE_DSBOTH_gc      @c  -     Dual Slope PWM, overflow on TOP and BOTTOM
 * - @c TCE_WGMODE_DSBOTTOM_gc    @c  -     Dual Slope PWM, overflow on BOTTOM
 * @return None.
 *
 */
void TCE0_ModeSet(TCE_WGMODE_t mode);


/**
 * @ingroup tce0
 * @brief Enables the TCE0 OVF, CMP0, CMP1, CMP2 or, CMP3 interrupts.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TCE0_Interrupts_Enable(uint8_t value);


/**
 * @ingroup tce0
 * @brief Clears the interrupt flags for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint8_t value.
 * @return None.
 */
void TCE0_Interrupts_FlagsClear(uint8_t value);


/**
 * @ingroup tce0
 * @brief Retrieves the interrupt flag status.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return uint8_t:
 * - @c - Bit 0 is set by the OVF interrupt
 * - @c - Bit 4 is set by the CMP0 interrupt
 * - @c - Bit 5 is set by the the CMP1 interrupt
 * - @c - Bit 6 is set by the the CMP2 interrupt
 * - @c - Bit 7 is set by the the CMP3 interrupt
 */
uint8_t TCE0_Interrupts_FlagsGet(void);


/**
 * @ingroup tce0
 * @brief Disables the TCE0 OVF, CMP0, CMP1, CMP2 or, CMP3 interrupts.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TCE0_Interrupts_Disable(uint8_t value);


/**
 * @ingroup tce0
 * @brief Controls the Waveform mode on the event output line for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint8_t state.
 * @return None.
 */
void TCE0_Event_OutputMode(uint8_t value);


/**
 * @ingroup tce0
 * @brief Enables the input event for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint8_t value.
 * @return None.
 */
void TCE0_Event_InputConfig(uint8_t value);


/**
 * @ingroup tce0
 * @brief Enables software control of the Update, Restart and Reset states of the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param TCE_CMD_t enum:
 * - @c TCE_CMD_NONE_gc       - No Command
 * - @c TCE_CMD_UPDATE_gc     - Force Update
 * - @c TCE_CMD_RESTART_gc    - Force Restart
 * - @c TCE_CMD_RESET_gc      - Force Hard Reset
 * @return None.
 *
 */
void TCE0_SoftwareCommand(TCE_CMD_t com);


/**
 * @ingroup tce0
 * @brief Enables/disables the Run-in-Standby mode (RUNSTBY register) for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param bool state:
 * - @c True - Enables Sleep.
 * - @c False - Disables Sleep.
 * @return None.
 *
 */
void TCE0_StandBySleep(bool state);


/**
 * @ingroup tce0
 * @brief Enables Debug mode for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param bool state:
 * - @c True - TCE0 continues running in Break Debug mode when the CPU is halted;
 * - @c False - TCE0 is halted in Break Debug mode and ignores events.
 * @return None.
 *
 */
void TCE0_DebugRun(bool state);


/**
 * @ingroup tce0
 * @brief Returns the Count (CNT) register value for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return uint16_t - CNT register value for the TCE0.
 *
 */
uint16_t TCE0_CounterGet(void);


/**
 * @ingroup tce0
 * @brief Sets the Count (CNT) register value for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t- TCE0 value to be written to the CNT register.
 * @return None.
 */
void TCE0_CounterSet(uint16_t value);


/**
 * @ingroup tce0
 * @brief Selects the prescaler for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param TCE_CLKSEL_t prescaler
 * - @c -DIV1 - direct clock source
 * - @c -DIV2 - clock speed divided by 2
 * - @c -DIV4 - clock speed divided by 4
 * - @c -DIV8 - clock speed divided by 8
 * - @c -DIV16 - clock speed divided by 16
 * - @c -DIV64 - clock speed divided by 64
 * - @c -DIV256 - clock speed divided by 256
 * - @c -DIV1024 - clock speed divided by 1024
 * @return None.
*/
void TCE0_PrescalerSet(TCE_CLKSEL_t prescaler);


/**
 * @ingroup tce0
 * @brief Sets the Compare 0 (CMP0) register value for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - TCE0 value to be written to the CMP0 register.
 * @return None.
 */
void TCE0_Compare0Set (uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the Compare 1 (CMP1) register value for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - TCE0 value to be written to the CMP1 register.
 * @return None.
 */
void TCE0_Compare1Set (uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the Compare 2 (CMP2) register value for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - TCE0 value to be written to the CMP2 register.
 * @return None.
 */
void TCE0_Compare2Set(uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the Compare 3 (CMP3) register value for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - TCE0 value to be written to the CMP3 register.
 * @return None.
 */
void TCE0_Compare3Set(uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets all four CMP registers for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3, represents values to be written to the registers CMP0, CMP1, CMP2 and, CMP3
 * @return None.
 */
void TCE0_CompareAllChannelsSet(uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3);


/**
 * @ingroup tce0
 * @brief Sets the CMP0, CMP1 and, CMP2 registers for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value0, uint16_t value1, uint16_t value2, represents values to be written to the registers CMP0, CMP1 and, CMP2
 * @return None.
 */
void TCE0_CompareChannels012Set(uint16_t value0, uint16_t value1, uint16_t value2);


/**
 * @ingroup tce0
 * @brief Sets the CMP1, CMP2, and CMP3 registers for the TCE.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value1, uint16_t value2, uint16_t value3, represents values to be written to the registers CMP1, CMP2 and, CMP3
 * @return None.
 */
void TCE0_CompareChannels123Set(uint16_t value1, uint16_t value2, uint16_t value3);


/**
 * @ingroup tce0
 * @brief Sets all four Compare Buffer (CMPBUF) registers for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3, represents values to be written to the registers CMP0, CMP1, CMP2 and, CMP3
 * @return None.
 */
void TCE0_CompareAllChannelsBufferedSet(uint16_t value0, uint16_t value1, uint16_t value2, uint16_t value3);


/**
 * @ingroup tce0
 * @brief Sets the Compare 0 Buffer (CMP0BUF), Compare 1 Buffer (CMP1BUF) and, Compare 2 Buffer (CMP2BUF) registers for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value0, uint16_t value1, uint16_t value2, represents values to be written to the registers CMP0BUF, CMP1BUF and, CMP2BUF
 * @return None.
 */
void TCE0_CompareChannels012BufferedSet(uint16_t value0, uint16_t value1, uint16_t value2);


/**
 * @ingroup tce0
 * @brief Sets the Compare 1 Buffer (CMP1BUF), Compare 2 Buffer (CMP2BUF) and, Compare 3 Buffer (CMP3BUF) registers for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value0, uint16_t value1, uint16_t value2, represents values to be written to the registers CMP1BUF, CMP2BUF and, CMP3BUF
 * @return None.
 */
void TCE0_CompareChannels123BufferedSet(uint16_t value1, uint16_t value2, uint16_t value3);


/**
 * @ingroup tce0
 * @brief Enables the Waveform outputs for the four TCE0 channels.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint8_t - Bitmask value to enable the corresponding Compare Enable (CMPEN) bits in the Control B (CTRLB) register.
 * @return None.
 */
void TCE0_OutputsEnable(uint8_t value);


/**
 * @ingroup tce0
 * @brief Sets the output values for the four TCE0 channels.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint8_t - Bitmask corresponding to the channel that requires enabling or polarity inversion.
 * @return None.
 */
void TCE0_OutputsValueSet(uint8_t value);


/**
 * @ingroup tce0
 * @brief Returns the values of the polarities and outputs for the four TCE0 channels.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return uint8_t - The bits from 7 to 4 indicate the normal/inverted status of the polarities for channels 3 to 0. The bits from 0 to 3 indicate the set/clear status of the waveform output value for the channels from 3 to 0.
 */
uint8_t TCE0_OutputsValueGet(void);


/**
 * @ingroup tce0
 * @brief Sets the high resolution used for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param TCE_HREN_t resolution:
 * - @c TCE_HREN_OFF_gc = (0x00<<6) - High Resolution Disable
 * - @c TCE_HREN_4X_gc = (0x01<<6) - Resolution increased by 4 (2 bits)
 * - @c TCE_HREN_8X_gc = (0x02<<6) - Resolution increased by 4 (3 bits)
 * @return None.
 */
void TCE0_HighResSet(TCE_HREN_t resolution);


/**
 * @ingroup tce0
 * @brief Sets the Scaling mode used for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param TCE_SCALEMODE_t mode:
 * - @c TCE_SCALEMODE_CENTER_gc CMPn registers scaled vs center (50% duty cycle)
 * - @c TCE_SCALEMODE_BOTTOM_gc CMPn registers scaled vs BOTTOM (0% duty cycle)
 * - @c TCE_SCALEMODE_TOP_gc CMPn registers scaled vs TOP (100% duty cycle)
 * - @c TCE_SCALEMODE_TOPBOTTOM_gc CMPn registers scaled vs TOP or BOTTOM depending on written value
 * @return None.
 */
void TCE0_ScaleModeSet(TCE_SCALEMODE_t mode);


/**
 * @ingroup tce0
 * @brief Allows for fractional values to be written to the Compare (CMP), Amplitude (AMP) and, Compare Buffer (CMPBUF) registers for TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param bool state:
 * - @c True - Enable scaling.
 * - @c False - Disables scaling.
 * @return None.
 *
 */
void TCE0_ScaleEnable(bool state);


/**
 * @ingroup tce0
 * @brief Allows the use of the AMP and OFFSET registers for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API. The SCALE bit needs to be set to "1".
 * @param bool state:
 * - @c True - enable amplitude control
 * - @c False - disable amplitude control.
 * @return None.
 *
 */
void TCE0_AmplitudeControlEnable(bool state);


/**
 * @ingroup tce0
 * @brief Sets the AMP register of the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value, represents the value to be written in the AMP register.
 * @return None.
 */
void TCE0_AmplitudeSet(uint16_t value);


/**
 * @ingroup tce0
 * @brief Returns the value of the AMP register of the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return uint16_t, represents the value in the AMP register.
 * @n @b NOTE: the value read may be different from the one that was previously written in the AMP register.
 */
uint16_t TCE0_AmplitudeGet(void);


/**
 * @ingroup tce0
 * @brief Sets the OFFSET register of the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - Value to be written in the OFFSET register.
 * @return None.
 */
void TCE0_OffsetSet(uint16_t value);


/**
 * @ingroup tce0
 * @brief Returns the value of the OFFSET register of the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return uint16_t - Value in the OFFSET register.
 *
 */
uint16_t TCE0_OffsetGet(void);


/**
 * @ingroup tce0
 * @brief Sets the Period (PER) register for the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - Value to be written in the PER register.
 * @return None.
 */
void TCE0_PeriodSet(uint16_t period);


/**
 * @ingroup tce0
 * @brief Sets the TCE0 to count UP.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TCE0_CountDirectionSet(void);


/**
 * @ingroup tce0
 * @brief Sets the TCE0 to count DOWN.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 *
 */
void TCE0_CountDirectionClear(void);


/**
 * @ingroup tce0
 * @brief Sets the Lock Update (LUPD) bit for the TCE0 to 1. This ensures there will be no update on the buffered registers.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 *
 */
void TCE0_LockUpdateSet(void);


/**
 * @ingroup tce0
 * @brief Clears the Lock Update bit for the TCE0. This ensures that the buffered registers are updated when an UPDATE condition has occurred.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 *
 */
void TCE0_LockUpdateClear(void);


/**
 * @ingroup tce0
 * @brief Sets the Auto-Lock Update (ALUPD) bit for the TCE0 to 1. This ensures that the Lock Update (LUPD) bit in the TCEn.CTRLE register is set and cleared automatically.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 *
 */
void TCE0_AutoLockUpdateSet(void);


/**
 * @ingroup tce0
 * @brief Clears the Auto-Lock Update bit for the TCE0. This ensures that the LUPD bit in the TCEn.CTRLE register is not altered by the system.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param None.
 * @return None.
 *
 */
void TCE0_AutoLockUpdateClear(void);


/**
 * @ingroup tce0
 * @brief Sets the value of the CMP0BUF register used in the Pulse-Width Modulation (PWM) mode by the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value - Value to be written in the CMP0BUF register.
 * @return None.
 *
 */
void TCE0_PWM_BufferedDutyCycle0Set(uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the value of the CMP1BUF register used in PWM mode by the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value - Value to be written in the CMP1BUF register.
 * @return None.
 *
 */
void TCE0_PWM_BufferedDutyCycle1Set(uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the value of the CMP2BUF register used in PWM mode by the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value - Value to be written in the CMP2BUF register.
 * @return None.
 *
 */
void TCE0_PWM_BufferedDutyCycle2Set(uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the value of the CMP3BUF register used in PWM mode by the TCE0.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t value - Value to be written in the CMP3BUF register.
 * @return None.
 *
 */
void TCE0_PWM_BufferedDutyCycle3Set(uint16_t value);


/**
 * @ingroup tce0
 * @brief Sets the value of the Period Buffer (PERBUF) register that serves as the buffer for the period register TCE0.PER.
 * @pre Initialize the TCE0 with TCE0_Initialize() before calling this API.
 * @param uint16_t - Value to be written in the PERBUF register.
 * @return None.
 *
 */
void TCE0_PeriodBufferSet(uint16_t value);


#endif // TCE0_H

/*******************************************************************************
 End of File
*/

