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

#ifndef WEX0_H
#define WEX0_H

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

/**
 * @ingroup wex0
 * @brief Defines the macro associated with the conversion from ns (nanoseconds) to clock increments. @n Used for setting the dead time or blanking time. @n The user must specify the desired value in ns (nanoseconds), the clock speed and the prescaler.
 * @brief The macro arguments:
 * - VALUE must be a constant number rather than a variable
 * - F_CLOCK is the WEX peripheral clock
 * - WEX_PRESCALER can be 1, 2, 4, 8, 16, 64, 256, 1024
 */
#define WEX0_NS_TO_TICKS(VALUE, F_CLOCK, WEX_PRESCALER) (uint8_t)( (float)(VALUE) / ( (1000000000.0 / (F_CLOCK) ) * (WEX_PRESCALER) ) + 0.5)

/**
 * @ingroup wex0
 * @typedef void * WEX0_cb_t
 * @brief This is the function pointer that accesses the callback function, whenever an interrupt occurs or a fault is detected.
 */
typedef void (* WEX0_cb_t)(void);


/**
 * @ingroup wex0
 * @brief Function used to register the ISR callback function.
 * @pre None.
 * @param WEX0_cb_t cb - a function pointer.
 * @return None.
 */
void WEX0_FAULTIsrCallbackRegister(WEX0_cb_t cb);


/**
 * @ingroup wex0
 * @brief Initializes the WEX0 module.
 * @pre None.
 * @param None.
 * @return None.
 */
void WEX0_Initialize(void);


/**
 * @ingroup wex0
 * @brief Deinitializes the WEX0 module.
 * @pre None.
 * @param None.
 * @return None.
 */
void WEX0_Deinitialize(void);


/**
 * @ingroup wex0
 * @brief Used to get the status of the WEX0's blanking/fault detection.
 * @pre None.
 * @param None.
 * @return uint8_t - the WEX0's running status.
 */
uint8_t WEX0_StatusRegisterGet(void);


/**
 * @ingroup wex0
 * @brief Sets the Pattern Generation mode of the WEX0 interface.
 * @pre None.
 * @param bool mode - boolean that has value true if enabled, false if disabled.
 * @return None.
 */
void WEX0_PatternGenerationMode(bool mode);


/**
 * @ingroup wex0
 * @brief Checks the Pattern Generation mode status of the WEX0 interface.
 * @pre None.
 * @param None.
 * @return bool - true if enabled, false if disabled.
 */
bool WEX0_IsPatternGenerationSet(void);


/**
 * @ingroup wex0
 * @brief Selects the Input Matrix mode of the WEX0 interface.
 * @pre None.
 * @param WEX_INMX_t config - enum that specifies the routing matrix for the waveform generation outputs to the port pins.
 * @return None.
 */
void WEX0_InputMatrixSet(WEX_INMX_t config);


/**
 * @ingroup wex0
 * @brief Used to enable the dead time insertion on each of the waveform output channels.
 * @pre None.
 * @param uint8_t channels - represents the channels that will have the dead time insertion enabled.
 * @return None.
 */
void WEX0_DeadTimeInsertionSet(uint8_t channels);


/**
 * @ingroup wex0
 * @brief Selects the update source for the WEX0 peripheral. This can be a PWM update or a condition where no hardware update is required.
 * @pre None.
 * @param WEX_UPDSRC_t config - enum that specifies the trigger source for the update condition.
 * @return None.
 */
void WEX0_UpdateSourceSet(WEX_UPDSRC_t config);

/**
 * @ingroup wex0
 * @brief  Gives software commands to the WEX0 peripheral. Commands can be: None, Update, FaultSet, FaultCLR, BlankSet, BlankCLR.
 * @pre None.
 * @param WEX_CMD_t com - enum that specify software commands used by WEX0 peripheral.
 * @return None.
 */
void WEX0_SoftwareCommand(WEX_CMD_t command);

/**
 * @ingroup wex0
 * @brief Filters event input A of the WEX0 peripheral.
 * @pre None.
 * @param WEX_FILTER_t samples - enum that represents the number of samples filtered for an event to go through, with a range between 0 and 7 samples.
 * @return None.
 */
void WEX0_FaultAEventFilter(WEX_FILTER_t samples);


/**
 * @ingroup wex0
 * @brief Enables fault blanking time on event A for the WEX0 peripheral.
 * @pre None.
 * @param bool en - boolean value is true if enabled, and false if disabled.
 * @return None.
 */
void WEX0_FaultAEventBlankingEnable(bool en);


/**
 * @ingroup wex0
 * @brief Enables event input A for the WEX0 peripheral.
 * @pre None.
 * @param bool en - boolean that has value true if enabled, false if disabled.
 * @return None.
 */
void WEX0_FaultAEventInputEnable(bool en);


/**
 * @ingroup wex0
 * @brief Filters event input B of the WEX0 peripheral.
 * @pre None.
 * @param WEX_FILTER_t samples - enum that represents the number of samples filtered for an event to go through, with a range between 0 and 7 samples.
 * @return None.
 */
void WEX0_FaultBEventFilter(WEX_FILTER_t samples);


/**
 * @ingroup wex0
 * @brief Enables fault blanking time on event B for the WEX0 peripheral.
 * @pre None.
 * @param bool en - boolean value is true if enabled, and false if disabled.
 * @return None.
 */
void WEX0_FaultBEventBlankingEnable(bool en);


/**
 * @ingroup wex0
 * @brief Enables event input B for the WEX0 peripheral.
 * @pre None.
 * @param bool en - boolean that has value true if enabled, false if disabled.
 * @return None.
 */
void WEX0_FaultBEventInputEnable(bool en);


/**
 * @ingroup wex0
 * @brief Filters event input C of the WEX0 peripheral.
 * @pre None.
 * @param WEX_FILTER_t samples - enum that represents the number of samples filtered for an event to go through, with a range between 0 and 7 samples.
 * @return None.
 */
void WEX0_FaultCEventFilter(WEX_FILTER_t samples);


/**
 * @ingroup wex0
 * @brief Enables fault blanking time on event C for the WEX0 peripheral.
 * @pre None.
 * @param bool en - boolean value is true if enabled, and false if disabled.
 * @return None.
 */
void WEX0_FaultCEventBlankingEnable(bool en);


/**
 * @ingroup wex0
 * @brief Enables event input C for the WEX0 peripheral.
 * @pre None.
 * @param bool en - boolean that has value true if enabled, false if disabled.
 * @return None.
 */
void WEX0_FaultCEventInputEnable(bool en);


/**
 * @ingroup wex0
 * @brief Sets the blanking prescaler for the WEX0 peripheral.
 * @pre None.
 * @param WEX_BLANKPRESC_t pre - enum that specifies the value of the prescaler, which can be 1, 4, 16 or 64.
 * @return None.
 */
void WEX0_BlankingPrescaler(WEX_BLANKPRESC_t prescaler);


/**
 * @ingroup wex0
 * @brief  Sets the blanking trigger command for the WEX0 peripheral. It can be software of hardware.
 * @pre None.
 * @param WEX_BLANKTRIG_t trig - enum that specifies the blanking trigger source, which can be on timer update condition, on compare channels match, or software.
 * @return None.
 */
void WEX0_BlankingTrigger(WEX_BLANKTRIG_t trig);


/**
 * @ingroup wex0
 * @brief  Sets the blanking time for the WEX0 peripheral. If prescaler is used this time can be extended.
 * @pre None.
 * @param uint8_t cnt - uint8_t that represents the blanking time given in a number of clock ticks, with a range between 0 and 255.
 * @return None.
 */
void WEX0_BlankingTimeSet(uint8_t cnt);


/**
 * @ingroup wex0
 * @brief  Gets the blanking time for the WEX0 peripheral.
 * @pre None.
 * @param None.
 * @return uint8_t - the blanking time that was previously set.
 */
uint8_t WEX0_BlankingTimeGet(void);


/**
 * @ingroup wex0
 * @brief  Sets fault detect on debug break detect for the WEX0 peripheral.
 * @pre None.
 * @param WEX_FDDBD_t mode - enum that specifies the fault detection on debug break detection.
 * @return None.
 */
void WEX0_FaultDetectionDebugBreak(WEX_FDDBD_t mode);


/**
 * @ingroup wex0
 * @brief  Resets the WEX0 peripheral from Fault mode.
 * @pre None.
 * @param WEX_FDMODE_t mode - enum that specifies the Restart mode from fault, it can be latched or cycle-by-cycle.
 * @return None.
 */
void WEX0_FaultDetectionRestartMode(WEX_FDMODE_t mode);


/**
 * @ingroup wex0
 * @brief  Selects the action to be taken when a fault detect occurs.
 * @pre None.
 * @param WEX_FDACT_t act - enum that specifies how are the output signals driven in case of a fault.
 * @return None.
 */
void WEX0_FaultDetectionAction(WEX_FDACT_t action);


/**
 * @ingroup wex0
 * @brief  Selects if an output signal is in tri-state or overwritten by the custom fault value defined by the user in case a fault is detected.
 * @pre None.
 * @param uint8_t channels - represents which of the 8 waveform signals are affected by fault drive, the bits between 0 and 7 have a one-to-one relationship with the waveform signals from 0 to 7.
 * @return None.
 */
void WEX0_FaultDriveSet(uint8_t channels);


/**
 * @ingroup wex0
 * @brief  Sets the output values of the signals in case of a fault detection.
 * @pre None.
 * @param uint8_t channels - represents which value will have each of the waveform signals if the custom drive option is selected,the bits between 0 and 7 have a one-to-one relationship with the waveform signals from 0 to 7 .
 * @return None.
 */
void WEX0_FaultOutputSet(uint8_t channels);

/**
 * @ingroup wex0
 * @brief  Enables fault interrupt on the WEX0 interface.
 * @pre None.
 * @param None.
 * @return None.
 */
void WEX0_FaultEnable(void);


/**
 * @ingroup wex0
 * @brief  Checks if fault interrupt is enabled on the WEX0 interface.
 * @pre None.
 * @param None.
 * @return bool - true if enabled, false if disabled.
 */
bool WEX0_IsFaultEnabled(void);


/**
 * @ingroup wex0
 * @brief  Disables fault interrupt on the WEX0 interface.
 * @pre None.
 * @param None.
 * @return None.
 */
void WEX0_FaultDisable(void);


/**
 * @ingroup wex0
 * @brief  Clears interrupt flags on the WEX0 interface.
 * @pre None.
 * @param flags - uint8_t that represents the 4 fault flags, which are the fault interrupt flags, and the fault event input detection flags .
 * @return None.
 */
void WEX0_FaultFlagsClear(uint8_t flags);


/**
 * @ingroup wex0
 * @brief  Checks which interrupt flags are set on the WEX0 interface.
 * @pre None.
 * @param None.
 * @return uint8_t - using only the bits 0, 2, 3 and 4, the bit 0 is the fault interrupt flag, the bit 2 is event input A flag, the bit 3 is event input B flag, and the bit 4 is event input C flag.
 */
uint8_t WEX0_FaultFlagsGet(void);


/**
 * @ingroup wex0
 * @brief  Sets the dead time on the low side of the output signals on the WEX0 interface.
 * @pre None.
 * @param uint8_t cnt - represents the value of the dead time on the low side, with a range between 0 and 255.
 * @return None.
 */
void WEX0_DeadTimeLowSideSet(uint8_t cnt);


/**
 * @ingroup wex0
 * @brief  Sets the dead time on the high side of the output signals on the WEX0 interface.
 * @pre None.
 * @param uint8_t cnt - represents the value of the dead time on the high side, with a range between 0 and 255.
 * @return None.
 */
void WEX0_DeadTimeHighSideSet(uint8_t cnt);


/**
 * @ingroup wex0
 * @brief  Sets the dead time on both sides of the side output signals on the WEX0 interface.
 * @pre None.
 * @param uint8_t cnt - represents the value of the dead time on both sides, with a range between 0 and 255.
 * @return None.
 */
void WEX0_DeadTimeBothSidesSet(uint8_t cnt);


/**
 * @ingroup wex0
 * @brief  Swaps the high-side and low side output signals from a complementary waveform pair of signals on the WEX0 interface.
 * @pre None.
 * @param uint8_t channels - specifies the desired channels to swap their low side and high-side signals, the first 0-3 bits are used and they have a one-to-one relatioship with the 0-3 channels.
 * @return None.
 */
void WEX0_SwapChannelSet(uint8_t channels);


/**
 * @ingroup wex0
 * @brief  Selects which output waveform signals can be overridden in Pattern Generation mode on the WEX0 interface.
 * @pre None.
 * @param uint8_t channels - represents which of the 8 waveform signals can be enabled to be overriden, the bits between 0 and 7 have a one-to-one relationship with the waveform output signals from 0 to 7.
 * @return None.
 */
void WEX0_PatternGenerationOverrideSet(uint8_t channels);


/**
 * @ingroup wex0
 * @brief  Sets the output values of the signals in Pattern Generation mode.
 * @pre None.
 * @param uint8_t channels - specifies the value each waveform output signal will have, the bits between 0 and 7 have a one-to-one relationship with the waveform output signals from 0 to 7.
 * @return None.
 */
void WEX0_PatternGenerationOutputSet(uint8_t channels);


/**
 * @ingroup wex0
 * @brief  Enables output signals and allows them to be overridden.
 * @pre None.
 * @param uint8_t channels - specifies which of the output signals can be overridden, can change their state, the bits between 0 and 7 have a one-to-one relationship with the waveform output signals from 0 to 7.
 * @return None.
 */
void WEX0_OutputOverrideEnable(uint8_t channels);


/**
 * @ingroup wex0
 * @brief  Sets the dead time on the low side of the output signals using a buffer register on the WEX0 interface.
 * @pre None.
 * @param uint8_t buff - specifies the value of the dead time inserted on the low side of the output signals using a buffer, given in clock ticks.
 * @return None.
 */
void WEX0_DeadTimeLowSideBufferSet(uint8_t buff);


/**
 * @ingroup wex0
 * @brief Sets the dead time on the high side of the output signals using a buffer register on the WEX0 interface.
 * @pre None.
 * @param uint8_t buff - specifies the value of the dead time inserted on the high side of the output signals using a buffer, given in clock ticks.
 * @return None.
 */
void WEX0_DeadTimeHighSideBufferSet(uint8_t buff);


/**
 * @ingroup wex0
 * @brief  Sets the dead time on both sides of the output signals using a buffer register on the WEX0 interface.
 * @pre None.
 * @param uint8_t buff - specifies the value of the dead time inserted on both sides of the output signals using a buffer given in clock ticks.
 * @return None.
 */
void WEX0_DeadTimeBothSidesBufferSet(uint8_t buff);


/**
 * @ingroup wex0
 * @brief  Sets the swap buffer of the output signal pairs on the WEX0 interface.
 * @pre None.
 * @param uint8_t buff - specifies the desired channels to swap their low side and high side signals, the first 0-3 bits are used and they have a one-to-one relatioship with the 0-3 channels.
 * @return None.
 */
void WEX0_SwapChannelBufferSet(uint8_t buff);


/**
 * @ingroup wex0
 * @brief  Sets the Pattern Generation mode override buffer on the WEX0 interface.
 * @pre None.
 * @param uint8_t buff - represents which of the 8 waveform signals can be enabled to be overriden, the bits between 0 and 7 have a one-to-one relationship with the waveform output signals from 0 to 7.
 * @return None.
 */
void WEX0_PatternGenerationOverrideBufferSet(uint8_t buff);


/**
 * @ingroup wex0
 * @brief  Sets the Pattern Generation mode output values buffer on the WEX0 interface.
 * @pre None.
 * @param uint8_t buff - specifies the value each waveform output signal will have, the bits between 0 and 7 have a one-to-one relationship with the waveform output signals from 0 to 7.
 * @return None.
 */
void WEX0_PatternGenerationOutputBufferSet(uint8_t buff);

#endif /* WEX0_H */
