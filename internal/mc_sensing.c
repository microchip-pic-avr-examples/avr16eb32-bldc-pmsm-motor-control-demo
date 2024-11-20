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

#include <string.h>
#include "mc_pins.h"
#include "mc_config.h"
#include "mc_sensing.h"
#include "mc_comparator.h"
#include "mc_sm.h"

#define HALL_READ(PORT, PIN)  (((PORT.IN & PIN) == 0) ? true:false)

static uint8_t        hall_table[8];
static mc_sense_t     sensing_state;
static uint8_t        bemf_current_sector, bemf_current_adjustment;

static inline void MC_Bemf_Initialize(void)
{
    MC_Comparator_Initialize();
    MC_Comparator_MuxSet(MC_PHASE_FLOAT_A);
}

static inline void MC_Hall_Initialize(void)
{
    HA_PORT.DIRCLR = HA_PIN;
    HB_PORT.DIRCLR = HB_PIN;
    HC_PORT.DIRCLR = HC_PIN;
}

void MC_Sensing_Initialize(void)
{
    sensing_state.byte = 0;
    MC_Hall_Initialize();
    MC_Bemf_Initialize();
    memset((void *)hall_table, 0, sizeof(hall_table));
}

mc_sense_t MC_Hall_IntGet(void)
{
    uint8_t event = 0;
    if(HALL_READ(HA_PORT,HA_PIN)) event |= 1;
    if(HALL_READ(HB_PORT,HB_PIN)) event |= 2;
    if(HALL_READ(HC_PORT,HC_PIN)) event |= 4;

    mc_sense_t retval;
    retval.byte = 0;

    uint8_t index = hall_table[event & 7];
    if(index != 0)
    {
        index++;
        retval.id = index;
    }
    retval.state = event;
    sensing_state = retval;
    return retval;
}

SMTD_STATES_BEGIN(BEMF)
SMTD_STATES_DECLARE(INIT0)
SMTD_STATES_DECLARE(PRE_0)
SMTD_STATES_DECLARE(WAIT0)
SMTD_STATES_DECLARE(END_0)
SMTD_STATES_DECLARE(INIT1)
SMTD_STATES_DECLARE(PRE_1)
SMTD_STATES_DECLARE(WAIT1)
SMTD_STATES_DECLARE(END_1)
SMTD_STATES_END(BEMF)

SMTD_EVENTS_BEGIN(BEMF)
SMTD_EVENTS_DECLARE(SCG_0) /* sector change 0 */
SMTD_EVENTS_DECLARE(SCG_1) /* sector change 1 */
SMTD_EVENTS_DECLARE(CMP_0) /* comparator 0    */
SMTD_EVENTS_DECLARE(CMP_1) /* comparator 1    */
SMTD_EVENTS_END(BEMF)

enum
{
    NONE = 0,           /* nothing */
    LEAD = 1,           /* bemf too early */
    BEMF = 1,           /* bemf normal */
    LAGG = (uint8_t)-1, /* bemf too late */
};

SMTD_TRANS_TABLE_BEGIN(BEMF, uint8_t)
                             /* events: SCG_0,      SCG_1,      CMP_0,      CMP_1        states: */   
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,NONE, INIT1,NONE, WAIT0,NONE, PRE_0,NONE) /* INIT0 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,NONE, INIT1,LEAD, WAIT0,NONE, PRE_0,NONE) /* PRE_0 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,NONE, INIT1,LAGG, WAIT0,NONE, END_0,BEMF) /* WAIT0 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,NONE, INIT1,NONE, END_0,NONE, END_0,BEMF) /* END_0 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,NONE, INIT1,NONE, PRE_1,NONE, WAIT1,NONE) /* INIT1 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,LEAD, INIT1,NONE, PRE_1,NONE, WAIT1,NONE) /* PRE_1 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,LAGG, INIT1,NONE, END_1,BEMF, WAIT1,NONE) /* WAIT1 */
SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(INIT0,NONE, INIT1,NONE, END_1,BEMF, END_1,NONE) /* END_1 */
SMTD_TRANS_TABLE_END()

SMTD_DEFINE(BEMF, INIT0, uint8_t)


mc_sense_t MC_Bemf_IntGet(void)
{
    mc_sense_t retval;
    retval.byte = 0;
    uint8_t local_adj;
    if(MC_Comparator_Get())
    {
        local_adj = SMTD_HANDLER_CALL(BEMF, CMP_1);
        retval.state = 1;
    }
    else
    {
        local_adj = SMTD_HANDLER_CALL(BEMF, CMP_0);
        retval.state = 0;
    }
    
    retval.id = 1 + bemf_current_sector + bemf_current_adjustment + local_adj;
    sensing_state = retval;
    return retval;
}

void MC_Bemf_Set(uint8_t sector, uint8_t mux)
{
    MC_Comparator_MuxSet(mux);
    bemf_current_sector = sector;
    if(bemf_current_sector & 1)
        bemf_current_adjustment = SMTD_HANDLER_CALL(BEMF, SCG_0);
    else
        bemf_current_adjustment = SMTD_HANDLER_CALL(BEMF, SCG_1);
}


mc_sense_t MC_Sensing_Get(void)
{
    return sensing_state;
}

uint8_t *MC_Sensing_HallTable_Get(void)
{
    return hall_table;
}
