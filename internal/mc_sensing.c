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

#include "mc_pins.h"
#include "mc_config.h"
#include "mc_sensing.h"
#include "mc_comparator.h"
#include "mc_sm.h"



#define HALL_READ(PORT, PIN)  (((PORT.IN & PIN) == 0) ? true:false)

static mc_sense_t   sensing_state;
static uint8_t      bemf_current_sector, bemf_current_adjustment;

static inline void MC_Bemf_Initialize(void)
{
    MC_Comparator_Initialize();
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
}

SMT_STATES_BEGIN(HALL)
SMT_STATES_DECLARE(ST0)
SMT_STATES_DECLARE(ST1)
SMT_STATES_DECLARE(ST2)
SMT_STATES_DECLARE(ST3)
SMT_STATES_DECLARE(ST4)
SMT_STATES_DECLARE(ST5)
SMT_STATES_END(HALL)

SMT_EVENTS_BEGIN(HALL)
SMT_EVENTS_DECLARE(H0)
SMT_EVENTS_DECLARE(H1)
SMT_EVENTS_DECLARE(H2)
SMT_EVENTS_DECLARE(H3)
SMT_EVENTS_DECLARE(H4)
SMT_EVENTS_DECLARE(H5)
SMT_EVENTS_DECLARE(H6)
SMT_EVENTS_DECLARE(H7)
SMT_EVENTS_END(HALL)


SMT_TRANS_TABLE_BEGIN(HALL) /* events: H0,  H1,  H2,  H3,  H4,  H5,  H6,  H7   states: */   
SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(ST0, ST0, ST0, ST0, ST0, ST1, ST0, ST0)  /* ST0 */
SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(ST0, ST2, ST1, ST1, ST2, ST1, ST1, ST0)  /* ST1 */
SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(ST0, ST2, ST2, ST3, ST2, ST2, ST3, ST0)  /* ST2 */
SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(ST0, ST3, ST4, ST3, ST3, ST3, ST3, ST0)  /* ST3 */
SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(ST0, ST4, ST4, ST5, ST4, ST4, ST5, ST0)  /* ST4 */
SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(ST0, ST0, ST5, ST5, ST0, ST5, ST5, ST0)  /* ST5 */
SMT_TRANS_TABLE_END()

SMT_DEFINE(HALL, ST0)


mc_sense_t MC_Hall_IntGet(void)
{
#if MOTOR_HALL_INVERTED == false
    uint8_t event = 0;
    if(HALL_READ(HA_PORT,HA_PIN)) event |= 1;
    if(HALL_READ(HB_PORT,HB_PIN)) event |= 2;
    if(HALL_READ(HC_PORT,HC_PIN)) event |= 4;
#else /* MOTOR_HALL_INVERTED */
    uint8_t event = 7;
    if(HALL_READ(HA_PORT,HA_PIN)) event &= ~1;
    if(HALL_READ(HB_PORT,HB_PIN)) event &= ~2;
    if(HALL_READ(HC_PORT,HC_PIN)) event &= ~4;
#endif /* MOTOR_HALL_INVERTED */
    mc_sense_t retval;
    retval.id = 4 + SMT_HANDLER_CALL(HALL, event);
    retval.state = event;
    sensing_state = retval;
    return retval;
}

mc_sense_t MC_Hall_Get(void)
{
    return sensing_state;
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
SMTD_EVENTS_DECLARE(SCG_0) // sector change 0
SMTD_EVENTS_DECLARE(SCG_1) // sector change 1
SMTD_EVENTS_DECLARE(CMP_0) // comparator 0
SMTD_EVENTS_DECLARE(CMP_1) // comparator 1
SMTD_EVENTS_END(BEMF)

enum
{
    NONE = 0,           // nothing
    LEAD = 1,           // bemf too early
    BEMF = 1,           // bemf normal
    LAGG = (uint8_t)-1, // bemf too late
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
        local_adj = SMTD_HANDLER_CALL(BEMF, CMP_1);
    else
        local_adj = SMTD_HANDLER_CALL(BEMF, CMP_0);
    retval.id = bemf_current_sector + bemf_current_adjustment + local_adj;
    sensing_state = retval;
    return retval;
}

mc_sense_t MC_Bemf_Get(void)
{
    return sensing_state;
}


void MC_Bemf_Set(uint8_t data)
{
    uint8_t step = data & (MC_PHASE_FLOAT_A | MC_PHASE_FLOAT_B | MC_PHASE_FLOAT_C);
    MC_Comparator_MuxSet(step);
    bemf_current_sector = 7 &  (data >> 4);
    if(bemf_current_sector & 1)
        bemf_current_adjustment = SMTD_HANDLER_CALL(BEMF, SCG_0);
    else
        bemf_current_adjustment = SMTD_HANDLER_CALL(BEMF, SCG_1);
}

