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

#ifndef STATE_MACHINES_H
#define STATE_MACHINES_H

/* do not use macros from different approaches with the same instance */

/* APPROACH 1 - SMT - state machine using table - not so useful, only for concept */
#define SMT_STATES_BEGIN(SM_INSTANCE)   typedef enum SM_INSTANCE##_state_enum {
#define SMT_STATES_DECLARE(STATE)       STATE,
#define SMT_STATES_END(SM_INSTANCE)     SM_INSTANCE##_MAX_STATES} SM_INSTANCE##_state_t;
#define SMT_STATES_TYPE(SM_INSTANCE)    SM_INSTANCE##_state_t

#define SMT_EVENTS_BEGIN(SM_INSTANCE)   typedef enum SM_INSTANCE##_event_enum {
#define SMT_EVENTS_DECLARE(EVENT)       EVENT,
#define SMT_EVENTS_END(SM_INSTANCE)     SM_INSTANCE##_MAX_EVENTS} SM_INSTANCE##_event_t;
#define SMT_EVENTS_TYPE(SM_INSTANCE)    SM_INSTANCE##_event_t

#define SMT_TRANS_TABLE_BEGIN(SM_INSTANCE)   static const SM_INSTANCE##_state_t SM_INSTANCE##_trans_table[SM_INSTANCE##_MAX_STATES][SM_INSTANCE##_MAX_EVENTS]={
#define SMT_TRANS_TABLE_2_EVENTS_TO_NEW_STATES(STATE_EV_0, STATE_EV_1)                                                   {STATE_EV_0, STATE_EV_1},
#define SMT_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(STATE_EV_0, STATE_EV_1, STATE_EV_2)                                       {STATE_EV_0, STATE_EV_1, STATE_EV_2},
#define SMT_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3)                           {STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3},
#define SMT_TRANS_TABLE_5_EVENTS_TO_NEW_STATES(STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3, STATE_EV_4)               {STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3, STATE_EV_4},
#define SMT_TRANS_TABLE_6_EVENTS_TO_NEW_STATES(STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3, STATE_EV_4, STATE_EV_5)   {STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3, STATE_EV_4, STATE_EV_5},
#define SMT_TRANS_TABLE_8_EVENTS_TO_NEW_STATES(STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3, STATE_EV_4, STATE_EV_5, STATE_EV_6, STATE_EV_7)\
                                                                                                                         {STATE_EV_0, STATE_EV_1, STATE_EV_2, STATE_EV_3, STATE_EV_4, STATE_EV_5, STATE_EV_6, STATE_EV_7},
#define SMT_TRANS_TABLE_END()                };

#define SMT_DEFINE(SM_INSTANCE, INITIAL_STATE)          static SM_INSTANCE##_state_t SM_INSTANCE##_state_var = (INITIAL_STATE);\
                                                        static inline SM_INSTANCE##_state_t SM_INSTANCE##_TransitionFunction(SM_INSTANCE##_event_t _event)\
                                                        {SM_INSTANCE##_state_var = SM_INSTANCE##_trans_table[SM_INSTANCE##_state_var][_event]; return SM_INSTANCE##_state_var;}
#define SMT_HANDLER_CALL(SM_INSTANCE, EVENT)            SM_INSTANCE##_TransitionFunction(EVENT)
#define SMT_STATE_GET(SM_INSTANCE)                      SM_INSTANCE##_state_var



/* do not use macros from different approaches with the same instance */

/* APPROACH 2 - SMTD - state machine using table and data */
#define SMTD_STATES_BEGIN(SM_INSTANCE)   typedef enum SM_INSTANCE##_state_enum {
#define SMTD_STATES_DECLARE(STATE)       STATE,
#define SMTD_STATES_END(SM_INSTANCE)     SM_INSTANCE##_MAX_STATES} SM_INSTANCE##_state_t;
#define SMTD_STATES_TYPE(SM_INSTANCE)    SM_INSTANCE##_state_t

#define SMTD_EVENTS_BEGIN(SM_INSTANCE)   typedef enum SM_INSTANCE##_event_enum {
#define SMTD_EVENTS_DECLARE(EVENT)       EVENT,
#define SMTD_EVENTS_END(SM_INSTANCE)     SM_INSTANCE##_MAX_EVENTS} SM_INSTANCE##_event_t;
#define SMTD_EVENTS_TYPE(SM_INSTANCE)    SM_INSTANCE##_event_t

#define SMTD_TRANS_TABLE_BEGIN(SM_INSTANCE, DTYPE)     typedef struct{SM_INSTANCE##_state_t state; DTYPE data;} SM_INSTANCE##_data_t; static const SM_INSTANCE##_data_t SM_INSTANCE##_trans_table[SM_INSTANCE##_MAX_STATES][SM_INSTANCE##_MAX_EVENTS]={
#define SMTD_TRANS_TABLE_2_EVENTS_TO_NEW_STATES(ST_EV_0, D0, ST_EV_1, D1)                                                      { {ST_EV_0,D0}, {ST_EV_1,D1} },
#define SMTD_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(ST_EV_0, D0, ST_EV_1, D1, ST_EV_2, D2)                                         { {ST_EV_0,D0}, {ST_EV_1,D1}, {ST_EV_2,D2} },
#define SMTD_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(ST_EV_0, D0, ST_EV_1, D1, ST_EV_2, D2, ST_EV_3, D3)                            { {ST_EV_0,D0}, {ST_EV_1,D1}, {ST_EV_2,D2}, {ST_EV_3,D3} },
#define SMTD_TRANS_TABLE_5_EVENTS_TO_NEW_STATES(ST_EV_0, D0, ST_EV_1, D1, ST_EV_2, D2, ST_EV_3, D3, ST_EV_4, D4)               { {ST_EV_0,D0}, {ST_EV_1,D1}, {ST_EV_2,D2}, {ST_EV_3,D3}, {ST_EV_4,D4} },
#define SMTD_TRANS_TABLE_6_EVENTS_TO_NEW_STATES(ST_EV_0, D0, ST_EV_1, D1, ST_EV_2, D2, ST_EV_3, D3, ST_EV_4, D4, ST_EV_5, D5)  { {ST_EV_0,D0}, {ST_EV_1,D1}, {ST_EV_2,D2}, {ST_EV_3,D3}, {ST_EV_4,D4}, {ST_EV_5,D5} },
#define SMTD_TRANS_TABLE_END()                         };

#define SMTD_DEFINE(SM_INSTANCE, INITIAL_STATE, DTYPE)   static SM_INSTANCE##_state_t SM_INSTANCE##_state_var = (INITIAL_STATE);\
                                                         static inline DTYPE SM_INSTANCE##_TransitionFunction(SM_INSTANCE##_event_t _event)\
                                                         {SM_INSTANCE##_data_t _data = SM_INSTANCE##_trans_table[SM_INSTANCE##_state_var][_event];\
                                                         SM_INSTANCE##_state_var = _data.state; return _data.data;}
#define SMTD_HANDLER_CALL(SM_INSTANCE, EVENT)            SM_INSTANCE##_TransitionFunction(EVENT)
#define SMTD_STATE_GET(SM_INSTANCE)                      SM_INSTANCE##_state_var



/* do not use macros from different approaches with the same instance */

/* APPROACH 3 - SMTF - state machine using table of function */
#define SMTF_STATES_BEGIN(SM_INSTANCE)   typedef enum SM_INSTANCE##_state_enum {
#define SMTF_STATES_DECLARE(STATE)       STATE,
#define SMTF_STATES_END(SM_INSTANCE)     SM_INSTANCE##_MAX_STATES} SM_INSTANCE##_state_t;
#define SMTF_STATES_TYPE(SM_INSTANCE)    SM_INSTANCE##_state_t

#define SMTF_EVENTS_BEGIN(SM_INSTANCE)   typedef enum SM_INSTANCE##_event_enum {
#define SMTF_EVENTS_DECLARE(EVENT)       EVENT,
#define SMTF_EVENTS_END(SM_INSTANCE)     SM_INSTANCE##_MAX_EVENTS} SM_INSTANCE##_event_t;
#define SMTF_EVENTS_TYPE(SM_INSTANCE)    SM_INSTANCE##_event_t

#define SMTF_TRANS_TABLE_BEGIN(SM_INSTANCE)     typedef struct{SM_INSTANCE##_state_t state; void(*func)(void);} SM_INSTANCE##_action_t; static const SM_INSTANCE##_action_t SM_INSTANCE##_trans_table[SM_INSTANCE##_MAX_STATES][SM_INSTANCE##_MAX_EVENTS]={
#define SMTF_TRANS_TABLE_2_EVENTS_TO_NEW_STATES(ST_EV_0, F0, ST_EV_1, F1)                                                      { {ST_EV_0,F0}, {ST_EV_1,F1} },
#define SMTF_TRANS_TABLE_3_EVENTS_TO_NEW_STATES(ST_EV_0, F0, ST_EV_1, F1, ST_EV_2, F2)                                         { {ST_EV_0,F0}, {ST_EV_1,F1}, {ST_EV_2,F2} },
#define SMTF_TRANS_TABLE_4_EVENTS_TO_NEW_STATES(ST_EV_0, F0, ST_EV_1, F1, ST_EV_2, F2, ST_EV_3, F3)                            { {ST_EV_0,F0}, {ST_EV_1,F1}, {ST_EV_2,F2}, {ST_EV_3,F3} },
#define SMTF_TRANS_TABLE_5_EVENTS_TO_NEW_STATES(ST_EV_0, F0, ST_EV_1, F1, ST_EV_2, F2, ST_EV_3, F3, ST_EV_4, F4)               { {ST_EV_0,F0}, {ST_EV_1,F1}, {ST_EV_2,F2}, {ST_EV_3,F3}, {ST_EV_4,F4} },
#define SMTF_TRANS_TABLE_6_EVENTS_TO_NEW_STATES(ST_EV_0, F0, ST_EV_1, F1, ST_EV_2, F2, ST_EV_3, F3, ST_EV_4, F4, ST_EV_5, F5)  { {ST_EV_0,F0}, {ST_EV_1,F1}, {ST_EV_2,F2}, {ST_EV_3,F3}, {ST_EV_4,F4}, {ST_EV_5,F5} },
//#define SMTF_TRANS_TABLE_11_EVENTS_TO_NEW_STATES(ST_EV_0, F0, ST_EV_1, F1, ST_EV_2, F2, ST_EV_3, F3, ST_EV_4, F4, ST_EV_5, F5, ST_EV_6, F6, ST_EV_7, F7, ST_EV_8, F8, ST_EV_9, F9, ST_EV_10, F10)  { {ST_EV_0,F0}, {ST_EV_1,F1}, {ST_EV_2,F2}, {ST_EV_3,F3}, {ST_EV_4,F4}, {ST_EV_5,F5}, {ST_EV_6,F6}, {ST_EV_7,F7}, {ST_EV_8,F8}, {ST_EV_9,F9}, {ST_EV_10,F10} },
#define SMTF_TRANS_TABLE_END()                         };

#define SMTF_DEFINE(SM_INSTANCE, INITIAL_STATE)          static SM_INSTANCE##_state_t SM_INSTANCE##_state_var = (INITIAL_STATE);\
                                                         static inline void SM_INSTANCE##_TransitionFunction(SM_INSTANCE##_event_t _event)\
                                                         {SM_INSTANCE##_action_t _act; ATOMIC_BLOCK(ATOMIC_RESTORESTATE){_act = SM_INSTANCE##_trans_table[SM_INSTANCE##_state_var][_event];\
                                                         SM_INSTANCE##_state_var = _act.state;} if(_act.func!=NULL)_act.func();}
#define SMTF_HANDLER_CALL(SM_INSTANCE, EVENT)            SM_INSTANCE##_TransitionFunction(EVENT)
#define SMTF_STATE_GET(SM_INSTANCE)                      SM_INSTANCE##_state_var



#endif /* STATE_MACHINES_H */

