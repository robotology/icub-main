
/** @file       eOcfg_umlsm_Ex1.c
    @brief      This file contains the configuration of the example state machine Ex1
    @author     marco.accame@iit.it
    @date       09/02/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "EoCommon.h"
#include "EOumlsm.h"
#include "EOumlsm_hid.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_umlsm_Ex1.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        01: define name of states with enum eOumlsmStateIndex_t
        -------------------------------------------------------------------------------
            these are the names of the states and indices to array.
             
                  insert in enum eOumlsmStateIndex_t the user-defined name of states with tags formed
            with the following naming convention:

                prefix:        STATE_ 
                suffix:        <name-of-state>        i.e., INIT, IDLE, TRANSMIT, RECEIVE, etc.
                
            IMPORTANT: keep first value to 0 and dont put any hole.
            

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @typedef    typedef enum eOumlsmStateIndexEx1_t
    @brief      eOumlsmStateIndexEx1_t contains the names of the states as indices to array.
    @details    Insert in enum eOumlsmStateIndexEx1_t the user-defined name of states with tags formed
                with the following naming convention:
                @arg prefix: STATE_ 
                @arg suffix: <name-of-state>
                
                IMPORTANT: keep first value to 0 and don't put any hole.
 **/ 
typedef enum
{
     STATE_ONE                  = 0,    /**< state ONE */
     STATE_TWO                  = 1,    /**< state TWO */
     STATE_THREE                = 2,    /**< state THREE */
     STATE_FOUR                 = 3     /**< state FOUR */
} eOumlsmStateIndexEx1_t;



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        02: declare name of functions used as actions in the state machine
        -------------------------------------------------------------------------------
            these are the functions executed in states (on-entry or on-exit) and
            in transitions (on-transitions or guard_fn). it is important to define them
            in here before the definition of other data structures which embed their
            names in their inside.
             
            use following naming conventions:

                prefix:
                s_eo_cfg_umlsm_<state-machine-name>_on_entry_                 for on entry
                s_eo_cfg_umlsm_<state-machine-name>_on_exit_                  for on exit
                s_eo_cfg_umlsm_<state-machine-name>_on_trans_                 for on transition 
                s_eo_cfg_umlsm_<state-machine-name>_guard_trans_              for guards

                suffix:
                <name-of-state>             for on entry and on exit
                <name-of-state>_<event>     for on transition and for guard_fn

            if there are common functionalities to several functions it is
            much better to embed a single function in different ones, than to
            lose the naming conventions.

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */



 
  

/** @fn         static void s_eo_cfg_umlsm_Ex1_reset(EOumlsm *s)
    @brief      Function executed by the state machine when a reset is required.
    @param      s               The calling state machine pass in s the pointer to the object EOumlsm itself.
    @details    use following naming conventions:
                @arg prefix: s_eo_cfg_umlsm_<name-of-state-machine> 
                @arg suffix: _reset
     
 **/ 
static void s_eo_cfg_umlsm_Ex1_reset(EOumlsm *s);


/** @fn         static void s_eo_cfg_umlsm_Ex1_on_entry_ONE(EOumlsm *s)
    @brief      Function executed by the state machine on entry of state EXAMPLE1.
    @param      s               The calling state machine pass in s the pointer to the object EOumlsm itself
    @details    use following naming conventions:
                @arg prefix: s_eo_cfg_umlsm_Ex1_on_entry_ 
                @arg suffix: <name-of-state>
                
                SUGGESTION: if there are common functionalities to several functions it is
                much better to embed a single function in different ones, than to
                lose the naming conventions
 **/ 
static void s_eo_cfg_umlsm_Ex1_on_entry_ONE(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_entry_TWO(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_entry_THREE(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_entry_FOUR(EOumlsm *s);


/** @fn         static void s_eo_cfg_umlsm_Ex1_on_exit_ONE(EOumlsm *s)
    @brief      function executed by the state machine on exit from state EXAMPLE1.
    @param      s               The calling state machine pass in s the pointer to the object EOumlsm itself.
    @details    use following naming conventions:
                @arg prefix: s_eo_cfg_umlsm_Ex1_on_exit_ 
                @arg suffix: <name-of-state>
                
                SUGGESTION: if there are common functionalities to several functions it is
                much better to embed a single function in different ones, than to
                lose the naming conventions
 **/ 
static void s_eo_cfg_umlsm_Ex1_on_exit_ONE(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_exit_TWO(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_exit_THREE(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_exit_FOUR(EOumlsm *s);


/** @fn         static void s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev01(EOumlsm *s)
    @brief      function executed by the state machine on transition from state EXAMPLE1 upon event ev01.
    @param      s               The calling state machine pass in s the pointer to the object EOumlsm 
                                itself.
    @details    use following naming conventions:
                @arg prefix: s_eo_cfg_umlsm_Ex1_on_trans_ 
                @arg suffix: <name-of-state>_<name-of-event>
                
                In case of transitions from many states, the string <name-of-state> can be MANYSTATES
                
                SUGGESTION: if there are common functionalities to several functions it is
                much better to embed a single function in different ones, than to
                lose the naming conventions
 **/ 
static void s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev01(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev02(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_TWO_ev03(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_TWO_ev07(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_THREE_ev04(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_THREE_ev06(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_FOUR_ev05(EOumlsm *s);
static void s_eo_cfg_umlsm_Ex1_on_trans_MANYSTATES_evreset(EOumlsm *s);


/** @fn         static uint8_t s_eo_cfg_umlsm_Ex1_guard_trans_ONE_ev01(EOumlsm *s)
    @brief      Function executed by the state machine in state EXAMPLE1 upon reception of event ev01.
    @param      s               The calling state machine pass in s the pointer to the object EOumlsm itself
    @return     1 upon true condition, 0 upon false condition. 
    @details    use following naming conventions:
                @arg prefix: s_eo_cfg_umlsm_Ex1_guard_trans_ 
                @arg suffix: <name-of-state>_<name-of-event>
                
                SUGGESTION: if there are common functionalities to several functions it is
                much better to embed a single function in different ones, than to
                lose the naming conventions
 **/ 
static uint8_t s_eo_cfg_umlsm_Ex1_guard_trans_ONE_ev01(EOumlsm *s);
 



static void s_dummy_to_hide_several_compiler_warnings(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------



/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        03: define the table of owners for each state
        -------------------------------------------------------------------------------

            the table of owners of a state is an array which contains in 0-th position
            the name of the state itself (one state owns itself), in 1-st position the 
            name of its father, in 2-nd position the name of its grand-dad, etc.

            naming convention:

                prefix:     s_eo_cfg_umlsm_Ex1_owners_table_
                suffix:     <name-of-state>             i.e,: EXAMPLE1, etc...
                contains:   STATE_<name-of-state>       i.e,: STATE_EXAMPLE1, etc...

    </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @var        s_eo_cfg_umlsm_Ex1_owners_table_ONE
    @brief      It is an array which contains the index for EXAMPLE1 and all the indices of 
                the states which own the state EXAMPLE1.
    @details    use the following naming convention:
                @arg prefix: s_eo_cfg_umlsm_Ex1_owners_table_ 
                @arg suffix: <name-of-state>
                
                initialise it with ... 
                s_eo_cfg_umlsm_Ex1_owners_table_ONE[] = { STATE_ONE, STATE_FOUR };
 **/
static const uint8_t s_eo_cfg_umlsm_Ex1_owners_table_ONE[]     = { STATE_ONE, STATE_FOUR };  
static const uint8_t s_eo_cfg_umlsm_Ex1_owners_table_TWO[]     = { STATE_TWO, STATE_FOUR }; 
static const uint8_t s_eo_cfg_umlsm_Ex1_owners_table_THREE[]   = { STATE_THREE }; 
static const uint8_t s_eo_cfg_umlsm_Ex1_owners_table_FOUR[]    = { STATE_FOUR }; 


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        04: define the sizes of the table of owners for each state
        -------------------------------------------------------------------------------

             naming convention:

             prefix:    s_eo_cfg_umlsm_Ex1_owners_number_
             suffix:    <name-of-state>             i.e,: EXAMPLE1, etc...

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */
 
/** @var        s_eo_cfg_umlsm_Ex1_owners_number_ONE
    @brief      It is the number of states which own the state EXAMPLE1.
    @details    use the following naming convention:
                @arg prefix: s_eo_cfg_umlsm_Ex1_owners_number_ 
                @arg suffix: <name-of-state>
                
                initialise it with ... 
                s_eo_cfg_umlsm_Ex1_owners_number_ONE = (sizeof(s_eo_cfg_umlsm_Ex1_owners_table_ONE)/sizeof(uint8_t));
 **/ 
enum    
{   // trick for some compilers, which cannot initialise a struct with a static const uint8_t variable
    s_eo_cfg_umlsm_Ex1_owners_number_ONE = (sizeof(s_eo_cfg_umlsm_Ex1_owners_table_ONE)/sizeof(uint8_t)),
    s_eo_cfg_umlsm_Ex1_owners_number_TWO = (sizeof(s_eo_cfg_umlsm_Ex1_owners_table_TWO)/sizeof(uint8_t)),
    s_eo_cfg_umlsm_Ex1_owners_number_THREE = (sizeof(s_eo_cfg_umlsm_Ex1_owners_table_THREE)/sizeof(uint8_t)),
    s_eo_cfg_umlsm_Ex1_owners_number_FOUR = (sizeof(s_eo_cfg_umlsm_Ex1_owners_table_FOUR)/sizeof(uint8_t))
};




/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        05: define tables of transitions from each state
        -------------------------------------------------------------------------------
            the table of transitions contains every possible transition from a state. 
            the transition is modeled as a struct which contains:
            - trigger:            if the event matches it then the guard_fn is evaluated
            - guard_fn:            if it evaluates to true the transition becomes active
            - next:                index to the target state of the transition
            - on_transition:     executed during the transition

            these fields are to be filled with already defined values or with NULL if
            no function is to be executed.

            use following naming conventions:
            
                prefix:        s_eo_cfg_umlsm_Ex1_trans_table_
                suffix:     <name-of-state>

              keep unspecified the size of the array.

          IMPORTANT: by using the const qualifier these tables will stay in rom.
           

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @var        s_eo_cfg_umlsm_Ex1_trans_table_ONE
    @brief      It is the array of transitions from the state EXAMPLE1.
    @details    use the following naming convention:
                @arg prefix: s_eo_cfg_umlsm_Ex1_trans_table_ 
                @arg suffix: <name-of-state>
 **/
static const eOumlsmTransition_t s_eo_cfg_umlsm_Ex1_trans_table_ONE[] = 
{ 
    {
        .trigger                = eo_umlsm_ex1_ev01,                                   // trigger
        .next                   = STATE_ONE,                        // next
        .guard_fn               = s_eo_cfg_umlsm_Ex1_guard_trans_ONE_ev01,                 // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev01               // on-transition
    },
    
    {
        .trigger                = eo_umlsm_ex1_ev02,                                   // trigger
        .next                   = STATE_TWO,                        // next
        .guard_fn               = NULL,                                   // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev02               // on-transition
    }
};

static const eOumlsmTransition_t s_eo_cfg_umlsm_Ex1_trans_table_TWO[] = 
{ 
    {
        .trigger                = eo_umlsm_ex1_ev07,                                   // trigger
        .next                   = STATE_ONE,                        // next
        .guard_fn               = NULL,                                   // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_TWO_ev07               // on-transition
    },
    
    {
        .trigger                = eo_umlsm_ex1_ev03,                                   // trigger
        .next                   = STATE_THREE,                        // next
        .guard_fn               = NULL,                                   // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_TWO_ev03               // on-transition
    }
};

static const eOumlsmTransition_t s_eo_cfg_umlsm_Ex1_trans_table_THREE[] = 
{ 
    {
        .trigger                = eo_umlsm_ex1_ev06,                                   // trigger
        .next                   = STATE_TWO,                        // next 
        .guard_fn               = NULL,                                   // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_THREE_ev06               // on-transition
    },
    
    {
        .trigger                = eo_umlsm_ex1_ev04,                                   // trigger
        .next                   = STATE_FOUR,                        // next 
        .guard_fn               = NULL,                                   // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_THREE_ev04               // on-transition
    }
};


static const eOumlsmTransition_t s_eo_cfg_umlsm_Ex1_trans_table_FOUR[] = 
{ 
    {
        .trigger                = eo_umlsm_ex1_ev05,                                   // trigger
        .next                   = STATE_THREE,                        // next
        .guard_fn               = NULL,                                   // guard_fn
        .on_transition_fn       = s_eo_cfg_umlsm_Ex1_on_trans_FOUR_ev05               // on-transition
    }
};



/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        06: define the number of transitions from each state
        -------------------------------------------------------------------------------
             naming convention:

                prefix:    s_trans_number_
                suffix: <name-of-state>

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @var        s_trans_number_ONE
    @brief      It is the number of transitions from the state EXAMPLE1.
    @details    use the following naming convention:
                @arg prefix: s_trans_number_ 
                @arg suffix: <name-of-state>
                
                initialise it with ... 
                s_trans_number_ONE = (sizeof(s_eo_cfg_umlsm_Ex1_trans_table_ONE)/sizeof(eOumlsmTransition_t));
 **/ 
enum    
{   // trick for some compilers, which cannot initialise a struct with a static const uint8_t variable
    s_trans_number_ONE = (sizeof(s_eo_cfg_umlsm_Ex1_trans_table_ONE)/sizeof(eOumlsmTransition_t)),
    s_trans_number_TWO = (sizeof(s_eo_cfg_umlsm_Ex1_trans_table_TWO)/sizeof(eOumlsmTransition_t)),
    s_trans_number_THREE = (sizeof(s_eo_cfg_umlsm_Ex1_trans_table_THREE)/sizeof(eOumlsmTransition_t)),
    s_trans_number_FOUR = (sizeof(s_eo_cfg_umlsm_Ex1_trans_table_FOUR)/sizeof(eOumlsmTransition_t))
};


/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        07: define tables of states
        -------------------------------------------------------------------------------
            the table of states contains all teh states of teh state machine. 
            a state is modeled as a struct which contains:
            - initial_substate:        it is the index to default entry substate.
                                    if not SMCfg_NONE, when the state machine
                                    enters the state must end up in the state indexed by
                                    initial_substate.
            - owners_number:        the number of owners including teh state itself
            - owners_table:            pointer to table of owners. in position 0-th there
                                    is the index of the state itself.
            - transitions_number
            - transitions_table
            - on_entry_fn:        if not NULL it is executed upon entry in the state
            - on_exit_fn:        if not NULL it is executed upon exit from the state


            these fields are to be filled with already defined values or with NULL if
            no function is to be executed.

            use following naming convention and dont change it:
            
                name:        s_states_table[]

          IMPORTANT: by using the const qualifier these tables will stay in rom.

          IMPORTANT: order is important. in position x-th place state whose name
                     is defined as x.
                     example: assert(i != s_states_table[i]->owners_table[0]);
                     a check must be done at initialisation time.
       

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */



/** @var        s_states_table
    @brief      It is the array whcih contains all the states.
    @details    Use the name s_states_table.
 **/
static const eOumlsmState_t s_states_table[] = 
{
    {   // position 0: STATE_ONE
        .name               = "st1",
        .initial_substate   = EOK_uint08dummy,                            // initial_substate
        .owners_number      = s_eo_cfg_umlsm_Ex1_owners_number_ONE,          // number of owners. 1 if the state owns itself, 2 if it has one up to it
        .owners_table       = s_eo_cfg_umlsm_Ex1_owners_table_ONE,           // owner table
        .transitions_number = s_trans_number_ONE,           // number or transitions from state
        .transitions_table  = s_eo_cfg_umlsm_Ex1_trans_table_ONE,            // table of transitions 
        .on_entry_fn        = s_eo_cfg_umlsm_Ex1_on_entry_ONE,               // function on entry 
        .on_exit_fn         = s_eo_cfg_umlsm_Ex1_on_exit_ONE,                // function on exit 
    },

    {   // position 1: STATE_TWO
        .name               = "st2",
        .initial_substate   = EOK_uint08dummy,                            // initial_substate
        .owners_number      = s_eo_cfg_umlsm_Ex1_owners_number_TWO,          // number of owners. 1 if the state owns itself, 2 if it has one up to it
        .owners_table       = s_eo_cfg_umlsm_Ex1_owners_table_TWO,           // owner table    
        .transitions_number = s_trans_number_TWO,           // number or transitions from state
        .transitions_table  = s_eo_cfg_umlsm_Ex1_trans_table_TWO,            // table of transitions 
        .on_entry_fn    = s_eo_cfg_umlsm_Ex1_on_entry_TWO,               // function on entry 
        .on_exit_fn     = s_eo_cfg_umlsm_Ex1_on_exit_TWO,                // function on exit 
    },
    
    {   // position 2: STATE_THREE
        .name               = "st3",
        .initial_substate   = EOK_uint08dummy,                            // initial_substate
        .owners_number      = s_eo_cfg_umlsm_Ex1_owners_number_THREE,          // number of owners. 1 if the state owns itself, 2 if it has one up to it
        .owners_table       = s_eo_cfg_umlsm_Ex1_owners_table_THREE,           // owner table    
        .transitions_number = s_trans_number_THREE,           // number or transitions from state
        .transitions_table  = s_eo_cfg_umlsm_Ex1_trans_table_THREE,            // table of transitions 
        .on_entry_fn        = s_eo_cfg_umlsm_Ex1_on_entry_THREE,               // function on entry 
        .on_exit_fn         = s_eo_cfg_umlsm_Ex1_on_exit_THREE,                // function on exit 
    },  
    
    {   // position 3: STATE_FOUR
        .name               = "st4",
        .initial_substate   = STATE_ONE,                    // initial_substate
        .owners_number      = s_eo_cfg_umlsm_Ex1_owners_number_FOUR,          // number of owners. 1 if the state owns itself, 2 if it has one up to it
        .owners_table       = s_eo_cfg_umlsm_Ex1_owners_table_FOUR,           // owner table    
        .transitions_number = s_trans_number_FOUR,           // number or transitions from state
        .transitions_table  = s_eo_cfg_umlsm_Ex1_trans_table_FOUR,            // table of transitions 
        .on_entry_fn        = s_eo_cfg_umlsm_Ex1_on_entry_FOUR,               // function on entry 
        .on_exit_fn         = s_eo_cfg_umlsm_Ex1_on_exit_FOUR,                // function on exit 
    }        
};

/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        08: define number of states with s_states_number
        -------------------------------------------------------------------------------
             this is the number of states. so far it is not used.

               if you have followed the naming conventions u dont need to change anything.


   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
*/

/** @var        s_states_number
    @brief      It is the number of states.
    @details    Use the name s_states_number.
 **/

enum    
{   // trick for some compilers, which cannot initialise a struct with static const variables
    s_states_number = (sizeof(s_states_table)/sizeof(eOumlsmState_t))
};


/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        09: define first state with s_initial_state
        -------------------------------------------------------------------------------
            it is the initial state of the state machine.
            however, if this state will have an initial_substate field not SMCfg_NONE,
            then teh state machine will go to initial_substate and recursively to 
            the deepest sub-state.

               initialise it with a state STATE_ value from eOumlsmcfgStates_t
    
   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @var        s_initial_state
    @brief      It is the index of initial state inside the states table.
    @details    Use the name s_initial_state.
 **/
enum    
{   // trick for some compilers, which cannot initialise a struct with static const variables
    s_initial_state = STATE_FOUR
};


/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        10: define size of an optional fifo queue with s_internal_event_fifo_size
        -------------------------------------------------------------------------------
               the fifo queue will contain high priority internal events that will be 
               processed before any external event. 

               set to 0 if you dont want any of it.

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @var        s_internal_event_fifo_size
    @brief      It is the size of the FIFO queue containing internal events. Sets different
                from zero only if the functions of the state machine generate internal events.
    @details    Use the name s_internal_event_fifo_size.
 **/
enum    
{   // trick for some compilers which cannot initialise a struct with static const variables
    s_internal_event_fifo_size = 6
};


/** @var        s_reset
    @brief      It is the pointer to the reset function for the state machine. This functions
                should erase all dynamic data structure and perform proper operations which are
                dependent of the state machine implementation defined in this file.
    @details    Use the name s_reset.
 **/
// trick for some compilers which cannot initialise a struct with static const variables
#define s_reset     s_eo_cfg_umlsm_Ex1_reset    // or any other function or NULL


/** @var        s_dynamicdata_size
    @brief      It is the size of the dynamic data that the state machine object will allocate.
    @details    Use the name s_dynamicdata_size.
 **/
enum    
{   // trick for some compilers which cannot initialise a struct with static const variables
    s_dynamicdata_size = sizeof(eOumlsmDynamicDataEx1_t)
};


/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        11: define the variable with configuration data for the state machine
        -------------------------------------------------------------------------------
               this is the ultimate data structure that is required to specialise a
            state machine. 

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */ 

/** @var        s_theconfiguration
    @brief      It is the required configuration, which is the colelction of previously defined 
                const variables.
    @details    Use the name s_theconfiguration.
 **/
static const eOumlsm_cfg_t s_theconfiguration =    
{
    .sizeofdynamicdata          = s_dynamicdata_size,                         
    .initial_state              = s_initial_state,                            
    .internal_event_fifo_size   = s_internal_event_fifo_size,  
    .states_number              = s_states_number, 
    .states_table               = s_states_table,                         
    .resetdynamicdata_fn        = s_reset                           
}; 


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        12: define the function which will return the configuration of the state machine
        -------------------------------------------------------------------------------
        enjoy it.

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */ 

extern eOumlsm_cfg_t * eo_cfg_umlsm_Ex1_Get(void)
{
    return(&s_theconfiguration);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

// WARNING: dont use static ram in the static function. they must be fully re-entrant.


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        13: define the static functions already declared 
        -------------------------------------------------------------------------------
            in here it is required to place the definition of on-entr, on-exit, 
            on-transition, guard_fn.

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */ 
 

static void s_eo_cfg_umlsm_Ex1_reset(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    // you must :
    // 1. clear dynamic data
    ram->number_of_fired_transitions = 0;
    ram->number_of_attempt_to_guard_state1 = 0;

        
    // 2. do proper actions which are state-machine dependent.

    if(666 == ram->number_of_fired_transitions)
    {
        s_dummy_to_hide_several_compiler_warnings();
    }
    

} 

// on entry
static void s_eo_cfg_umlsm_Ex1_on_entry_ONE(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    printf("EOumlsm: entering state one\n\r");
}

static void s_eo_cfg_umlsm_Ex1_on_entry_TWO(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    printf("EOumlsm: entering state two\n\r");
}

static void s_eo_cfg_umlsm_Ex1_on_entry_THREE(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    printf("EOumlsm: entering state three\n\r");
}

static void s_eo_cfg_umlsm_Ex1_on_entry_FOUR(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    printf("EOumlsm: entering state four\n\r");
}


// on exit
static void s_eo_cfg_umlsm_Ex1_on_exit_ONE(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
}

static void s_eo_cfg_umlsm_Ex1_on_exit_TWO(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
}

static void s_eo_cfg_umlsm_Ex1_on_exit_THREE(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
}

static void s_eo_cfg_umlsm_Ex1_on_exit_FOUR(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
}


// on trans from ONE
static void s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev01(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    
    eo_umlsm_PutInternalEvent(s, eo_umlsm_ex1_ev02);
    eo_umlsm_PutInternalEvent(s, eo_umlsm_ex1_ev07);
    eo_umlsm_PutInternalEvent(s, eo_umlsm_ex1_ev02);
    eo_umlsm_PutInternalEvent(s, eo_umlsm_ex1_ev07);

    printf("EOumlsm: inside state one and evt1: added internal events ev02, ev07, ev02, ev07\n\r");

}

static void s_eo_cfg_umlsm_Ex1_on_trans_ONE_ev02(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
}



// on trans from TWO

static void s_eo_cfg_umlsm_Ex1_on_trans_TWO_ev03(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
}

static void s_eo_cfg_umlsm_Ex1_on_trans_TWO_ev07(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
}


// on trans from THREE
static void s_eo_cfg_umlsm_Ex1_on_trans_THREE_ev04(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
}

static void s_eo_cfg_umlsm_Ex1_on_trans_THREE_ev06(EOumlsm *s)
{

}


// on trans from FOUR
static void s_eo_cfg_umlsm_Ex1_on_trans_FOUR_ev05(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
}



// from many states
static void s_eo_cfg_umlsm_Ex1_on_trans_MANYSTATES_evreset(EOumlsm *s) 
{
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
}




static uint8_t s_eo_cfg_umlsm_Ex1_guard_trans_ONE_ev01(EOumlsm *s) 
{
    uint8_t retval = 0;
    eOumlsmDynamicDataEx1_t *ram = eo_umlsm_GetDynamicData(s);

    retval = (ram->number_of_attempt_to_guard_state1) % 2;   

    ram->number_of_attempt_to_guard_state1 ++;
    
    return(retval);
}

static void s_dummy_to_hide_several_compiler_warnings(void) 
{

    // this function calls all the functions that have been defined but not used.
    // at the end we shall have only one warning: the one for this function.

    s_eo_cfg_umlsm_Ex1_on_trans_MANYSTATES_evreset(NULL);

}
 

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



