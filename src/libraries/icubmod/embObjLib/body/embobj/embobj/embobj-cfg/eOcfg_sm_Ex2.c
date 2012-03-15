
/** @file       eOcfg_sm_Ex2.c
    @brief      This file implements internal implementation to the configuration of the example state machine Ex2
    @author     marco.accame@iit.it
    @date       09/01/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "EoCommon.h"
#include "EOsm_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_sm_Ex2.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        01: define the max number of events managed by the state machine.
            No more than 32.
        -------------------------------------------------------------------------------
 
   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

enum
{
    s_events_number           = 16  
};

// use this trick to verify that the number of events is no more than 32
typedef int dummy[(s_events_number > 32) ? -1 : 1];


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section






// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        02: declare name of functions used as actions in the state machine
        -------------------------------------------------------------------------------
            these are the functions executed: 
            - in transitions (on-transitions), 
            - in entering the state (on-entry),
            - in exiting the state (on-exit).
            
            it is important to define them in here before the definition of other data 
            structures which embed their names in their inside.
             
            use following naming conventions:

                prefix:
                s_smcfg_<state-machine-name>_on_trans_                for on transition 
 
                suffix:
                <name-of-state>             for on entry and on exit
                <name-of-state>_<event>     for on transition and for guard

            if there are common functionalities to several functions it is
            much better to embed a single function in different ones, than to
            lose the naming conventions.

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */



static void s_smcfg_Ex2_init(EOsm *s);

static void s_smcfg_Ex2_reset(EOsm *s);


static void s_smcfg_Ex2_on_entry_ONE(EOsm *s);
static void s_smcfg_Ex2_on_exit_ONE(EOsm *s);
static void s_smcfg_Ex2_on_entry_TWO(EOsm *s);
static void s_smcfg_Ex2_on_exit_TWO(EOsm *s);
static void s_smcfg_Ex2_on_entry_THREE(EOsm *s);
static void s_smcfg_Ex2_on_exit_THREE(EOsm *s);
static void s_smcfg_Ex2_on_entry_FOUR(EOsm *s);
static void s_smcfg_Ex2_on_exit_FOUR(EOsm *s);


static void s_smcfg_Ex2_on_trans_ONE_ev01(EOsm *s);
static void s_smcfg_Ex2_on_trans_TWO_ev01(EOsm *s);
static void s_smcfg_Ex2_on_trans_TWO_ev02(EOsm *s);
static void s_smcfg_Ex2_on_trans_THREE_ev03(EOsm *s);
static void s_smcfg_Ex2_on_trans_FOUR_ev01(EOsm *s);
static void s_smcfg_Ex2_on_trans_FOUR_ev02(EOsm *s);
static void s_smcfg_Ex2_on_trans_ONE_ev07(EOsm *s);
static void s_smcfg_Ex2_on_trans_ONE_ev00(EOsm *s);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        03: define the table of the states and its indices
            No more than 256
        -------------------------------------------------------------------------------
 
   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

typedef enum 
{
     index00State_ONE                  = 0,    /**< state ONE */
     index01State_TWO                  = 1,    /**< state TWO */
     index02State_THREE                = 2,    /**< state THREE */
     index03State_FOUR                 = 3     /**< state FOUR */
} eOsmcfgStatesIndex_t;

static const eOsmState_t s_smcfg_Ex2_states_table_global[] = 
{ 
    {   // index00State_ONE
        .name           = "stONE",                                 // name
        .on_entry_fn    = s_smcfg_Ex2_on_entry_ONE,                   // onentry
        .on_exit_fn     = s_smcfg_Ex2_on_exit_ONE                     // onexit
    },
    {   // index01State_TWO
        .name           = "stTWO",                                 // name
        .on_entry_fn    = s_smcfg_Ex2_on_entry_TWO,                   // onentry
        .on_exit_fn     = s_smcfg_Ex2_on_exit_TWO                     // onexit
    },
    {   // index02State_THREE
        .name           = "stTHREE",                               // name
        .on_entry_fn    = s_smcfg_Ex2_on_entry_THREE,                 // onentry
        .on_exit_fn     = s_smcfg_Ex2_on_exit_THREE                   // onexit
    },
    {   // index03State_FOUR
        .name           = "stFOUR",                                // name
        .on_entry_fn    = s_smcfg_Ex2_on_entry_FOUR,                  // onentry
        .on_exit_fn     = s_smcfg_Ex2_on_exit_FOUR                    // onexit
    }
};

/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        05: define first state with s_initial_state
        -------------------------------------------------------------------------------
            it is the initial state of the state machine.
            however, if this state will have an initial_substate field not SMCfg_NONE,
            then teh state machine will go to initial_substate and recursively to 
            the deepest sub-state.

            initialise it with a state STATE_ value from eOsmcfgStatesIndex_t
    
   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */


enum    
{   // trick for compilers which cannot initialise a struct with a static const variable
    s_initial_state = index00State_ONE
};


/* ------------------------------------------------------------------------------------
   <SMCFG_SPECIALISE>

        06: define the table of transitions (one only for the entire state machine)
        -------------------------------------------------------------------------------
            the table of transitions contains all the transitions inside the state 
            machine. the transition is modeled as a struct which contains:
            - curr:             index of the originating state
            - next:             index to the destination state
            - evt:              the triggering event
            - on_transition_fn:    executed during the transition

            these fields are to be filled with already defined values or with NULL if
            no function is to be executed.

            use following naming conventions:
            
                prefix:        s_smcfg_<name>_trans_table_global

              keep unspecified the size of the array.

          IMPORTANT: by using the const qualifier these tables will stay in rom.
           

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */


static const eOsmTransition_t s_smcfg_Ex2_trans_table_global[] = 
{ 
    {   // transition marked as (1) in the picture EOtheSMCfgEx2.jpg
        .curr               = index00State_ONE,                                 
        .next               = index01State_TWO,                                  
        .evt                = eo_sm_ex2_ev01,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_ONE_ev01               
    },
 
    {   // transition marked as (2) in the picture EOtheSMCfgEx2.jpg
        .curr               = index01State_TWO,                                 
        .next               = index00State_ONE,                                  
        .evt                = eo_sm_ex2_ev01,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_TWO_ev01               
    },
 
    {   // transition marked as (3) in the picture EOtheSMCfgEx2.jpg
        .curr               = index01State_TWO,                                  
        .next               = index02State_THREE,                                
        .evt                = eo_sm_ex2_ev02,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_TWO_ev02               
    },
 
    {   // transition marked as (4) in the picture EOtheSMCfgEx2.jpg
        .curr               = index02State_THREE,                                
        .next               = index03State_FOUR,                                 
        .evt                = eo_sm_ex2_ev03,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_THREE_ev03             
    },
    
    {   // transition marked as (5) in the picture EOtheSMCfgEx2.jpg
        .curr               = index03State_FOUR,                                 
        .next               = index01State_TWO,                                  
        .evt                = eo_sm_ex2_ev01,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_FOUR_ev01              
    },

    {   // transition marked as (6) in the picture EOtheSMCfgEx2.jpg
        .curr               = index03State_FOUR,                                 
        .next               = index03State_FOUR,                                 
        .evt                = eo_sm_ex2_ev02,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_FOUR_ev02              
    }, 
    
    {   // transition marked as (7) in the picture EOtheSMCfgEx2.jpg
        .curr               = index00State_ONE,                                  
        .next               = index02State_THREE,                                
        .evt                = eo_sm_ex2_ev07,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_ONE_ev07               
    },

   {   // transition marked as (8) in the picture EOtheSMCfgEx2.jpg
        .curr               = index00State_ONE,                                  
        .next               = index00State_ONE,                                  
        .evt                = eo_sm_ex2_ev00,                             
        .on_transition_fn   = s_smcfg_Ex2_on_trans_ONE_ev00               
    }      
};



/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        06: define the global number of transitions
        -------------------------------------------------------------------------------
             naming convention:

                prefix:    s_trans_number_
                suffix: <name>

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */

/** @var        s_trans_number_ONE
    @brief      It is the number of transitions from the state EXAMPLE1.
    @details    use the following naming convention:
                s_smcfg_<name>_trans_number 
 
                initialise it with ... 
                s_smcfg_Ex2_trans_number = (sizeof(s_smcfg_Ex2_trans_table_global)/sizeof(eOsmTransition_t));
 **/



enum    
{   // trick for compilers which cannot initialise a struct with a static const variable
    s_trans_number = (sizeof(s_smcfg_Ex2_trans_table_global)/sizeof(eOsmTransition_t))
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
{   // trick for compilers which cannot initialise a struct with a static const variable
    s_states_number = (sizeof(s_smcfg_Ex2_states_table_global)/sizeof(eOsmState_t))
};




/** @var        s_reset
    @brief      It is the pointer to the reset function for the state machine. This functions
                should erase all dynamic data structure and perform proper operations which are
                dependent of the state machine implementation defined in this file.
    @details    Use the name s_reset.
 **/

//#define s_reset     s_smcfg_Ex2_reset    // or any other function or NULL
// on armcc a struct can be correctly initialised by the following static const variable
//static const eOvoid_fp_cvoidp_t s_reset = s_smcfg_Ex2_reset;  


/** @var        s_dynamicdata_size
    @brief      It is the size of the dynamic data that the state machine object will allocate.
    @details    Use the name s_dynamicdata_size.
 **/

enum    
{   // trick for compilers which cannot initialise a struct with a static const variable
    s_dynamicdata_size = sizeof(eOsmDynamicDataEx2_t)
};



//#define s_init     s_smcfg_Ex2_init    // or any other function or NULL

//static const eOvoid_fp_cvoidp_t s_init = s_smcfg_Ex2_init;  

/* ------------------------------------------------------------------------------------

   <SMCFG_SPECIALISE>

        66: define the variable with configuration data for the state machine
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
static const eOsm_cfg_t s_theconfiguration =    
{
    .nstates                = s_states_number,  
    .ntrans                 = s_trans_number, 
    .maxevts                = s_events_number,                                         
    .initstate              = s_initial_state,
    .sizeofdynamicdata      = s_dynamicdata_size,     
    .states                 = s_smcfg_Ex2_states_table_global,          
    .transitions            = s_smcfg_Ex2_trans_table_global,                          
    .init_fn                = s_smcfg_Ex2_init,
    .resetdynamicdata_fn    = s_smcfg_Ex2_reset
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

extern const eOsm_cfg_t * eo_cfg_sm_ex2_Get(void)
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
            on-transition, guard.

   </SMCFG_SPECIALISE>
   ------------------------------------------------------------------------------------
 */ 
 



static void s_smcfg_Ex2_init(EOsm *s) 
{
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    // in here you could .... initialise a library which requires single init

    printf("init executed, initial state is %s\n", s->cfg->states[s_initial_state].name);

}
static void s_smcfg_Ex2_reset(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    // you must :
    // 1. clear dynamic data
    ram->number_of_fired_transitions = 0;
    ram->number_of_crossed_states    = 0;
    ram->data3                       = 0;
        
    // 2. do proper actions which are state-machine dependent.
    // for instance ...
    ram->data3 = 666;

    printf("reset of state machine executed, we are in initial state %s ... again\n", s->cfg->states[s_initial_state].name);

} 


static void s_print(EOsm *s, const char *str, uint8_t enter)
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    if(1 == enter)
    {
        ram->number_of_crossed_states ++;  
    }

    printf("%s state %s. it is the %d-th visited state\n",
            str, 
            s->cfg->states[s->activestate].name,
            ram->number_of_crossed_states);

}
static void s_smcfg_Ex2_on_entry_ONE(EOsm *s) 
{
    s_print(s, "entering", 1);
}
static void s_smcfg_Ex2_on_exit_ONE(EOsm *s) 
{
    s_print(s, "exiting", 0);
}
static void s_smcfg_Ex2_on_entry_TWO(EOsm *s) 
{
    s_print(s, "entering", 1);
}
static void s_smcfg_Ex2_on_exit_TWO(EOsm *s) 
{
    s_print(s, "exiting", 0);
}
static void s_smcfg_Ex2_on_entry_THREE(EOsm *s) 
{
    s_print(s, "entering", 1);
}
static void s_smcfg_Ex2_on_exit_THREE(EOsm *s) 
{
    s_print(s, "exiting", 0);
}
static void s_smcfg_Ex2_on_entry_FOUR(EOsm *s) 
{
    s_print(s, "entering", 1);
}
static void s_smcfg_Ex2_on_exit_FOUR(EOsm *s) 
{
    s_print(s, "exiting", 0);
}



static void s_smcfg_Ex2_on_trans_ONE_ev01(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;  
    printf("state one, evt 1, trans %d\n", ram->number_of_fired_transitions);

}
static void s_smcfg_Ex2_on_trans_TWO_ev01(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;  
    printf("state two, evt 1, trans %d\n", ram->number_of_fired_transitions);    

}
static void s_smcfg_Ex2_on_trans_TWO_ev02(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    printf("state two, evt 2, trans %d\n", ram->number_of_fired_transitions);

}
static void s_smcfg_Ex2_on_trans_THREE_ev03(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    printf("state three, evt 3, trans %d\n", ram->number_of_fired_transitions);

}
static void s_smcfg_Ex2_on_trans_FOUR_ev01(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    printf("state four, evt 1, trans %d\n", ram->number_of_fired_transitions);

}
static void s_smcfg_Ex2_on_trans_FOUR_ev02(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    printf("state four, evt 2, trans %d\n", ram->number_of_fired_transitions);

}
static void s_smcfg_Ex2_on_trans_ONE_ev07(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    printf("state one, evt 7, trans %d\n", ram->number_of_fired_transitions);

}
static void s_smcfg_Ex2_on_trans_ONE_ev00(EOsm *s) 
{
    
    eOsmDynamicDataEx2_t *ram = eo_sm_GetDynamicData(s);
    
    ram->number_of_fired_transitions ++;   
    printf("state one, evt 0, trans %d\n", ram->number_of_fired_transitions);

}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



