
/* @file       test-eogpio.c
	@brief      This file implements a test for embobj
	@author     marco.accame@iit.it
    @date       06/21/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

// embobj
#include "EoCommon.h"

#include "EOsm.h"
#include "eOcfg_sm_Ex2.h"



#include "EOumlsm.h"
#include "eOcfg_umlsm_Ex1.h"




// --------------------------------------------------------------------------------------------------------------------
// - declaration of external variables 
// --------------------------------------------------------------------------------------------------------------------


 
// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "test-eostatemachine.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static EOsm *s_sm01 = NULL;


static EOumlsm *s_statemachine01 = NULL;
static EOumlsm *s_statemachine02 = NULL;






// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


void test_eosm_Init(void)
{



    s_sm01 = eo_sm_New(eo_cfg_sm_ex2_Get());


    eo_sm_Start(s_sm01);
    
    
    eo_sm_ProcessEvent(s_sm01, eo_sm_ex2_ev01);

    eo_sm_Reset(s_sm01);

    eo_sm_ProcessEvent(s_sm01, eo_sm_ex2_ev01);

    eo_sm_Start(s_sm01);

    eo_sm_ProcessEvent(s_sm01, eo_sm_ex2_ev01);

    eo_sm_ProcessEvent(s_sm01, eo_sm_ex2_ev01);

    eo_sm_ProcessEvent(s_sm01, eo_sm_ex2_ev00);

    eo_sm_ProcessEvent(s_sm01, eo_sm_ex2_ev00);



    // i can create a state machine EOstateMachine by passing to its constructor a given configuration cfg.
    // 
    // the configuration cfg is a singleton object derived by EOVstateMachineCfg. the user shall fill the
    // required data structure, which is mainly mapped in rom. teh configuration uses rom to map 
    // constant data, such as states, transitions, callback functions, types and size of dynamic data. 
    // and uses ram to keep validation  flags.

    // the constructor of the state machine uses teh configuration cfg to allocate its own dynamic data
    // and keeps a reference to the constant data (states, etc).


    s_statemachine01 = eo_umlsm_New(eo_cfg_umlsm_Ex1_Get());
    
    eo_umlsm_Start(s_statemachine01);

    
    eo_umlsm_ProcessEvent(s_statemachine01, eo_umlsm_ex1_ev01, eo_umlsm_consume_UPTO08);
 

    // i can also create a new different state machine with teh same configuration cfg.
    // the state machine maintains the same logic as the previous one, 
    // but it has its own dynamic data structure, so it can evolve independently.
    s_statemachine02 = eo_umlsm_New(eo_cfg_umlsm_Ex1_Get());
    
    eo_umlsm_Start(s_statemachine02);
    
    eo_umlsm_ProcessEvent(s_statemachine01, eo_umlsm_ex1_ev02, eo_umlsm_consume_UPTO08);

}

void test_eosm_Tick(void)
{

 
}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

  


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



