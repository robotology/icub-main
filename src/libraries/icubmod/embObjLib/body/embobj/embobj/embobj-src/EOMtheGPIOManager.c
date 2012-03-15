

/* @file       EOMtheGPIOManager.c
    @brief      This file implements the GPIO manager adapted for MEE.
    @author     marco.accame@iit.it
    @date       08/24/2011
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheErrorManager.h"

#include "EOMtask.h"
#include "EOMmutex.h"
#include "EOtheGPIOManager_hid.h"
#include "EOMtheCallbackManager_hid.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheGPIOManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheGPIOManager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOmgpioman_cfg_t eom_gpioman_DefaultCfg = 
{ 
    .priority   = 200, 
    .stacksize  = 512, 
    .period     = 10000
};



// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_eom_gpioman_tskproc_forever(EOMtask *rt, uint32_t period);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOMtheGPIOManager";

static EOMtheGPIOManager s_mgpiomanager = 
{
    .gpioman    = NULL,                   // EOtheGPIOManager           *gpioman;
    .tskproc    = NULL                    // EOMtask                     *tskproc;
}; 



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOMtheGPIOManager * eom_gpioman_Initialise(EOtheGPIO *thegpio, const eOmgpioman_cfg_t *gpiomancfg) 
{
    // already initialised ?
    if((NULL != s_mgpiomanager.gpioman) && (NULL != s_mgpiomanager.tskproc)) 
    {
        // already initialised !
        return(&s_mgpiomanager);
    }
    
    eo_errman_Assert(eo_errman_GetHandle(), (NULL == s_mgpiomanager.gpioman), s_eobj_ownname, "EOtheGPIOManager already initted by someone else");

    
    if(NULL == gpiomancfg)
    {
        gpiomancfg = &eom_gpioman_DefaultCfg;
    }
    
    // trying to initialise the manager without a valid gpio ??
    eo_errman_Assert(eo_errman_GetHandle(), (thegpio != NULL), s_eobj_ownname, "EOtheGPIO is NULL");

    // trying to initialise with wrong params error
    eo_errman_Assert(eo_errman_GetHandle(), (0 != gpiomancfg->period), s_eobj_ownname, "gpiomancfg->period is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != gpiomancfg->stacksize), s_eobj_ownname, "gpiomancfg->stacksize is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != gpiomancfg->priority), s_eobj_ownname, "gpiomancfg->priority is 0");
     
 
    // i get a basic gpio manager 
    s_mgpiomanager.gpioman = eo_gpioman_Initialise(thegpio, eom_mutex_New()); 

    // i prepare the task able to periodically tick the gpioman
    s_mgpiomanager.tskproc = eom_task_New(eom_mtask_Periodic,                          // type 
                                              gpiomancfg->priority,                                              // priority
                                              gpiomancfg->stacksize,                                              // stacksize
                                              NULL,                                             // startup_fn 
                                              s_eom_gpioman_tskproc_forever,                   // run_fn 
                                              0,                                      // maxmessages: msg fifo can hold all timers 
                                              gpiomancfg->period,                                  // timeoutorperiod
                                              NULL,
                                              eom_gpioman,                              // nameofthetask_fn
                                              "gpioman");
    
    // return the singleton handler
    return(&s_mgpiomanager);
    
}    


extern EOMtheGPIOManager * eom_gpioman_GetHandle(void) 
{ 
    return( (NULL == s_mgpiomanager.gpioman) ? (&s_mgpiomanager) : (NULL) );   
}



/* ------------------------------------------------------------------------------------
   definition of extern protected functions
   ------------------------------------------------------------------------------------
*/

// name of the task as it is shown in uvision
void eom_gpioman(void *p)
{
    eom_task_Start(p);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

 
static void s_eom_gpioman_tskproc_forever(EOMtask *rt, uint32_t period)
{
    eo_gpioman_Tick(s_mgpiomanager.gpioman, period);
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




