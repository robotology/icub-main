
/* @file       EOStheCallbackManager.c
    @brief      This file implements internal implementation of the rtos callback manager singleton.
    @author     marco.accame@iit.it
    @date       10/21/2009
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOVtask.h"

#include "EOStheFOOP.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheCallbackManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheCallbackManager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOscallbackman_cfg_t eos_callbackman_DefaultCfg = 
{
    .queuesize = 8
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eos_callbackman_execute(EOVtheCallbackManager *v, eOcallback_t cbk, void *arg, eOreltime_t tout);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOStheCallbackManager";
 
static EOStheCallbackManager s_eos_callbackmanager = 
{
    .vcm    = NULL,
    .tsk    = NULL
}; 


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOStheCallbackManager * eos_callbackman_Initialise(const eOscallbackman_cfg_t *cbkmancfg) 
{
    if(NULL != s_eos_callbackmanager.tsk) 
    {
        // already initialised
        return(&s_eos_callbackmanager);
    }

    if(NULL == cbkmancfg)
    {
        cbkmancfg = &eos_callbackman_DefaultCfg;
    }
    
    // trying to initialise with wrong params error
    eo_errman_Assert(eo_errman_GetHandle(), (0 != cbkmancfg->queuesize), s_eobj_ownname, "cbkmancfg->queuesize is 0");


    // i prepare the task able to execute callbacks actions associated to expiry of the timers or on gpio
    s_eos_callbackmanager.tsk = eos_foop_GetHandle();
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != s_eos_callbackmanager.tsk), s_eobj_ownname, "eos_callbackman_Initialise(): FOOP not yet initialised");

    // i initialise the base callback manager
    eov_callbackman_hid_Initialise(s_eos_callbackman_execute, s_eos_callbackmanager.tsk);
    
    return(&s_eos_callbackmanager);
}    

extern EOStheCallbackManager* eos_callbackman_GetHandle(void)
{
    if(NULL == s_eos_callbackmanager.tsk) 
    {
        return(NULL);
    }
    
    return(&s_eos_callbackmanager);
}


extern eOresult_t eos_callbackman_Execute(EOStheCallbackManager *p, eOcallback_t cbk, void *arg) 
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    return(s_eos_callbackman_execute(s_eos_callbackmanager.vcm, cbk, arg, 0));
}


extern EOStheFOOP * eos_callbackman_GetTask(EOStheCallbackManager *p) 
{
    if(NULL == p) 
    {
        return(NULL);
    }
    
    return(s_eos_callbackmanager.tsk);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static eOresult_t s_eos_callbackman_execute(EOVtheCallbackManager *v, eOcallback_t cbk, void *arg, eOreltime_t tout)
{
    if(NULL == v) 
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eov_task_tskExecCallback(s_eos_callbackmanager.tsk, cbk, arg, 0));
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





