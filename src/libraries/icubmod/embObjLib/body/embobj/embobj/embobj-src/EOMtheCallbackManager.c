
/* @file       EOMtheCallbackManager.c
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


#include "EOMtask.h"

#include "EOVtask.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheCallbackManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheCallbackManager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOmcallbackman_cfg_t eom_callbackman_DefaultCfg = 
{
    .priority   = 202, 
    .stacksize  = 512, 
    .queuesize  = 8
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_eom_callbackman_tsk_aftercallbackorontimeout(EOMtask *rt, uint32_t msg);

static eOresult_t s_eom_callbackman_execute(EOVtheCallbackManager *v, eOcallback_t cbk, void *arg, eOreltime_t tout);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOMtheCallbackManager";
 
static EOMtheCallbackManager s_eom_callbackmanager = 
{
    .vcm    = NULL,
    .tsk    = NULL
}; 


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

extern EOMtheCallbackManager * eom_callbackman_Initialise(const eOmcallbackman_cfg_t *cbkmancfg) 
{
    if(NULL != s_eom_callbackmanager.tsk) 
    {
        // already initialised
        return(&s_eom_callbackmanager);
    }

    if(NULL == cbkmancfg)
    {
        cbkmancfg = &eom_callbackman_DefaultCfg;
    }
    
    // trying to initialise with wrong params error
    eo_errman_Assert(eo_errman_GetHandle(), (0 != cbkmancfg->queuesize), s_eobj_ownname, "cbkmancfg->queuesize is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != cbkmancfg->stacksize), s_eobj_ownname, "cbkmancfg->stacksize is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != cbkmancfg->priority), s_eobj_ownname, "cbkmancfg->priority is 0");

    // i prepare the task able to execute callbacks actions associated to expiry of the timers or on gpio
    s_eom_callbackmanager.tsk = eom_task_New(eom_mtask_CallbackDriven, 
                                             cbkmancfg->priority, 
                                             cbkmancfg->stacksize,
                                             NULL,
                                             s_eom_callbackman_tsk_aftercallbackorontimeout, 
                                             cbkmancfg->queuesize,
                                             eok_reltimeINFINITE, 
                                             NULL,
                                             sys_cbackman,
                                             "cbkman"
                                             );

    // i initialise the base callback manager
    eov_callbackman_hid_Initialise(s_eom_callbackman_execute, s_eom_callbackmanager.tsk);
    
    return(&s_eom_callbackmanager);
}    


extern EOMtheCallbackManager* eom_callbackman_GetHandle(void)
{
    if(NULL == s_eom_callbackmanager.tsk) 
    {
        return(NULL);
    }
    
    return(&s_eom_callbackmanager);
}


extern eOresult_t eom_callbackman_Execute(EOMtheCallbackManager *p, eOcallback_t cbk, void *arg, eOreltime_t tout) 
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    return(s_eom_callbackman_execute(s_eom_callbackmanager.vcm, cbk, arg, tout));
}


extern EOMtask * eom_callbackman_GetTask(EOMtheCallbackManager *p) 
{
    if(NULL == p) 
    {
        return(NULL);
    }
    
    return(s_eom_callbackmanager.tsk);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


void sys_cbackman(void *p)
{
    eom_task_Start(p);
} 




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_eom_callbackman_tsk_aftercallbackorontimeout(EOMtask *rt, uint32_t msg)
{
    eOvoid_fp_void_t callback = (eOvoid_fp_void_t)msg;
 
    if(NULL == callback)
    {
        // callback not received: the waiting timeout has expired.
    }
} 


static eOresult_t s_eom_callbackman_execute(EOVtheCallbackManager *v, eOcallback_t cbk, void *arg, eOreltime_t tout)
{
    if(NULL == v) 
    {
        return(eores_NOK_nullpointer);
    }
    
    return(eov_task_tskExecCallback(s_eom_callbackmanager.tsk, cbk, arg, tout));
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





