
/* @file       EOMtheTimerManager.c
	@brief      This file implements internal implementation of the rtos timer manager singleton.
	@author     marco.accame@iit.it
    @date       08/03/2009
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"
#include "EoCommon.h"
#include "EOaction.h"

#include "EOtheErrorManager.h"

#include "EOVtheTimerManager_hid.h"
#include "EOtimer_hid.h"
#include "EOMmutex.h"
#include "EOMtheCallbackManager_hid.h"

#include "EOMtask_hid.h"

#include "osal.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheTimerManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOMtheTimerManager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define _EOM_TIMERMAN_EVENTS_DIRECTLY_SENT_BY_OSAL_


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOmtimerman_cfg_t eom_timerman_DefaultCfg = 
{
    .priority           = 240, 
    .stacksize          = 512, 
    .messagequeuesize   = 8
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

void s_eom_timerman_tskproc_forever(EOMtask *rt, uint32_t msg);

static void s_eom_timerman_ProcessExpiry(eOmessage_t msg);
static eOresult_t s_eom_timerman_OnNewTimer(EOVtheTimerManager* tm, EOtimer *t);
static eOresult_t s_eom_timerman_AddTimer(EOVtheTimerManager* tm, EOtimer *t);
static eOresult_t s_eom_timerman_RemTimer(EOVtheTimerManager* tm, EOtimer *t);

static void s_eom_timerman_OnExpiry(osal_timer_t *osaltmr, void *tmr);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
 
static const char s_eobj_ownname[] = "EOMtheTimerManager";
 
static EOMtheTimerManager s_eom_thetimermanager = 
{
    .tmrman     = NULL,           // tmrman
    .tskproc    = NULL           // tskproc
}; 



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

 
extern EOMtheTimerManager * eom_timerman_Initialise(const eOmtimerman_cfg_t *tmrmancfg) 
{

    if(NULL != s_eom_thetimermanager.tmrman) 
    {
        // already initialised
        return(&s_eom_thetimermanager);
    }

    if(NULL == tmrmancfg)
    {
        tmrmancfg = &eom_timerman_DefaultCfg;
    }
    
    // trying to initialise with wrong params error
    eo_errman_Assert(eo_errman_GetHandle(), (0 != tmrmancfg->messagequeuesize), s_eobj_ownname, "tmrmancfg->messagequeuesize is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != tmrmancfg->stacksize), s_eobj_ownname, "tmrmancfg->stacksize is 0");
    eo_errman_Assert(eo_errman_GetHandle(), (0 != tmrmancfg->priority), s_eobj_ownname, "tmrmancfg->priority is 0");
    
    // i get a basic timer manager with onnew, add and rem functions proper for osal. and an EOMmutex (by the way ... eom_mutex_New() never returns NULL).
    s_eom_thetimermanager.tmrman = eov_timerman_hid_Initialise(s_eom_timerman_OnNewTimer, s_eom_timerman_AddTimer, s_eom_timerman_RemTimer, eom_mutex_New()); 

    // i prepare the task able to process actions associated to expiry of the timers 
    s_eom_thetimermanager.tskproc = eom_task_New(eom_mtask_MessageDriven,                          // type 
                                              tmrmancfg->priority,                                 // priority
                                              tmrmancfg->stacksize,                               // stacksize
                                              NULL,                                             // startup_fn 
                                              s_eom_timerman_tskproc_forever,                   // run_fn 
                                              tmrmancfg->messagequeuesize,                      // maxmessages: msg fifo can hold all timers 
                                              eok_reltimeINFINITE,                                  // timeoutorperiod
                                              NULL,
                                              sys_timerman,                               // nameofthetask_fn
                                              "timerman");                              
    
    
   
    return(&s_eom_thetimermanager);
}    

    
extern EOMtheTimerManager* eom_timerman_GetHandle(void) 
{
    if(NULL == s_eom_thetimermanager.tmrman) 
    {
        return(NULL);
    }
    
    return(&s_eom_thetimermanager);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

 
/*  @fn        static void s_eom_timerman_OnExpiry(void *osaltmr, void *tmr)
     @brief     this function is called by the expired osal timer to alert the ruling task. the osal automatically 
                reloads the timer if it was created as a periodic timer.
 **/    
static void s_eom_timerman_OnExpiry(osal_timer_t *osaltmr, void *tmr) 
{

#ifdef _EOM_TIMERMAN_EVENTS_DIRECTLY_SENT_BY_OSAL_

    EOtimer *t = (EOtimer*)tmr;

    if(eo_actypeEvent == t->onexpiry.actiontype)
    {
        osal_eventflag_set(t->onexpiry.data.evt.event, ((EOMtask*)t->onexpiry.data.evt.totask)->osaltask, osal_callerISR);

        if(EOTIMER_ONESHOT == t->mode) 
        {
            // sets dummy values for the timer, which is completed 
            eo_timer_hid_Reset_but_not_osaltime(t, eo_tmrstat_Completed); 
            //t->osaltimer = osaltimer;   
        }
    }
    else
    {
        eom_task_isrSendMessage(s_eom_thetimermanager.tskproc, (eOmessage_t)tmr);
    }


#else
    // EOtimer *eotimer = (EOtimer*)tmr;
    // if timer is expired .... make it clear by the executer task
    eom_task_isrSendMessage(s_eom_thetimermanager.tskproc, (eOmessage_t)tmr);
#endif
} 


static eOresult_t s_eom_timerman_OnNewTimer(EOVtheTimerManager* tm, EOtimer *t) 
{
    eOresult_t res = eores_NOK_generic;
    
    if((NULL == tm) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    if(NULL != t->envir.osaltimer)
    {
        // this timer has already been created ..... return error
        return(eores_NOK_generic);
    }

// no need to protect data structures of the timer manager    
//    if(eores_OK != eom_mutex_Take(tm->mutex, eok_reltimeINFINITE))
//    {
//        // cannot lock it ... bye bye
//         return(eores_NOK_generic);
//    }
    
    // create the osal timer
    t->envir.osaltimer = osal_timer_new();
    t->status = EOTIMER_IDLE;
    
    // it may be that osal does not have any more timers available.
    if(NULL != t->envir.osaltimer)
    {
        res = eores_OK;
    }
    else
    {
        res = eores_NOK_generic;
    }
    
    
//    // unlock the manager
//    eom_mutex_Release(tm->mutex);

    return(res);  
   
}


static eOresult_t s_eom_timerman_AddTimer(EOVtheTimerManager* tm, EOtimer *t) 
{
    eOresult_t res = eores_NOK_generic;
    osal_timer_timing_t   timing;
    osal_timer_onexpiry_t onexpi;

    
    if((NULL == tm) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    if(EOTIMER_RUNNING == t->status)
    {
        // this timer is already running ..... return error
        return(eores_NOK_generic);
    }

    if(NULL == t->envir.osaltimer)
    {
        // this timer does not have a osal timer ..... return error
        return(eores_NOK_generic);
    }

    
//    if(eores_OK != eov_mutex_Take(tm->mutex, eok_reltimeINFINITE)) 
    if(eores_OK != eom_mutex_Take(tm->mutex, eok_reltimeINFINITE))
    {
        // cannot lock it ... bye bye
         return(eores_NOK_generic);
    }
    
    // start the osal timer
    timing.startat  = t->startat; 
    timing.count    = t->expirytime; 
    timing.mode     = ( (EOTIMER_FOREVER == t->mode) ? (osal_tmrmodeFOREVER) : (osal_tmrmodeONESHOT) );
    onexpi.cbk      = s_eom_timerman_OnExpiry;
    onexpi.par      = t;

    res = (eOresult_t)osal_timer_start(t->envir.osaltimer, &timing, &onexpi, osal_callerTSK);
    
    
    // unlock the manager
    //eov_mutex_Release(tm->mutex);
    eom_mutex_Release(tm->mutex);

    return(res);  
   
}


static eOresult_t s_eom_timerman_RemTimer(EOVtheTimerManager* tm, EOtimer *t) 
{
    //osal_timer_t osaltimer = NULL;

    if((NULL == tm) || (NULL == t)) 
    {
         return(eores_NOK_nullpointer);
    }
  

  
    //if(eores_OK != eov_mutex_Take(tm->mutex, eok_reltimeINFINITE)) 
    if(eores_OK != eom_mutex_Take(tm->mutex, eok_reltimeINFINITE))
    {
        return(eores_NOK_generic);
    }

    // store it ... for later
    //osaltimer = t->envir.osaltimer;
    
    // stop the osal timer. operation is null safe.
    osal_timer_stop(t->envir.osaltimer, osal_callerTSK);

        
    // reset the values of the timer but do not set envir.osaltimer to NULL
    eo_timer_hid_Reset_but_not_osaltime(t, eo_tmrstat_Idle);
    // thus reassign the value of osaltimer
    //t->osaltimer = osaltimer;

    // unlock the manager
    // eov_mutex_Release(tm->mutex);
    eom_mutex_Release(tm->mutex);

    return(eores_OK);
}


// name of the task as it is shown in uvision
void sys_timerman(void *p)
{
    eom_task_Start(p);
}


/* @fn         static void s_eom_timerman_tskproc_forever(EOMtask *rt, uint32_t msg)
    @brief      this is the forever funtion of the rtos task.
                It is called when a message coming from osal is received.
                This functions simply calls s_eom_timerman_ProcessExpiry().
 **/
 
static void s_eom_timerman_tskproc_forever(EOMtask *rt, uint32_t msg)
{
    s_eom_timerman_ProcessExpiry(msg);
}
 


/* @fn         static void s_eom_timerman_ProcessExpiry(eOmessage_t msg)
    @brief      this function is called by the forever method of the internal task s_eom_thetimermanager.tskproc
                in order to manage the expiry of the rtos timer.
 
 **/
static void s_eom_timerman_ProcessExpiry(eOmessage_t msg) 
{
    // important ... the message contains the pointer to the target EOtimer
    EOtimer *t = (EOtimer *) msg; 
    //osal_timer_t osaltimer = NULL;
    
    // we dont have a valid pointer to a EOtimer
    if(NULL == t)
    {
        return;
    }

    //osaltimer = t->osaltimer;
 
    // get the mutex of the timer manager.  
//    if(eores_OK != eov_mutex_Take(s_eom_thetimermanager.tmrman->mutex, eok_reltimeINFINITE)) 
    if(eores_OK != eom_mutex_Take(s_eom_thetimermanager.tmrman->mutex, eok_reltimeINFINITE)) 
    {
        return;
    }
    
    // 1. perform the action

    eo_action_Execute(&t->onexpiry, eok_reltimeZERO);

                
    // 2. clear if one shot
    if(EOTIMER_ONESHOT == t->mode) 
    {
        // sets dummy values for the timer, which is completed 
        eo_timer_hid_Reset_but_not_osaltime(t, eo_tmrstat_Completed); 
        // but do not erase the osaltimer.
        //t->osaltimer = osaltimer;   
    }


    // release the mutex of the timer manager
    //eov_mutex_Release(s_eom_thetimermanager.tmrman->mutex);
    eom_mutex_Release(s_eom_thetimermanager.tmrman->mutex);
    
    return;
}

   

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





