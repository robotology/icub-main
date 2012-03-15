
/* @file       EOtimer.c
    @brief      This file implements internal implementation of a timer object.
    @author     marco.accame@iit.it
    @date       08/04/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "EOaction_hid.h" // to allow access to the full type
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOVtheTimerManager.h"

#include "EOVtheSystem.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtimer.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtimer_hid.h" 


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
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOtimer";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOtimer* eo_timer_New(void) 
{
    EOtimer *retptr = NULL;
    EOVtheTimerManager *p = NULL;
    eOresult_t res = eores_NOK_generic;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOtimer), 1);

    // now the obj has valid memory. i need to initialise it with user-defined data,
    // sets dummy values for the timer
    eo_timer_hid_Reset(retptr, eo_tmrstat_Idle);
    
    // i get the timer manager of the system.
    p = eov_timerman_GetHandle();
    
    if(NULL == p) 
    {
         //return(eores_NOK_nullpointer);
         eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_timer_New() cannot get the timer manager");
    }

    res = eov_timerman_OnNewTimer(p, retptr);

    if(eores_OK != res)
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "in eo_timer_New() the timer manager cannot init the timer");
    }


    return(retptr);
}


extern eOresult_t eo_timer_Start(EOtimer *t, eOabstime_t startat, eOreltime_t countdown, eOtimerMode_t mode, EOaction *action) 
{
    eOresult_t res = eores_NOK_busy;
    EOVtheTimerManager *p = NULL;
    
    // i get the timer manager of the system.
    p = eov_timerman_GetHandle();
    
    if((NULL == p) || (NULL == action)) 
    {
         //return(eores_NOK_nullpointer);
         eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_timer_Start() found a NULL pointer as argument");
    }
    
    if((eok_abstimeNOW != startat) && (0 == countdown))
    {
        //return(eores_NOK_generic);
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "eo_timer_Start() has zero expiry time and startat not eok_abstimeNOW");
    }


    // take exclusive access to the timer manager. result can be ok, nok_nullpointer, or ... nok_timeout.
    if(eores_NOK_timeout == eov_timerman_Take(p, eok_reltimeINFINITE)) 
    {
         return(res);
    }


    // if it is a oneshot synchrotimer, then make it start only if action is in the future.
    if((eok_abstimeNOW != startat) && (eo_tmrmode_ONESHOT == mode)) 
    { 
        if((startat + countdown) <= eov_sys_LifeTimeGet(eov_sys_GetHandle()))
        {
            // release the timer manager
            eov_timerman_Release(p);
            return(eores_NOK_generic);
        }
    }

    // start the timer only if it is not running now
    if(eo_tmrstat_Running != eo_timer_GetStatus(t)) 
    {
        // put inside the timer only the values that the timer manager needs to know. 
        // the timer manager will then complete the others according to its needs
        t->status           = EOTIMER_IDLE; 
        t->mode             = (eo_tmrmode_ONESHOT == mode) ? (EOTIMER_ONESHOT) : (EOTIMER_FOREVER);
        t->startat          = startat;
        t->expirytime       = countdown;
        memcpy(&t->onexpiry, action, sizeof(EOaction));

         
        // now give the timer to the timer manager. the function returns error if the 
        // manager is full and cannot accept timers any more. 
        // if successfull, the timer manager will set status of timer to running and will keep it 
        // in this status until:
        // 1. the timer is one shot and expires
        // 2. someone removes it.
        // after either one of these two cases the timer manager will release the timer and sets 
        // its status to completed or idle.
        res = eov_timerman_AddTimer(p, t);
    }    

    // release the timer manager
    eov_timerman_Release(p);

    return(res);
}


extern eOresult_t eo_timer_Stop(EOtimer *t) 
{
    eOresult_t res = eores_NOK_busy;
    EOVtheTimerManager *p = NULL;
    
    // i get the timer manager of the system.
    p = eov_timerman_GetHandle();
    
    if((NULL == p) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);
    }

    // take exclusive access to the timer manager. result can be ok, nok_nullpointer, or ... nok_timeout.
    if(eores_NOK_timeout == eov_timerman_Take(p, eok_reltimeINFINITE)) 
    {
        return(res);
    }

    if(eo_tmrstat_Running == eo_timer_GetStatus(t)) 
    {
        // the timer is running, thus ... i can remove it

        // remove the timer from the manager.
        res = eov_timerman_RemTimer(p, t);

    }
    else 
    {
        // timer has already expired or was never started ..... what to do?
        // ....
        // reset the values of the timer to idle. there is no need to remove it from the timer manager 
        // since the timer manager does not own it anymore.
        eo_timer_hid_Reset(t, eo_tmrstat_Idle);
        res = eores_OK;  
    }

    // release the timer manager
    eov_timerman_Release(p);

    return(res);
}


extern eOabstime_t eo_timer_Remaining(EOtimer *t) 
{
    eOabstime_t tm = 0;
    
//    if(NULL != t) 
//    {
//        // we dont protect because there is no need to do that in reading a byte in ram
//        //eov_timerman_Take(eov_timerman_GetHandle(), eok_reltimeZERO);
//        //tm = t->counting;
//        //eov_timerman_Release(eov_timerman_GetHandle());
//    }

    return(tm);
}


extern eOtimerStatus_t eo_timer_GetStatus(EOtimer *t) 
{
    eOtimerStatus_t st = eo_tmrstat_Idle;
    uint8_t sss = 0;
    
    if(NULL != t) 
    {    
        // we dont protect because there is no need to do that in reading a byte in ram
        //eov_timerman_Take(eov_timerman_GetHandle(), eok_reltimeZERO);
        sss = t->status;
        //eov_timerman_Release(eov_timerman_GetHandle());
    }
    
    st = (EOTIMER_IDLE == sss) ? (eo_tmrstat_Idle) : 
         ((EOTIMER_RUNNING == sss) ? (eo_tmrstat_Running) : (eo_tmrstat_Completed) );

    return(st);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern void eo_timer_hid_Reset(EOtimer *t, eOtimerStatus_t stat) 
{
    uint8_t v = (eo_tmrstat_Idle == stat) ? (EOTIMER_IDLE) : 
                ((eo_tmrstat_Running == stat) ? (EOTIMER_RUNNING) : (EOTIMER_COMPLETED));
    if(NULL != t) 
    {
        t->startat              = eok_abstimeNOW;
        t->expirytime           = 0;
        t->counting             = 0;
        t->status               = v;
        t->mode                 = 0;
        t->dummy                = 0;
        t->envir.osaltimer      = NULL;
        t->envir.nextexpiry     = 0;
        memset(&(t->onexpiry), 0, sizeof(EOaction));
        t->onexpiry.actiontype  = eo_actypeNONE;
    }
}


extern void eo_timer_hid_Reset_but_not_osaltime(EOtimer *t, eOtimerStatus_t stat) 
{
    uint8_t v = (eo_tmrstat_Idle == stat) ? (EOTIMER_IDLE) : 
                ((eo_tmrstat_Running == stat) ? (EOTIMER_RUNNING) : (EOTIMER_COMPLETED));
    if(NULL != t) 
    {
        t->status               = v;
        t->mode                 = 0;
        t->dummy                = 0;
        t->startat              = eok_abstimeNOW;
        t->expirytime           = 0;
        t->counting             = 0;
//        t->envir.osaltimer            = NULL;
// and not even envir.nextexpiry as it stays in teh same memory
        memset(&(t->onexpiry), 0, sizeof(EOaction));
        t->onexpiry.actiontype  = eo_actypeNONE;
    }
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section




// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




