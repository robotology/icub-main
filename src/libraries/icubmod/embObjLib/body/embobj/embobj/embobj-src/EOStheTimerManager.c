
/* @file       EOStheTimerManager.c
	@brief      This file implements internal implementation of the rtos timer manager singleton.
	@author     marco.accame@iit.it
    @date       08/04/2011
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
#include "EOSmutex.h"
#include "EOStheSystem_hid.h"





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheTimerManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOStheTimerManager_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------

const eOstimerman_cfg_t eos_timerman_DefaultCfg = 
{
    .timernum                           = 8
};


// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void s_eos_timerman_ProcessExpiry(EOtimer *t, uint64_t currtickoflife);
static eOresult_t s_eos_timerman_OnNewTimer(EOVtheTimerManager* tm, EOtimer *t);
static eOresult_t s_eos_timerman_AddTimer(EOVtheTimerManager* tm, EOtimer *t);
static eOresult_t s_eos_timerman_RemTimer(EOVtheTimerManager* tm, EOtimer *t);

static eOresult_t s_eos_timerman_expirytimelater_than(void *item, void *param);

static void s_eos_timerman_insert_timer(EOtimer *t);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
 
static const char s_eobj_ownname[] = "EOStheTimerManager";

static EOStheTimerManager s_eos_thetimermanager = 
{
    .tmrman                             = NULL,  
    .activetimers                       = NULL,
    .lastprocessedsystick               = 0,
    .tickperiod                         = 0,
    .tmptmrs                            = NULL,
    .numtimers                          = 0
}; 

// acemor-04aug2011: not sure if the following is still true. verify!

// the EOList activetimers is either empty (no timers to process) or has some pointers inside which hold the timers
// ordered in expiry delta time in the field tmr->counting arranged in differential way.
// suppose that at absolute time 100 we have 3 timers whcih expires at time 103, 110 and 115. the list contains
// pointers to 3 EOtimer where the counting are: [3, 7, 5].
// when we tick for time 101, we just decremnt the counting field of the head of list.
// thus teh content becomes: [2, 7, 5].
// suppose that in the same time 101, we wnat to add a tiemr tmr0 with expiry in 5 ticks at 106
// we loop the timers until the cumulative sum of the head and the next etc is higher than 5 ... it happend with the second entry, where
// 2+7 is 9 and is the first cumulative value to be higher than 5.
// at this stage we set the tmr0->counting to the value 5 - 2 = 3, with 2 the cumulative sum just lower than 5.
// we then change teh counting of teh second entry to counting -= tmr0->counting or 7-3=4
// and we insert tmr0 just befor the second entry.
// teh result is: [2, 3, 4, 5]. the result is correct because if we are at time 101 we shall have timers expiring at 101+2 = 103,
// at 101+2+3 = 106, at 101+2+3+4 = 110, at 101+2+3+4+5 = 115.
//
// counting:     the incremental time expressed in ticks (or usec??)
// expirytime:   keeps the first value of expiry so that periodic timers can retrigger with that.
// startat: keeps the absolute time of start in ticks or usec??
// envir.nextexpiry: keeps the 


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------

 
extern EOStheTimerManager * eos_timerman_Initialise(const eOstimerman_cfg_t *tmrmancfg) 
{
    EOSmutex *mtx = NULL;

    if(NULL != s_eos_thetimermanager.tmrman) 
    {
        // already initialised
        return(&s_eos_thetimermanager);
    }

    if(NULL == tmrmancfg)
    {
        tmrmancfg = &eos_timerman_DefaultCfg;
    }
    
    // trying to initialise with wrong params error
    eo_errman_Assert(eo_errman_GetHandle(), (0 != tmrmancfg->timernum), s_eobj_ownname, "tmrmancfg->timernum is 0");
    
    s_eos_thetimermanager.numtimers = tmrmancfg->timernum; 

//    if((NULL != tmrmancfg->hal_sys_criticalsection_take) && (NULL != tmrmancfg->hal_sys_criticalsection_release_fn) )
//    {
//        mtx = eos_mutex_New(tmrmancfg->hal_sys_criticalsection_take, tmrmancfg->hal_sys_criticalsection_release_fn);
//    }

    
    // i get a basic timer manager with add and rem functions proper for singletask env. And an EOSmutex 
    s_eos_thetimermanager.tmrman = eov_timerman_hid_Initialise(s_eos_timerman_OnNewTimer, s_eos_timerman_AddTimer, s_eos_timerman_RemTimer, mtx); 
    
    
    // i get activetimers list. it keeps pointers to EOtimer objects because we dont want to copy data.
    s_eos_thetimermanager.activetimers = eo_list_New(sizeof(EOtimer*), s_eos_thetimermanager.numtimers, NULL, 0, NULL, NULL);
    
    s_eos_thetimermanager.tickperiod = eos_sys_hid_tickperiodget();
    
    s_eos_thetimermanager.lastprocessedsystick = 0;

    s_eos_thetimermanager.tmptmrs = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOtimer*), s_eos_thetimermanager.numtimers);
    
  
    return(&s_eos_thetimermanager);
}    

    
extern EOStheTimerManager* eos_timerman_GetHandle(void) 
{
    if(NULL == s_eos_thetimermanager.tmrman) 
    {
        return(NULL);
    }
    
    return(&s_eos_thetimermanager);
}


extern void eos_timerman_Tick(EOStheTimerManager *p)
{
    EOlist *activetimers = NULL;
    EOtimer *t = NULL;
    uint64_t delta = 0;
    uint64_t currtickoflife = 0;
    EOtimer **ppt;
    
    if(NULL == p)
    {
        return;    
    }

    // delta is in ticks
    currtickoflife = eos_sys_hid_tickoflifeget();
    delta = currtickoflife - s_eos_thetimermanager.lastprocessedsystick;
    s_eos_thetimermanager.lastprocessedsystick = currtickoflife;

   
    
    // i lock the timer manager. 
    // actually there could be no need because ... either (a) or (b):
    // (a) if the _Tick() is called by an ISR and the mutex is correctly set, then the addtimer 
    //     and remtimer would have disabled the isr so this function would not have been called.
    // (b) if the _Tick() and any other EoTimer operation is not called by an ISR, then everything 
    //     is called by the single thread FOOP.
    // only problem is ... if someone puts a timer.start() or .stop() inside an isr. in this case the
    // addtimer or remtimer can interrupt a call of _Tick().
//    if(eores_NOK_timeout == eov_mutex_Take(s_eos_thetimermanager.tmrman->mutex, eok_reltimeZERO)) 
//   {
//        return(res_OK);
//   }  




    // i get the timer in front
    activetimers = s_eos_thetimermanager.activetimers;
    ppt = eo_list_Front(activetimers);
    // ppt is a pointer to the stored object (a EOtimer*), thus ppt holds a EOtimer**
    t = (NULL == ppt) ? (NULL) : (*ppt);
    
    
    if(NULL == t)
    {
        return;
    }
    
    if(t->counting > delta)
    {
        t->counting -= delta;
        return;
    }
    else
    {
        // we have at least one expired timer
        
        while((NULL != t) && (t->counting <= delta))
        {
            // reduce the delta for next timer.
            delta -= t->counting;
            
            // remove timer t from head of list
            eo_list_PopFront(activetimers);
            
            // execute timer t and reload if periodic
            s_eos_timerman_ProcessExpiry(t, currtickoflife);
            
            // get the new front

            ppt = eo_list_Front(activetimers);
            t = (NULL == ppt) ? (NULL) : (*ppt);
        }
        
        return;
 
    }
 
 
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern void eos_hid_timerman_Synch(EOStheTimerManager *p, uint64_t oldtick, uint64_t newtick)
{
    // we modify only the absolute timers and leave the ordinary timers as they are.
    // put the absolute timers temporarily inside s_eos_thetimermanager.tmptmrs[].

    EOlist *activetimers = s_eos_thetimermanager.activetimers;
    uint8_t i;
    EOtimer *t = NULL;
    EOtimer *tt = NULL;
    void * tmp = NULL;
    uint8_t num_tmptmrs = 0;
    EOlistIter *li = NULL; 
    EOlistIter *pi = NULL;
    uint64_t tickperiod = eos_sys_hid_tickperiodget();


    // 1. adjust the s_eos_thetimermanager.lastprocessedsystick

    if(newtick > oldtick)
    {   // we go forward, thus we increment
        s_eos_thetimermanager.lastprocessedsystick += (newtick-oldtick);
    }
    else
    {   // we go in the past, thus we decrement
        s_eos_thetimermanager.lastprocessedsystick -= (oldtick-newtick);
    }


    // stage 1: remove from the list every absolute timer

    for(li = eo_list_Begin(activetimers); li != NULL; li = eo_list_Next(activetimers, li))
    {
        tmp = eo_list_At(activetimers, li);
        t = (NULL == tmp) ? (NULL) : (*((EOtimer **) tmp));
        if((NULL != t) && (eok_abstimeNOW != t->startat))
        {   // found an absolute timer t. 
            // store its prev
            pi = eo_list_Prev(activetimers, li);
            // i remove it from the list, but before adjust the delta time of the timer after it
            tmp = eo_list_At(activetimers, eo_list_Next(activetimers, li));
            tt = (NULL == tmp) ? (NULL) : (*((EOtimer **) tmp));
            if(NULL != tt)
            {
                // increment the delay of the prev one
                tt->counting += (t->counting);
            }

             // now we can remove the timer t
            eo_list_Erase(activetimers, li);

            // but i dont reset it because i want to keep its information and i store it.
            s_eos_thetimermanager.tmptmrs[num_tmptmrs++] = t;

            // adjust iterator
            li = pi;
        }
    }

    // stage 2: insert them back in the list by calling s_eos_timerman_AddTimer()

    for(i=0; i<num_tmptmrs; i++)
    {
        t = s_eos_thetimermanager.tmptmrs[i];

        // need to insert inside t all info required to make the timer start again correctly
        // i start from the top with invariant data
        t->status       = EOTIMER_IDLE;
        t->mode         = t->mode;
        t->startat      = (eok_abstimeNOW == t->startat) ? (eok_abstimeNOW) : (t->startat * tickperiod); 
        t->expirytime   = t->expirytime * tickperiod;
        // osaltimer, onexpiry are not changed.

        // counting is computed by the function


        // action is not changed ...


        

        // fill timing. 
        // if periodic it is easy: we leave the same values. if singleshot in the future ok. if in the past we set curtime+1.

        if(EOTIMER_FOREVER == t->mode)
        {   // periodic: easy as we leave the same values

        }
        else if(t->envir.nextexpiry > newtick)
        {   // singleshot in the future: we set startat with the desired expiry time
            // already done.
        }
        else
        {   // singleshot in the past: we execute it at next tick
            t->startat          = tickperiod*(newtick+1);
            t->expirytime       = 0;
        }


        // now clear the timer and start it
        t->status = EOTIMER_IDLE;
        s_eos_timerman_AddTimer(s_eos_thetimermanager.tmrman, t);
    }

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static eOresult_t s_eos_timerman_OnNewTimer(EOVtheTimerManager* tm, EOtimer *t)
{
    
    if((NULL == tm) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }

    

// no need to protect data structures of the timer manager    
//    if(eores_OK != eov_mutex_Take(s_eos_thetimermanager.tmrman->mutex, eok_reltimeZERO))
//    {
//        // cannot lock it ... bye bye
//         return(eores_NOK_generic);
//    }
    
    // dont have any osal timer, so put it NULL. but set status as idle
    t->envir.nextexpiry = 0;
    t->status = EOTIMER_IDLE;
    
    
//    // unlock the manager
//    eov_mutex_Release(s_eos_thetimermanager.tmrman->mutex);

    return(eores_OK);  

}


static eOresult_t s_eos_timerman_AddTimer(EOVtheTimerManager* tm, EOtimer *t) 
{
    eOresult_t res = eores_NOK_generic;
    uint32_t delta_shot_others = 0;
    uint64_t absol_tickoflife = 0;
    uint64_t tmp64 = 0;
    
    if((NULL == tm) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    if(EOTIMER_RUNNING == t->status)
    {
        // this timer is already running ..... return error
        return(eores_NOK_generic);
    }
    
    // transform from micro-seconds to ticks
    t->startat  = (eok_abstimeNOW == t->startat) ? (eok_abstimeNOW) : (t->startat / eos_sys_hid_tickperiodget());
    t->expirytime   /= eos_sys_hid_tickperiodget();
    
    // get some values
    delta_shot_others = (EOTIMER_ONESHOT == t->mode) ? (0) : (t->expirytime);
    absol_tickoflife = eos_sys_hid_tickoflifeget();
    
    
    if(eok_abstimeNOW != t->startat)
    {
        if(((t->startat+t->expirytime) <= absol_tickoflife) && (0 == delta_shot_others))
        {
            return(eores_NOK_generic);
        }
    }
    else if(0 == (t->expirytime+delta_shot_others))
    {
        return(eores_NOK_generic);
    } 


   // now we have either a valid incremental or a future absolute (at least in one of its periods)

   // fill values in the timer t
   if(eok_abstimeNOW == t->startat)
   {
        t->counting = (0 != t->expirytime) ? (t->expirytime) : (delta_shot_others);
        t->expirytime = delta_shot_others;
        t->envir.nextexpiry = absol_tickoflife + t->counting;
   }
   else
   {
        // first absolute occurrence
        tmp64 = t->startat + ((0 != t->expirytime) ? (t->expirytime) : (delta_shot_others));

        // if it is in the past, find smallest multiple bigger than absol_tickoflife
        if(tmp64 <= absol_tickoflife)
        {
            tmp64 = absol_tickoflife + delta_shot_others - ((absol_tickoflife - tmp64) % delta_shot_others);
        }
        

        // now tmp64 keeps the first occurrence in absolute ticks. 
        // i use tmp64 to compute the delta time of the first occurrence an i put it in counting
        t->counting = tmp64 - absol_tickoflife;
        // in expirytime we have a constant: the period for others occurrences in ticks
        t->expirytime = delta_shot_others;     
        // in startat we have a constant: the absolute beginning of the timer in ticks
        t->startat = t->startat;
        t->envir.nextexpiry = absol_tickoflife + t->counting;
      
   }


    
    
//    if(eores_OK != eov_mutex_Take(s_eos_thetimermanager.tmrman->mutex, eok_reltimeZERO))
//    {
//        // cannot lock it ... bye bye
//         return(eores_NOK_generic);
//    }
    

    // put the timer inside the list of timers using incremental time.

   
    if(eo_list_Size(s_eos_thetimermanager.activetimers) < eo_list_Capacity(s_eos_thetimermanager.activetimers))
    {
        // set the tick of next expiry
//        t->envir.nextexpiry = absol_tickoflife + t->counting;
        // ok, the timer manager can handle one or more timers ... i place it in ascending order of expiry
        s_eos_timerman_insert_timer(t);
        res = eores_OK;    
    }

    
//    // unlock the manager
//    eov_mutex_Release(s_eos_thetimermanager.tmrman->mutex);

    return(res);  

}


static eOresult_t s_eos_timerman_RemTimer(EOVtheTimerManager* tm, EOtimer *t) 
{
    EOtimer *tt = NULL; 
    EOlistIter *li = NULL;

    if((NULL == tm) || (NULL == t)) 
    {
         return(eores_NOK_nullpointer);
    }
  

  
//    if(eores_OK != eov_mutex_Take(s_eos_thetimermanager.tmrman->mutex, eok_reltimeZERO))
//    {
//        return(eores_NOK_generic);
//    }
    
    // search for the timer t in the list
    li = eo_list_FindItem(s_eos_thetimermanager.activetimers, &t);
    
    if(NULL != li)
    {
        // get the timer after the one to be removed ...
        tt = *((EOtimer**) eo_list_At(s_eos_thetimermanager.activetimers, eo_list_Next(s_eos_thetimermanager.activetimers, li)));
        if(NULL != tt)
        {
            // increment the delay of the next one
            tt->counting += (t->counting);
        }
        
        // now we can remove the timer
        eo_list_Erase(s_eos_thetimermanager.activetimers, li);
    }
    
    // in any case we reset the values of the removed timer
    eo_timer_hid_Reset(t, eo_tmrstat_Idle);

//    // unlock the manager
//    eov_mutex_Release(s_eos_thetimermanager.tmrman->mutex);

    return(eores_OK);
}


static void s_eos_timerman_ProcessExpiry(EOtimer *t, uint64_t currtickoflife) 
{
  
    // 1. perform the action

     
    eo_action_Execute(&t->onexpiry, eok_reltimeZERO);

                
    // 2. clear if one shot
    if(EOTIMER_ONESHOT == t->mode) 
    {
        // sets dummy values for the timer, which is completed 
        eo_timer_hid_Reset(t, eo_tmrstat_Completed);    
    }
    else
    {
        // retrigger the timer
        t->counting = t->expirytime;
        // set the tick of next expiry
        t->envir.nextexpiry = currtickoflife + t->expirytime;
        s_eos_timerman_insert_timer(t);
    }

    
    return;
}


static eOresult_t s_eos_timerman_expirytimelater_than(void *item, void *param)
{
    // item is a pointer to the object contained (a EOtimer*), hence it holds a EOtimer**
    // param hold a pointer to a uint32_t
    EOtimer *p = *((EOtimer**)item);
    uint32_t *v = ((uint32_t*)param);

    if(p->counting > (*v))
    {
        return(eores_OK);
    }
    else
    {
        (*v) = (*v) - p->counting;
        return(eores_NOK_generic);
    }
}


static void s_eos_timerman_insert_timer(EOtimer *t)
{
    EOlistIter *li = NULL;
    EOlist *activetimers = s_eos_thetimermanager.activetimers;
    

    li = eo_list_Find(activetimers, s_eos_timerman_expirytimelater_than, &t->counting);

    // set status to running
    t->status = EOTIMER_RUNNING; 
   
    if(NULL == li)
    {
        // no element in list or no element which comes after my timer, thus ... add to back the pointer to the hold item
        eo_list_PushBack(activetimers, &t);
    }
    else
    {
        // found a timer, tt pointed by iterator li, which expires after t. i need to decrement the delta counting of tt
        // of the delta counting of t and then insert t before tt.
        EOtimer *tt = *((EOtimer**) eo_list_At(activetimers, li));
        tt->counting -= (t->counting);
        eo_list_Insert(activetimers, li, &t);   
    }


}

   

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





