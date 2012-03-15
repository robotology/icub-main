
/* @file       EOsm.c
    @brief      This file implements internal implementation of a state machine object.
    @author     marco.accame@iit.it
    @date       09/01/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "string.h"
#include "EoCommon.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOsm.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOsm_hid.h" 


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

 
static void s_eo_sm_Specialise(EOsm *p, const eOsm_cfg_t * c);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOsm";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern EOsm * eo_sm_New(const eOsm_cfg_t * cfg) 
{
    EOsm *retptr = NULL;
    
    // verify that we have a non NULL cfg
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != cfg), s_eobj_ownname, "cfg is NULL");    

    // i get the memory for the object. no need to check versus NULL because the memory pool already does it
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOsm), 1);
    
    // now the obj has valid memory. i need to initialise it with other user-defined data, by means of cfg.
    s_eo_sm_Specialise(retptr, cfg);
    
    return(retptr);    
}


extern eOresult_t eo_sm_Start(EOsm *p)
{
    const eOsmState_t *currstate = NULL;

    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }

    if(0 == p->started)
    {
        p->started = 1;
        p->latestevent = eo_sm_evNONE;

        currstate = &(p->cfg->states[p->cfg->initstate]);

        // execute on-entry
        if(NULL != currstate->on_entry_fn)
        {
            currstate->on_entry_fn(p);
        }

    }

    return(eores_OK);

}


extern eOresult_t eo_sm_ProcessEvent(EOsm *p, eOsmEvent_t ev) 
{
    EOsmStateQuickInfo_t *quickinfoonstate = NULL;
    const eOsmState_t *currstate = NULL;
    const eOsmState_t *nextstate = NULL;
    const eOsmTransition_t *transition = NULL;
 
    
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    if(ev >= p->cfg->maxevts)
    {
        return(eores_NOK_generic);
    }

    if(0 == p->started)
    {
        eo_sm_Start(p);
    }
    
    quickinfoonstate = &(p->statequickinfo[p->activestate]);
    
   
    if(0 == (quickinfoonstate->evtmask & (0x00000001 << ev)))
    {
        // no event for this state.
        return(eores_NOK_nodata);
    }
    
    // set the latest event
    p->latestevent = ev;
    
    // there is a transition. 
    transition = &(p->cfg->transitions[quickinfoonstate->transindices[ev]]);

    // the current state is 
    currstate = &(p->cfg->states[p->activestate]);

    // the next state is
    nextstate = &(p->cfg->states[transition->next]);


    // execute on-exit
    if((currstate != nextstate) && (NULL != currstate->on_exit_fn))
    {
        currstate->on_exit_fn(p);
    }

    
    // execute the on-transition
    if(NULL != transition->on_transition_fn)
    {
        transition->on_transition_fn(p);
    }
    
    // move to next state
    p->activestate = transition->next;
   

    // execute on-entry
    if((currstate != nextstate) && (NULL != nextstate->on_entry_fn))
    {
        nextstate->on_entry_fn(p);
    }

   
    return(eores_OK);
}
    

extern void eo_sm_Reset(EOsm *p) 
{

    p->activestate = p->cfg->initstate;

    p->started = 0;

   
    if(NULL != p->cfg->resetdynamicdata_fn) 
    {
        p->cfg->resetdynamicdata_fn(p);
    } 

    
}

extern void* eo_sm_GetDynamicData(EOsm *p)
{
    if(NULL == p)
    {
        return(NULL);
    }

    return(p->ram);
}

extern uint8_t eo_sm_GetActiveState(EOsm *p)
{
    if(NULL == p)
    {
        return(EOK_uint08dummy);
    }

    return(p->activestate);
}

extern eOsmEvent_t eo_sm_GetLatestEvent(EOsm *p)
{
    if(NULL == p)
    {
        return(eo_sm_evNONE);
    }

    return(p->latestevent);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static void s_eo_sm_Specialise(EOsm *p, const eOsm_cfg_t * c) 
{
 
    const eOsmTransition_t *tr = NULL;
    EOsmStateQuickInfo_t *st = NULL;
    uint8_t i = 0;
    
    
    // fill state machine object with user-define data structure.
    
    p->cfg = c;
    p->started = 0;  
    
    // activestate. 
    p->activestate = c->initstate;
    p->latestevent = eo_sm_evNONE;


    


    // statequickinfo: get memory. the memory is zero initialised. 
    // IMPORTANT: every evtmask must be zero.
    p->statequickinfo = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOsmStateQuickInfo_t), c->nstates);
    // also for transindices
    for(i=0; i<c->nstates; i++)
    {
        p->statequickinfo[i].evtmask = 0; // it is a redundant instruction.
        p->statequickinfo[i].transindices = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_08bit, 1, c->maxevts);
    }
    

    // we map the transitions in ram into the statequickinfo
    for(i=0; i<c->ntrans; i++)
    {
        tr = &c->transitions[i];
        // tr must point to a valid location .... we cannot do much to verify that. however, we verify its content.
        eo_errman_Assert(eo_errman_GetHandle(), 
                         (tr->curr < c->nstates) && (tr->next < c->nstates) && (tr->evt < c->maxevts), 
                         s_eobj_ownname, "cfg is not correct"); 
        
        st = &p->statequickinfo[tr->curr];
        
        // the evtmask keeps a bit in pos j-th if the j-th event triggers a transition
        st->evtmask |= (0x00000001 << tr->evt);
        // the j-th event triggers transition number st->transindices[j] in cfg->transitions.
        st->transindices[tr->evt] = i; 
    }
    
    
    // ram
    p->ram = (0 == c->sizeofdynamicdata) ? (NULL) : (eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, c->sizeofdynamicdata, 1));
    
    // init
    if(NULL != p->cfg->init_fn) 
    {
        p->cfg->init_fn(p);
    }

    // reset
    eo_sm_Reset(p);
    
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------


