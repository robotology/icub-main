
/* @file       EOumlsm.c
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

#include "EOumlsm.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOumlsm_hid.h" 


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

 
static void s_eo_umlsm_Specialise(EOumlsm *p, const eOumlsm_cfg_t * c);

 
static uint8_t s_eo_umlsm_ConsumeOneEvent(EOumlsm *const p, eOumlsmEvent_t event);

static uint8_t s_eo_umlsm_GetDeepestStateFrom(eOumlsm_cfg_t * p, uint8_t index);

static eOresult_t s_eo_umlsm_Verify(eOumlsm_cfg_t * p);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOumlsm";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern EOumlsm * eo_umlsm_New(const eOumlsm_cfg_t * cfg) 
{
    EOumlsm *retptr = NULL;
    
    // verify that we have a non NULL cfg
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != cfg), s_eobj_ownname, "cfg is NULL");    

    // i get the memory for the object. no need to check versus NULL because the memory pool already does it
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOumlsm), 1);
    
    // now the obj has valid memory. i need to initialise it with other user-defined data, by means of cfg.
    s_eo_umlsm_Specialise(retptr, cfg);

    return(retptr);    
}


extern void eo_umlsm_Start(EOumlsm *p) 
{
    const eOumlsmState_t *state = NULL;
    const eOumlsmState_t *targetstate = NULL;
    const eOumlsm_cfg_t *cfg = NULL;
    uint8_t targetowners_num = 0;
    uint8_t j = 0;

    if(NULL == p)
    {
        return;
    }
    
    if(1 == p->initialised) 
    {
        return;
    }
    
    // get rom config
    cfg = p->cfg;
    
    targetstate = &(cfg->states_table[p->activestate]);
    targetowners_num = targetstate->owners_number;
    
    
    // execute on entry starting from top-down
    
    for(j=0; j<targetowners_num; j++) 
    {
        state = &(cfg->states_table[targetstate->owners_table[targetowners_num -1 -j]]);
         
        if(NULL != state->on_entry_fn) 
        {
            state->on_entry_fn(p);
        }
         
    }
         
    p->initialised = 1;
}



extern uint8_t eo_umlsm_ProcessEvent(EOumlsm *p, eOumlsmEvent_t ev, eOumlsmConsumeMode_t consume) 
{
    uint8_t retval = 0;
    eOumlsmEvent_t event = eo_umlsm_evNONE;
    uint8_t consumed = 0;
 
    
    if(NULL == p)
    {
        return(retval);
    }
    
    if(0 == p->initialised) 
    {
        eo_umlsm_Start(p);
    }
    
    if(consume == eo_umlsm_consume_ONE) 
    {
        return(s_eo_umlsm_ConsumeOneEvent(p, ev));
    }
    else if(consume == eo_umlsm_consume_UPTO08) 
    {
    
        for(;;) 
        {

            if(eo_umlsm_evNONE == (event = eo_umlsm_GetInternalEvent(p))) 
            {
                // ok, i consume the passed argument and reset it
                event = ev;
                ev = eo_umlsm_evNONE;
            }

            if(eo_umlsm_evNONE == event) 
            {
                // process events until there is none left in queue or in the argument passed 
                return(retval);
            }

            // increment retval only if the event was triggered
            retval += s_eo_umlsm_ConsumeOneEvent(p, event);

            consumed ++;
    
            // consumed too many
            if(consumed >= eo_umlsm_consume_UPTO08) 
            {
                return(retval);    
            }
        }            
        
    }
    
    return(retval);
}
    

extern eOumlsmEvent_t eo_umlsm_GetInternalEvent(EOumlsm *p) 
{
    eOumlsmEvent_t ret = eo_umlsm_evNONE;
    eOresult_t r;
     
    if(NULL == p)
    {
        return(eo_umlsm_evNONE);
    }    
    
    if(NULL != p->internal_event_fifo) 
    {
        // extract events from queue until we find a not zero event or until we find it empty.
        do 
        {
            r = eo_fifobyte_Get(p->internal_event_fifo, &ret, eok_reltimeZERO);
            if(eores_OK == r) 
            {
                eo_fifobyte_Rem(p->internal_event_fifo, eok_reltimeZERO);
            }
        } 
        while((eo_umlsm_evNONE == ret) && (eores_OK == r));

        // if result was not ok, then we return a zero event
        if(eores_OK != r) 
        {
                 // ret is not changed by fifobyte, but i do the following fro clarity
                ret = eo_umlsm_evNONE;
        }

    }
    
    return(ret);
}


extern void eo_umlsm_Reset(EOumlsm *p) 
{
    const eOumlsm_cfg_t *cfg = NULL;    

    if(NULL == p)
    {
        return;
    }
    
    // get rom config
    cfg = p->cfg;    
    
    // set initialised to false
    p->initialised = 0;
    
    // if defined, call reset from configuration. it must clear the ram as well
    if(NULL != cfg->resetdynamicdata_fn) 
    {
        cfg->resetdynamicdata_fn(p);
    }

    // if exists, clear the fifo of internal events
    if(NULL != p->internal_event_fifo) 
    {
        eo_fifobyte_Clear(p->internal_event_fifo, eok_reltimeZERO);
    }
    
    // go to initial state
    eo_umlsm_Start(p);
}


extern eOresult_t eo_umlsm_PutInternalEvent(EOumlsm *p, eOumlsmEvent_t event)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }    
    
    if((NULL == p->internal_event_fifo) || (eo_umlsm_evNONE == event)) 
    {
        return(eores_NOK_generic);
    }
    
    return(eo_fifobyte_Put(p->internal_event_fifo, event, eok_reltimeZERO));
}

extern void* eo_umlsm_GetDynamicData(EOumlsm *p)
{
    return(p->ram);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

 
static uint8_t s_eo_umlsm_ConsumeOneEvent(EOumlsm *const p, eOumlsmEvent_t event) 
{
    uint8_t                         i = 0U;
    uint8_t                         j = 0U;
    uint8_t                         sourceowners_num = 0U;
    uint8_t                         targetowners_num = 0U;
    uint8_t                         executeit = 0;
    uint8_t                         indextonextstate = 0U;
    eOumlsm_cfg_t                   *cfg = NULL; 
    const eOumlsmTransition_t    *transition = NULL;
    const eOumlsmState_t         *sourcestate = NULL;
    //const eOumlsmcfgState_t       *firingstate = NULL;
    const eOumlsmState_t           *targetstate = NULL; 
    const eOumlsmState_t           *state1 = NULL;
    const eOumlsmState_t           *state2 = NULL;
    const eOumlsmTransition_t      *firingtransition = NULL;
       

    if(eo_umlsm_evNONE == event) 
    {
        return(0);
    }

    // get rom config
    cfg = p->cfg;  
 
    // i get the source state and the number of its owners
    sourcestate = &(cfg->states_table[p->activestate]);
    sourceowners_num = sourcestate->owners_number;
    

    // i look for a firing transition in the transitions table of the state starting from the 
    // active state and going bottom-up
    
    // we quit the loops as soon as we find a firing transition, which is initially NULL 
    for(j=0; (j<sourceowners_num) && (NULL == firingtransition); j++) 
    {
        state1 = &(cfg->states_table[sourcestate->owners_table[j]]);
        
        for(i=0; (i<state1->transitions_number) && (NULL == firingtransition); i++ ) 
        {
            transition = &(state1->transitions_table[i]);
            
            // it may be that we have a state which has a transition table with a single NULL pointer.
            // that is the case of a owner state which only executes on-entry and on-exit depending on 
            // transitions of its sub-states.
            
            // and also it is safe to check against NULL.
            
            if(NULL != transition) 
            {
                if(event == transition->trigger) 
                {
                    // ok, the received event matches the trigger of the transition. 
                    // what about guard conditions?
                    
                    if(NULL == transition->guard_fn) 
                    {
                        // there is no guard, thus the match is complete
                        firingtransition = transition;
                        //firingstate = state1;
                        break;
                    }
                    else if(eobool_true == transition->guard_fn(p)) 
                    { 
                        // there is a true condition on the guard, thus teh match is complete
                        firingtransition = transition;
                        //firingstate = state1;
                        break;                    
                    }
                    else 
                    {
                        // else ... nothing. the match is not complete
                        // we DONT break because there can be another transition with a trigger matching the event
                        // we implement the choice state with multiple transition with tehy same trigger but different guards
                    }
                }
            
            }
            
        }
        
    
    }
    
        

        
    if(NULL == firingtransition) 
    {
        // no firing transition found. i quit
        return(0);
    }
    

    // found a firing transition .... follow it
    

    // search for the deepest state pointed by the entry point
    // we also check for a possible human bug in filling the tables
    
    indextonextstate = s_eo_umlsm_GetDeepestStateFrom(cfg, firingtransition->next);
    
#if 0    
    indextonextstate = firingtransition->next;
    targetstate = &(cfg->states_table[indextonextstate]);
    i = 0; 
    while(sm_NotAnIndex != targetstate->initial_substate) 
    {
        // we are not working with ram, thus we dont exit the while loop only if the
        // states tables have been incorrectly filled. much better to find the bug early,
        // thus i dont put any safe exit in case of mistake.
        indextonextstate        = targetstate->initial_substate;
        targetstate             = &(cfg->states_table[indextonextstate]);
        errman_Assert(errman_GetHandle(), i < sm_OWNERS_maxnumber);
        i++;
    }  
#endif

    targetstate = &(cfg->states_table[indextonextstate]);
    
    // finally sets the number of its owners
    targetowners_num = targetstate->owners_number;
    
    
        
    // ON EXIT
    
    // execute on exit from any of the source states only if the target state does not belong to any of them
    // i begin with lowest owner because on-exit functions are called bottom-up.
     
    for(j=0; j<sourceowners_num; j++) 
    {
        state1 = &(cfg->states_table[sourcestate->owners_table[j]]);
        
        if(NULL != state1->on_exit_fn) 
        { 
            // ok, this state has an exit. we execute it unless this state is the same as the target one or as one
            // of its owners.
             
            executeit = 1;
             
            for(i=0U; i<targetowners_num; i++) 
            {
                state2 = &(cfg->states_table[targetstate->owners_table[i]]);
            
                if(state1 == state2) 
                {
                    executeit = 0;
                    // ok it is the same as one of its owners, then we dont search any more
                    break;
                }
            }
             
            if(1 == executeit) 
            {
                state1->on_exit_fn(p);
            }
            
        }                      
    }
             
         
         
    // ON TRANSITION
     
    // execute on transition from firing state to the target one
    if(NULL != firingtransition->on_transition_fn) 
    {
        firingtransition->on_transition_fn(p);
    }

    
    // SET STATE

    // set the currente state with the target state
    p->activestate = indextonextstate;
//    p->state = targetstate;



    // ON ENTRY
    
    
    // execute on entry from any of the target states only if the source state does not belong to any of them
    // i begin with highest owner because on-entry functions are called top-down.
         
    for(j=0; j<targetowners_num; j++) 
    {
        state1 = &(cfg->states_table[targetstate->owners_table[targetowners_num -1 -j]]);
         
        if(NULL != state1->on_entry_fn) 
        {
            // ok, this state has an entry. we execute it unless this state is the same as the source one or as one
            // of its owners.
             
            executeit = 1;
             
            for(i=0; i<sourceowners_num; i++) 
            {
                state2 = &(cfg->states_table[sourcestate->owners_table[i]]);
            
                if(state1 == state2) 
                {
                     executeit = 0;
                     // ok it is the same as one of its owners, then we dont search any more
                     break;
                }
            }
             
            if(1 == executeit) 
            {
                state1->on_entry_fn(p);
            }
            
        }                      
    }
     
    return(1);
}
 


static void s_eo_umlsm_Specialise(EOumlsm *p, const eOumlsm_cfg_t * c) 
{
    uint8_t size = 0;
    
    
    // verify consistency of the user-defined cfg data structure. humans can fail!
    s_eo_umlsm_Verify(c); //eov_umlsmcfg_hid_Verify(c);

    // fill state machine object with user-define data structure.
    
    // cfg
    p->cfg = c;    
 
    // ram
    size = c->sizeofdynamicdata;
    p->ram = (0 == size) ? (NULL) : (eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, size, 1));
    
    
    // initialised
    p->initialised = 0;
    
    // activestate. the function navigates into the deepest entry point and checks
    
    // search for the deepest state pointed by the entry point
    // also check for a possible human bug in filling the rom tables
    p->activestate = s_eo_umlsm_GetDeepestStateFrom(c, c->initial_state);    
    
    
    // internal_event_fifo: we dont use any mutex because the sm must be used by a single task
    size = c->internal_event_fifo_size;
    p->internal_event_fifo = (0 == size) ? (NULL) : eo_fifobyte_New(size, NULL);

    // reset dynamic data 
    if(NULL != c->resetdynamicdata_fn) 
    {
        c->resetdynamicdata_fn(p);
    }

      
//    // reset the event queue.
//    if(NULL != p->internal_event_fifo) 
//    {
//        eo_fifobyte_Clear(p->internal_event_fifo, eok_reltimeZERO);
//    }
    
}


static uint8_t s_eo_umlsm_GetDeepestStateFrom(eOumlsm_cfg_t * p, uint8_t index)
{
    const eOumlsmState_t *targetstate = NULL;
    uint8_t i = 0;
    // search for the deepest state pointed by index
    // we also check for a possible human bug in filling the tables
    
 
    targetstate = &(p->states_table[index]);
    i = 0; 
    while(eok_uint08dummy != targetstate->initial_substate) 
    {
        // we are not working with ram, thus we dont exit the while loop only if the
        // states tables have been incorrectly filled. much better to find the bug early,
        // thus i dont put any safe exit in case of mistake.
        index                   = targetstate->initial_substate;
        targetstate             = &(p->states_table[index]);
        eo_errman_Assert(eo_errman_GetHandle(), i < eov_umlsm_OWNERS_maxnumber, s_eobj_ownname, "too many owners");
        i++;
    }  

    return(index);
}


static eOresult_t s_eo_umlsm_Verify(eOumlsm_cfg_t * p)
{
    uint8_t i = 0;
    
 
    // verify the integrity of the owners table.
    for(i=0; i<p->states_number; i++) 
    {
        eo_errman_Assert(eo_errman_GetHandle(), i == p->states_table[i].owners_table[0], s_eobj_ownname, "incorrect owners table");
    } 

 
    return(eores_OK);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------


