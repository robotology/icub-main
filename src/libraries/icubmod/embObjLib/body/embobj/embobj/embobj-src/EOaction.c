
/* @file       EOaction.c
    @brief      This file keeps internal implementation of a action object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOVtask.h"





// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOaction.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOaction_hid.h" 


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
// commented out as it is not used
//static const char s_eobj_ownname[] = "EOaction";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOaction* eo_action_New()
{
    EOaction *retptr = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOaction), 1);
    
    retptr->actiontype = eo_actypeNONE;

    return(retptr);
}


extern eOresult_t eo_action_Clear(EOaction *p)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    p->actiontype = eo_actypeNONE;
    
    return(eores_OK);
}


extern eOresult_t eo_action_Copy(EOaction *p, const EOaction *src)
{
    if((NULL == p) || (NULL == src))
    {
        return(eores_NOK_nullpointer);
    }

    memcpy(p, src, sizeof(EOaction));
    
    return(eores_OK);
}


extern eOresult_t eo_action_SetEvent(EOaction *p, eOevent_t event, EOVtaskDerived *totask)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    p->actiontype = eo_actypeEvent;
    
    p->data.evt.event = event;
    p->data.evt.totask = totask;
    
    return(eores_OK);
}


extern eOresult_t eo_action_SetMessage(EOaction *p, eOmessage_t message, EOVtaskDerived *totask)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    p->actiontype = eo_actypeMessage;
    
    p->data.msg.message = message;
    p->data.msg.totask = totask;
    
    return(eores_OK);
}


extern eOresult_t eo_action_SetCallback(EOaction *p, eOcallback_t callback, void *arg, EOVtaskDerived *exectask)
{
    if(NULL == p)
    {
        return(eores_NOK_nullpointer);
    }
    
    p->actiontype = eo_actypeCallback;
    
    p->data.cbk.callback = callback;
    p->data.cbk.argument = arg;
    p->data.cbk.exectask = exectask;

    return(eores_OK);
}


extern eOactiontype_t eo_action_GetType(EOaction *p)
{
    if(NULL == p)
    {
        return(eo_actypeNONE);
    }

    return(p->actiontype);
}


extern eOresult_t eo_action_GetEvent(EOaction *p, eOevent_t *event, EOVtaskDerived **totask)
{
    if((NULL == p) || (NULL == event) || (NULL == totask))
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eo_actypeEvent != p->actiontype)
    {
        *event = 0;
        *totask = NULL;
        return(eores_NOK_generic);
    }
    
    *event = p->data.evt.event;
    *totask = p->data.evt.totask;
    
    return(eores_OK);
}


extern eOresult_t eo_action_GetMessage(EOaction *p, eOmessage_t *message, EOVtaskDerived **totask)
{
    if((NULL == p) || (NULL == message) || (NULL == totask))
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eo_actypeMessage != p->actiontype)
    {
        *message = 0;
        *totask = NULL;
        return(eores_NOK_generic);
    }
    
    *message = p->data.msg.message;
    *totask = p->data.msg.totask;
    
    return(eores_OK);
}


extern eOresult_t eo_action_GetCallback(EOaction *p, eOcallback_t *callback, void **arg, EOVtaskDerived **exectask)
{
    if((NULL == p) || (NULL == callback) || (NULL == arg) || (NULL == exectask))
    {
        return(eores_NOK_nullpointer);
    }
    
    if(eo_actypeCallback != p->actiontype)
    {
        *callback = NULL;
        *exectask = NULL;
        *arg      = NULL;
        return(eores_NOK_generic);
    }
    
    *callback = p->data.cbk.callback;
    *exectask = p->data.cbk.exectask;
    *arg      = p->data.cbk.argument;
    
    return(eores_OK);
}


extern eOresult_t eo_action_Execute(EOaction *act, eOreltime_t tout) 
{
    EOVtaskDerived *totask = NULL;
    eOresult_t res = eores_NOK_generic;

    // do action
    switch(eo_action_GetType(act)) 
    {
        case eo_actypeEvent: 
        {
            eOevent_t event = 0;
            eo_action_GetEvent(act, &event, &totask);
    
            if(NULL != totask) 
            {   // set an event to a task
                res = eov_task_tskSetEvent(totask, event);
            }
        } break;
        
        case eo_actypeMessage: 
        {
            eOmessage_t message = 0;
            eo_action_GetMessage(act, &message, &totask);

            if(NULL != totask)
            {   // send a message to a task. 
                 res = eov_task_tskSendMessage(totask, message, tout);
            }
        } break;

        case eo_actypeCallback: 
        {
            eOcallback_t callback = NULL;
            void *arg = NULL;
            eo_action_GetCallback(act, &callback, &arg, &totask);

            if(NULL != callback) 
            {   // request a task to execute a callback
                res = eov_task_tskExecCallback(totask, callback, arg, tout);
            }
        } break;

        default:
        {
        } break;
    }


    return(res);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




