
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheCallbackManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheCallbackManager_hid.h" 


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

static const char s_eobj_ownname[] = "EOVtheCallbackManager";

static EOVtheCallbackManager s_eov_callbackmanager = 
{
    EO_INIT(.vtable)        {NULL}, 
    EO_INIT(.task)          NULL  
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOVtheCallbackManager* eov_callbackman_GetHandle(void) 
{
    if(NULL == s_eov_callbackmanager.vtable[0])
    {    
        return(NULL);
    }
    return(&s_eov_callbackmanager);
}


extern eOpurevirtual eOresult_t eov_callbackman_Execute(EOVtheCallbackManager *p, eOcallback_t cbk, void *arg, eOreltime_t tout)
{
    eOres_fp_vcbkmanp_cbk_voidp_uint32_t fptr;


    if((NULL == p) || (NULL == cbk)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_vcbkmanp_cbk_voidp_uint32_t)p->vtable[VF00_execute];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_callbackman_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, cbk, arg, tout));
}


extern eOpurevirtual EOVtaskDerived * eov_callbackman_GetTask(EOVtheCallbackManager *p)
{
    if((NULL == p)) 
    {
        return(NULL);    
    }
    
    return(p->task);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOVtheCallbackManager * eov_callbackman_hid_Initialise(eOres_fp_vcbkmanp_cbk_voidp_uint32_t execute_fn, EOVtaskDerived *task)
{

    if(NULL != s_eov_callbackmanager.vtable[0]) 
    {
        // already initialised
        return(&s_eov_callbackmanager);
    }


    eo_errman_Assert(eo_errman_GetHandle(), NULL != execute_fn, s_eobj_ownname, "execute_fn() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != task, s_eobj_ownname, "task is NULL");

    s_eov_callbackmanager.vtable[VF00_execute]              = execute_fn;
    s_eov_callbackmanager.task                              = task;


    return(&s_eov_callbackmanager);   
} 

extern void eov_callbackman_hid_SetTask(EOVtheCallbackManager *p, EOVtaskDerived *task)
{
    s_eov_callbackmanager.task = task;
}   


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




