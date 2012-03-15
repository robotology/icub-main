
/* @file       EOMmutex.c
	@brief      This file implements internal implementation of a MEE mutex object.
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
#include "EOtheErrorManager.h"
#include "EOVmutex_hid.h"

// we use osal
#include "osal.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOMmutex.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOMmutex_hid.h" 


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

// virtual
static eOresult_t s_eom_mutex_take(void *p, eOreltime_t tout);
// virtual
static eOresult_t s_eom_mutex_release(void *p);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOMmutex";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern EOMmutex* eom_mutex_New(void) 
{
    EOMmutex *retptr = NULL;    

    // i get the memory for the multitask mutex object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOMmutex), 1);
    
    // i get the base mutex
    retptr->mutex = eov_mutex_hid_New();

    // init its vtable
    eov_mutex_hid_SetVTABLE(retptr->mutex, s_eom_mutex_take, s_eom_mutex_release); 
    
    // i get a new ral mutex
    retptr->osalmutex = osal_mutex_new();

    // need to check because osal may return NULL
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != retptr->osalmutex), s_eobj_ownname, "osal cannot give a mutex");
    
    return(retptr);    
}


extern eOresult_t eom_mutex_Take(EOMmutex *m, eOreltime_t tout)
{
    if(NULL == m)
    {
        return(eores_NOK_nullpointer);
    }
    
    return(s_eom_mutex_take(m, tout));
}


extern eOresult_t eom_mutex_Release(EOMmutex *m)
{
    if(NULL == m)
    {
        return(eores_NOK_nullpointer);
    }
    
    return(s_eom_mutex_release(m));
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static eOresult_t s_eom_mutex_take(void *p, eOreltime_t tout) 
{
    EOMmutex *m = (EOMmutex *)p;
    // p it is never NULL because the base function calls checks it before calling this function, then osal will
    // check m->osalmutex vs NULL
    return((eOresult_t)osal_mutex_take(m->osalmutex, tout));
}


static eOresult_t s_eom_mutex_release(void *p) 
{
    EOMmutex *m = (EOMmutex *)p;
    // p it is never NULL because the base function calls checks it before calling this function, then osal will
    // check m->osalmutex vs NULL
    return((eOresult_t)osal_mutex_release(m->osalmutex));
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



