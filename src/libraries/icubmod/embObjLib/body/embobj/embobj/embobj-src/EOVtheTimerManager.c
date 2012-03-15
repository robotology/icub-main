
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"

#include "EOlist.h"
#include "EOtimer.h"
#include "EOtheErrorManager.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheTimerManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheTimerManager_hid.h" 


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

static const char s_eobj_ownname[] = "EOVtheTimerManager";

static EOVtheTimerManager s_eov_timermanager = 
{
    EO_INIT(.vtable)        {NULL}, 
    EO_INIT(.mutex)         NULL  
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOVtheTimerManager* eov_timerman_GetHandle(void) 
{
    if(NULL == s_eov_timermanager.vtable[0])
    {    
        return(NULL);
    }
    return(&s_eov_timermanager);
}


extern eOpurevirtual eOresult_t eov_timerman_OnNewTimer(EOVtheTimerManager* p, EOtimer *t)
{
    eOres_fp_tmrmanp_tmrp_t fptr;


    if((NULL == p) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_tmrmanp_tmrp_t)p->vtable[VF00_onnewtimer];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_timerman_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, t));
}


extern eOresult_t eov_timerman_AddTimer(EOVtheTimerManager* p, EOtimer *t) 
{
    eOres_fp_tmrmanp_tmrp_t fptr;

    if((NULL == p) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_tmrmanp_tmrp_t)p->vtable[VF01_addtimer];

    // if p is not NULL, ftpr cannot be NULL because we have verifeid that in eov_timerman_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, t));
}


extern eOresult_t eov_timerman_RemTimer(EOVtheTimerManager* p, EOtimer *t) 
{
    eOres_fp_tmrmanp_tmrp_t fptr;

    if((NULL == p) || (NULL == t)) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_tmrmanp_tmrp_t)p->vtable[VF02_remtimer];

    // if p is not NULL, ftpr cannot be NULL because we have verifeid that in eov_timerman_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, t));
}


extern eOresult_t  eov_timerman_Take(EOVtheTimerManager* p, eOreltime_t tout) 
{
    return(eov_mutex_Take(s_eov_timermanager.mutex, tout));
}


extern eOresult_t eov_timerman_Release(EOVtheTimerManager *p) 
{
    return(eov_mutex_Release(s_eov_timermanager.mutex));
}    


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



extern EOVtheTimerManager * eov_timerman_hid_Initialise(eOres_fp_tmrmanp_tmrp_t onnewtimer_fn,
                                                        eOres_fp_tmrmanp_tmrp_t addtimer_fn, 
                                                        eOres_fp_tmrmanp_tmrp_t remtimer_fn,
                                                        EOVmutexDerived *mutex)
{

    if(NULL != s_eov_timermanager.vtable[0]) 
    {
        // already initialised
        return(&s_eov_timermanager);
    }


    eo_errman_Assert(eo_errman_GetHandle(), NULL != onnewtimer_fn, s_eobj_ownname, "onnewtimer_fn() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != addtimer_fn, s_eobj_ownname, "addtimer_fn() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != remtimer_fn, s_eobj_ownname, "remtimer_fn() is NULL");

    s_eov_timermanager.vtable[VF00_onnewtimer]          = onnewtimer_fn;
    s_eov_timermanager.vtable[VF01_addtimer]            = addtimer_fn;
    s_eov_timermanager.vtable[VF02_remtimer]            = remtimer_fn;
    
    // i copy the mutex. i also accept a NULL mutex.
    s_eov_timermanager.mutex = mutex;


    return(&s_eov_timermanager);   
}    


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




