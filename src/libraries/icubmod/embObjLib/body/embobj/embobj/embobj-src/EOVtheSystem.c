
// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"

#include "EOtheErrorManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheSystem.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheSystem_hid.h" 


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

static void s_eov_sys_dummy_stop(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOVtheSystem";

static EOVtheSystem s_eov_system = 
{
    EO_INIT(.vtable)    {NULL}
};




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOVtheSystem* eov_sys_GetHandle(void) 
{
    if(NULL == s_eov_system.vtable[VF00_start])
    {    
        return(NULL);
    }
    return(&s_eov_system);
}



extern EOVtaskDerived* eov_sys_GetRunningTask(EOVtheSystem* p)
{
    eOvoidp_fp_void_t fptr;

    if(NULL == p) 
    {
        return(0);    
    }

    fptr = (eOvoidp_fp_void_t)p->vtable[VF01_getcurrtask];
 

    // just call the method initialised by the derived object
    return(fptr());

    
}


extern eOabstime_t eov_sys_LifeTimeGet(EOVtheSystem* p)
{
    eOuint64_fp_void_t fptr;
    if(NULL == p) 
    {
        return(0);    
    }
    
    fptr = (eOuint64_fp_void_t)p->vtable[VF02_timeget];

    // just call the method initialised by the derived object
    return(fptr());
}


extern eOresult_t eov_sys_LifeTimeSet(EOVtheSystem* p, eOabstime_t ltime)
{
    eOvoid_fp_uint64_t fptr;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOvoid_fp_uint64_t)p->vtable[VF03_timeset];


    // just call the method initialised by the derived object
    fptr(ltime);

    return(eores_OK);
}


extern eOresult_t eov_sys_NanoTimeGet(EOVtheSystem* p, eOnanotime_t *nt)
{
    eOuint64_fp_void_t fptr;

    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOuint64_fp_void_t)p->vtable[VF04_nanotimeget];

    // just call the method initialised by the derived object

    *nt = fptr();

    return(eores_OK);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



extern EOVtheSystem * eov_sys_hid_Initialise(const eOmempool_cfg_t *mpcfg,
                                             const eOerrman_cfg_t *emcfg,
                                             eOres_fp_voidfpvoid_t          start, 
                                             eOvoidp_fp_void_t              getcurrtask,
                                             eOuint64_fp_void_t             timeget,
                                             eOvoid_fp_uint64_t             timeset,
                                             eOuint64_fp_void_t             nanotimeget,
                                             eOvoid_fp_void_t               stop
                                             )
{


    if(NULL != s_eov_system.vtable[VF00_start]) 
    {
        // already initialised
        return(&s_eov_system);
    }

       
    // trying to initialise with NULL functions ?
    eo_errman_Assert(eo_errman_GetHandle(), NULL != start, s_eobj_ownname, "start() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != getcurrtask, s_eobj_ownname, "getcurrtask() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != timeget, s_eobj_ownname, "timeget() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != timeset, s_eobj_ownname, "timeset() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != nanotimeget, s_eobj_ownname, "nanotimeget() is NULL");

    // i copy the functions
    s_eov_system.vtable[VF00_start]         = (eOres_fp_voidfpvoid_t)start;
    s_eov_system.vtable[VF01_getcurrtask]   = (eOvoidp_fp_void_t)getcurrtask;
    s_eov_system.vtable[VF02_timeget]       = (eOuint64_fp_void_t)timeget;
    s_eov_system.vtable[VF03_timeset]       = (eOvoid_fp_uint64_t)timeset;
    s_eov_system.vtable[VF04_nanotimeget]   = (eOuint64_fp_void_t)nanotimeget;

    s_eov_system.vtable[VF04_stop]          =  (NULL != stop) ? (stop) : s_eov_sys_dummy_stop;


    // initialise error manager
    eo_errman_Initialise(emcfg);
    
    // initialise memory pool
    eo_mempool_Initialise(mpcfg);




    return(&s_eov_system);
    
}  


extern eOresult_t eov_sys_hid_Start(EOVtheSystem *p, void (*init_fn)(void))
{
    eOres_fp_voidfpvoid_t fptr;
    
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_voidfpvoid_t)p->vtable[VF00_start];


    // just call the method initialised by the derived object

    fptr(init_fn);

    return(eores_OK);
} 


extern eOresult_t eov_sys_Stop(EOVtheSystem *p)
{
    eOvoid_fp_void_t fptr;
    
    if(NULL == p) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOvoid_fp_void_t)p->vtable[VF04_stop];


    // just call the method initialised by the derived object

    fptr();

    return(eores_OK);
} 


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 


static void s_eov_sys_dummy_stop(void)
{
    ;
}

 
// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




