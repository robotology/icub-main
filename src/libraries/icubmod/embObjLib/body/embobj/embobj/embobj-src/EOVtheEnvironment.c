
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

#include "EOVtheEnvironment.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVtheEnvironment_hid.h" 


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

static const char s_eobj_ownname[] = "EOVtheEnvironment";

static EOVtheEnvironment s_eov_environment = 
{
    .vtable     = {NULL}
};


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
//extern EOVtheEnvironment* eov_env_GetHandle(void) 
//{
//    if(NULL == s_eov_environment.vtable[0])
//    {    
//        return(NULL);
//    }
//    return(&s_eov_environment);
//}


extern eOresult_t eov_env_SharedData_Synchronise(EOVtheEnvironmentDerived *p)
{
    EOVtheEnvironment *env;
    eOres_fp_voidp_t fptr;

    if(NULL == (env = eo_common_getbaseobject(p))) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_voidp_t)env->vtable[VF00_env_shareddata_sync];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_env_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p));
}

extern eOresult_t eov_env_ProcessOffset_Get(EOVtheEnvironmentDerived *p, uint32_t *offset)
{
    EOVtheEnvironment *env;
    eOres_fp_voidp_uint32p_t fptr;

    if(NULL == (env = eo_common_getbaseobject(p))) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_voidp_uint32p_t)env->vtable[VF01_env_proc_offset_get];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_env_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, offset));
}


extern eOresult_t eov_env_RunningEprocess_Get(EOVtheEnvironmentDerived *p, eEprocess_t *eproc)
{
    EOVtheEnvironment *env;
    eOres_fp_voidp_uint8p_t fptr;

    if(NULL == (env = eo_common_getbaseobject(p))) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_voidp_uint8p_t)env->vtable[VF04_env_eproc_get];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_env_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, eproc));
}

extern eOresult_t eov_env_IPnetwork_Get(EOVtheEnvironmentDerived *p, const eEipnetwork_t **ipnet)
{
    EOVtheEnvironment *env;
    eOres_fp_voidp_cvoidpp_t fptr;

    if(NULL == (env = eo_common_getbaseobject(p))) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_voidp_cvoidpp_t)env->vtable[VF02_env_ipnet_get];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_env_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, (const void**)ipnet));
}

extern eOresult_t eov_env_CANnetworks_Get(EOVtheEnvironmentDerived *p, const eEcannetwork_t **cannets, uint8_t *numnets)
{
    EOVtheEnvironment *env;
    eOres_fp_voidp_cvoidpp_uint8p_t fptr;

    if(NULL == (env = eo_common_getbaseobject(p))) 
    {
        return(eores_NOK_nullpointer);    
    }
    
    fptr = (eOres_fp_voidp_cvoidpp_uint8p_t)env->vtable[VF03_env_cannets_get];

    // if p is not NULL, ftpr cannot be NULL because we have verified that in eov_env_hid_Initialise(), thus ...
    // just call the method initialised by the derived object
    return(fptr(p, (const void**)cannets, numnets));
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOVtheEnvironment * eov_env_hid_Initialise(eOres_fp_voidp_t shareddata_sync, 
                                                  eOres_fp_voidp_uint32p_t proc_offset_get,
                                                  eOres_fp_voidp_cvoidpp_t ipnet_get,
                                                  eOres_fp_voidp_cvoidpp_uint8p_t cannets_get,
                                                  eOres_fp_voidp_uint8p_t eproc_get                                                 
                                                  )
{

    if(NULL != s_eov_environment.vtable[0]) 
    {
        // already initialised
        return(&s_eov_environment);
    }


    eo_errman_Assert(eo_errman_GetHandle(), NULL != shareddata_sync, s_eobj_ownname, "shareddata_sync() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != proc_offset_get, s_eobj_ownname, "proc_offset_get() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != ipnet_get, s_eobj_ownname, "ipnet_get() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != cannets_get, s_eobj_ownname, "cannet_get() is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), NULL != eproc_get, s_eobj_ownname, "eproc_get() is NULL");


    s_eov_environment.vtable[VF00_env_shareddata_sync]          = shareddata_sync;
    s_eov_environment.vtable[VF01_env_proc_offset_get]          = proc_offset_get;
    s_eov_environment.vtable[VF02_env_ipnet_get]                = ipnet_get;
    s_eov_environment.vtable[VF03_env_cannets_get]              = cannets_get;
    s_eov_environment.vtable[VF04_env_eproc_get]                = eproc_get;


    return(&s_eov_environment);   
} 


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




