
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

#include "EOVmutex.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVmutex_hid.h" 


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

static const char s_eobj_ownname[] = "EOVmutex";



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern eOresult_t eov_mutex_Take(EOVmutexDerived *d, eOreltime_t tout) 
{   
    EOVmutex *mutex;
    eOres_fp_voidp_uint32_t fptr;

    
    mutex = eo_common_getbaseobject(d);
	
	if(NULL == mutex) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get take function
    fptr = (eOres_fp_voidp_uint32_t)mutex->vtable[VF00_take]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. it cant be NULL
    return(fptr(d, tout));
}


extern eOresult_t eov_mutex_Release(EOVmutexDerived *d) 
{
	EOVmutex *mutex;
    eOres_fp_voidp_t fptr;
    
    mutex = eo_common_getbaseobject(d);

	if(NULL == mutex) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get release function
    fptr = (eOres_fp_voidp_t)mutex->vtable[VF01_release]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    // call funtion of derived object. it cant be NULL
    return(fptr(d));
	
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


extern EOVmutex* eov_mutex_hid_New(void) 
{
	EOVmutex *retptr = NULL;	

	// i get the memory for the object
	retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOVmutex), 1);

	// now the obj has valid memory. i need to initialise it with user-defined data
    
    // vtable
    retptr->vtable[VF00_take]           = NULL;
    retptr->vtable[VF01_release]        = NULL;
    // other stuff


	return(retptr);	
}

extern eOresult_t eov_mutex_hid_SetVTABLE(EOVmutex *p, eOres_fp_voidp_uint32_t v_take, eOres_fp_voidp_t v_release)
{

    eo_errman_Assert(eo_errman_GetHandle(), (NULL != v_take), s_eobj_ownname, "v_tke is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != v_release), s_eobj_ownname, "v_release is NULL");

    p->vtable[VF00_take]            = v_take;
    p->vtable[VF01_release]         = v_release;

    return(eores_OK);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty section





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




