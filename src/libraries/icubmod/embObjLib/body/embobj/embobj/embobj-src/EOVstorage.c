
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

#include "EOVstorage.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOVstorage_hid.h" 


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

static const char s_eobj_ownname[] = "EOVstorage";



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern eOpurevirtual eOresult_t eov_strg_Reset(EOVstorageDerived *d)
{   
    EOVstorage *strg;
    eOres_fp_voidp_uint32_uint32_cvoidp_t fptr;

    eOresult_t res;

    
    strg = eo_common_getbaseobject(d);
	
	if(NULL == strg) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get take function
    fptr = (eOres_fp_voidp_uint32_uint32_cvoidp_t)strg->vtable[VF00_set]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    eov_mutex_Take(strg->mutex, eok_reltimeINFINITE);
    // call funtion of derived object. it cant be NULL
    res = fptr(d, 0, strg->capacity, strg->defval);
    eov_mutex_Release(strg->mutex);

    return(res);
}


extern eOpurevirtual eOresult_t eov_strg_Set(EOVstorageDerived *d, uint32_t offset, uint32_t size, const void *data)
{
    EOVstorage *strg;
    eOres_fp_voidp_uint32_uint32_cvoidp_t fptr;

    eOresult_t res;

    
    strg = eo_common_getbaseobject(d);
	
	if(NULL == strg) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get take function
    fptr = (eOres_fp_voidp_uint32_uint32_cvoidp_t)strg->vtable[VF00_set]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    eov_mutex_Take(strg->mutex, eok_reltimeINFINITE);
    // call funtion of derived object. it cant be NULL
    res = fptr(d, offset, size, data);
    eov_mutex_Release(strg->mutex);

    return(res);

}


extern eOpurevirtual eOresult_t eov_strg_Get(EOVstorageDerived *d, uint32_t offset, uint32_t size, void *data)
{
    EOVstorage *strg;
    eOres_fp_voidp_uint32_uint32_voidp_t fptr;

    eOresult_t res;

    
    strg = eo_common_getbaseobject(d);
	
	if(NULL == strg) 
	{
		return(eores_NOK_nullpointer); 
	}

    // get take function
    fptr = (eOres_fp_voidp_uint32_uint32_voidp_t)strg->vtable[VF01_get]; 
	
//    if(NULL == fptr)
//	{
//		return(eores_NOK_nullpointer);     
//	}

    eov_mutex_Take(strg->mutex, eok_reltimeINFINITE);
    // call funtion of derived object. it cant be NULL
    res = fptr(d, offset, size, data);
    eov_mutex_Release(strg->mutex);

    return(res);

}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------

extern EOVstorage* eov_strg_hid_New(uint32_t id, uint32_t capacity, const void *defval, EOVmutexDerived *mtx) 
{
	EOVstorage *retptr = NULL;	

	// i get the memory for the object
	retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOVstorage), 1);

	// now the obj has valid memory. i need to initialise it with user-defined data
    
    // vtable
    retptr->vtable[VF00_set]            = NULL;
    retptr->vtable[VF01_get]            = NULL;
    // other stuff
    retptr->id                          = id;
    retptr->capacity                    = capacity;
    retptr->defval                      = defval;
    retptr->mutex                       = mtx;


	return(retptr);	
}

extern eOresult_t eov_strg_hid_SetVTABLE(EOVstorage *p, eOres_fp_voidp_uint32_uint32_cvoidp_t v_set, eOres_fp_voidp_uint32_uint32_voidp_t v_get)
{

    eo_errman_Assert(eo_errman_GetHandle(), (NULL != v_set), s_eobj_ownname, "v_set is NULL");
    eo_errman_Assert(eo_errman_GetHandle(), (NULL != v_get), s_eobj_ownname, "v_get is NULL");

    p->vtable[VF00_set]         = v_set;
    p->vtable[VF01_get]         = v_get;

    return(eores_OK);
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty section





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




