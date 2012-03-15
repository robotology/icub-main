
/* @file       EOfakeStorage.c
    @brief      This file implements internal implementation of a datagram socket object.
    @author     marco.accame@iit.it
    @date       12/24/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"

#include "EOVstorage_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOfakeStorage.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOfakeStorage_hid.h" 


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

static eOresult_t s_eo_fakestrg_set(EOfakeStorage *s, uint32_t offset, uint32_t size, const void *data);
static eOresult_t s_eo_fakestrg_get(EOfakeStorage *s, uint32_t offset, uint32_t size, void *data);

static void * s_eo_fakestrg_get_storagememory(EOVstorage *strg);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOfakeStorage";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOfakeStorage* eo_fakestrg_New(uint32_t id, uint32_t capacity, const void *defvalue, EOVmutexDerived *mtx)
{
    EOfakeStorage *retptr = NULL;  
    eov_strg_memmap_t *strgmem = NULL;    

    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOfakeStorage), 1);

    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity cannot be zero");

    // get the base object
    retptr->storage = eov_strg_hid_New(id, capacity, defvalue, mtx);
    
    // pass to it the virtual table
    
    eov_strg_hid_SetVTABLE(retptr->storage, 
                          (eOres_fp_voidp_uint32_uint32_cvoidp_t) s_eo_fakestrg_set, 
                          (eOres_fp_voidp_uint32_uint32_voidp_t) s_eo_fakestrg_get);


    // now initialise the derived object
    
    strgmem = s_eo_fakestrg_get_storagememory(retptr->storage);
    
    if((NULL == strgmem) || (id != strgmem->id))
    {
        // create new memory partition, mark it with id and capacity, set its content to defvalue
        
        strgmem = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(uint32_t) + sizeof(uint32_t) + capacity, 1);
        strgmem->id         = id;
        strgmem->capacity   = capacity;

        memcpy(strgmem->data, defvalue, strgmem->capacity);
    
    }
 
    // assign the partition to the object
    retptr->ram = (void*)strgmem;
 
    return(retptr);

}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static eOresult_t s_eo_fakestrg_set(EOfakeStorage *s, uint32_t offset, uint32_t size, const void *data)
{
    eov_strg_memmap_t *strgmem = (eov_strg_memmap_t*)s->ram;  
    memcpy(&strgmem->data[offset], data, size);
    
    return(eores_OK);
}


static eOresult_t s_eo_fakestrg_get(EOfakeStorage *s, uint32_t offset, uint32_t size, void *data)
{
    eov_strg_memmap_t *strgmem = (eov_strg_memmap_t*)s->ram;  
    memcpy(data, &strgmem->data[offset], size);
  
    return(eores_OK);
}

static void * s_eo_fakestrg_get_storagememory(EOVstorage *strg)
{
    // it should search if strg->id has a memory location
    return(NULL);
}





// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




