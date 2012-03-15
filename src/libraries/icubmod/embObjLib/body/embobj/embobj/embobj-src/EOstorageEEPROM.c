
/* @file       EOstorageEEPROM.c
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

#include "hal.h"

#include "EOVstorage_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOstorageEEPROM.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOstorageEEPROM_hid.h" 


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

static eOresult_t s_eo_strgeeprom_set(EOstorageEEPROM *s, uint32_t offset, uint32_t size, const void *data);
static eOresult_t s_eo_strgeeprom_get(EOstorageEEPROM *s, uint32_t offset, uint32_t size, void *data);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static const char s_eobj_ownname[] = "EOstorageEEPROM";


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


 
extern EOstorageEEPROM* eo_strgeeprom_New(hal_eeprom_t id, uint32_t baseaddress, uint32_t capacity, const void *defvalue, EOVmutexDerived *mtx)
{
    EOstorageEEPROM *retptr = NULL;  
 
    // i get the memory for the object
    retptr = eo_mempool_GetMemory(eo_mempool_GetHandle(), eo_mempool_align_32bit, sizeof(EOstorageEEPROM), 1);

    eo_errman_Assert(eo_errman_GetHandle(), (0 != capacity), s_eobj_ownname, "capacity cannot be zero");
    
    eo_errman_Assert(eo_errman_GetHandle(), (hal_eeprom_i2c_01 == id), s_eobj_ownname, "only hal_eeprom_i2c_01 is supported");
    
    eo_errman_Assert(eo_errman_GetHandle(), (hal_true == hal_eeprom_address_is_valid(id, baseaddress)), s_eobj_ownname, "baseaddress is out of range");
    
    eo_errman_Assert(eo_errman_GetHandle(), (hal_true == hal_eeprom_address_is_valid(id, baseaddress+capacity-1)), s_eobj_ownname, "baseaddress+capacity-1 is out of range");
    
     

    // get the base object
    retptr->storage = eov_strg_hid_New(id, capacity, defvalue, mtx);
    
    // pass to it the virtual table
    
    eov_strg_hid_SetVTABLE(retptr->storage, 
                          (eOres_fp_voidp_uint32_uint32_cvoidp_t) s_eo_strgeeprom_set, 
                          (eOres_fp_voidp_uint32_uint32_voidp_t) s_eo_strgeeprom_get);


    // now initialise the derived object
    retptr->baseaddress = baseaddress;
    
    if(hal_res_OK != hal_eeprom_init(id, NULL))
    {
        eo_errman_Assert(eo_errman_GetHandle(), 0, s_eobj_ownname, "cannot initialise eeprom");
    }
    
 
    return(retptr);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static eOresult_t s_eo_strgeeprom_set(EOstorageEEPROM *s, uint32_t offset, uint32_t size, const void *data)
{
    // no need for check vs NULL
    hal_result_t res = hal_eeprom_write((hal_eeprom_t)s->storage->id, s->baseaddress+offset, size, (void*)data);
     
    return((hal_res_OK == res) ? (eores_OK) : (eores_NOK_generic));
}


static eOresult_t s_eo_strgeeprom_get(EOstorageEEPROM *s, uint32_t offset, uint32_t size, void *data)
{
    // no need for check vs NULL
    hal_result_t res = hal_eeprom_read((hal_eeprom_t)s->storage->id, s->baseaddress+offset, size, data);
     
    return((hal_res_OK == res) ? (eores_OK) : (eores_NOK_generic));
}






// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




