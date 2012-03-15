
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTORAGEEEPROM_H_
#define _EOSTORAGEEEPROM_H_



/** @file       EOstorageEEPROM.h
    @brief      This header file implements public interface to a fake storage object.
    @author     marco.accame@iit.it
    @date       02/15/2011
**/

/** @defgroup eo_strgeeprom Object EOstorageEEPROM
    The EOstorageEEPROM object ce3cecece
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVstorage.h"
#include "hal.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



/** @typedef    typedef struct EOfakeStorage_hid EOstorageEEPROM
    @brief      EOstorageEEPROM is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOstorageEEPROM_hid EOstorageEEPROM;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOstorageEEPROM* eo_strgeeprom_New(uint32_t id, uint32_t capacity, const void *defvalue, EOVmutexDerived *mtx)
    @param      id              the unique id of the storage object
    @param      capacity        the capacity of the storage
    @param      defvalue        the default value of the storage
    @brief      Creates a new fake storage object. 
    @return     The pointer to the required object.
 **/
extern EOstorageEEPROM* eo_strgeeprom_New(hal_eeprom_t id, uint32_t baseaddress, uint32_t capacity, const void *defvalue, EOVmutexDerived *mtx);



/** @}            
    end of group eo_strgeeprom  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

