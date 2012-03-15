
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOFAKESTORAGE_H_
#define _EOFAKESTORAGE_H_



/** @file       EOfakeStorage.h
    @brief      This header file implements public interface to a fake storage object.
    @author     marco.accame@iit.it
    @date       02/15/2011
**/

/** @defgroup eo_fakestrg Object EOfakeStorage
    The EOfakeStorage object ce3cecece
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVstorage.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 



/** @typedef    typedef struct EOfakeStorage_hid EOfakeStorage
    @brief      EOfakeStorage is an opaque struct. It is used to implement data abstraction for the  
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOfakeStorage_hid EOfakeStorage;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOfakeStorage* eo_fakestrg_New(uint32_t id, uint32_t capacity, const void *defvalue, EOVmutexDerived *mtx)
    @param      id              the unique id of the storage object
    @param      capacity        the capacity of the storage
    @param      defvalue        the default value of the storage
    @brief      Creates a new fake storage object. 
    @return     The pointer to the required object.
 **/
extern EOfakeStorage* eo_fakestrg_New(uint32_t id, uint32_t capacity, const void *defvalue, EOVmutexDerived *mtx);



/** @}            
    end of group eo_fakestrg  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

