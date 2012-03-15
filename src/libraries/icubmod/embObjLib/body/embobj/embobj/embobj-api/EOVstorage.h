
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVSTORAGE_H_
#define _EOVSTORAGE_H_


/** @file       EOVstorage.h
    @brief      This header file implements public interface to a virtual object used to store data
    @author     marco.accame@iit.it
    @date       09/01/2011
**/

/** @defgroup eov_strg Object EOVstorage
    The EOVstorage is an abstract object used as an interface for a storage object.

    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOVstorage_hid EOVstorage
    @brief      EOVstorage is an opaque struct. It is used to implement data abstraction for the mutex 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOVstorage_hid EOVstorage;


/** @typedef    typedef void EOVstorageDerived
    @brief      EOVstorageDerived is used to implement polymorphism in the objects derived from EoMutex
 **/
typedef void EOVstorageDerived;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 

/** @fn         extern eOpurevirtual eOresult_t eov_strg_Reset(EOVstorageDerived *d)
    @brief      Apply to the storage its default value
    @param      d               Pointer to the derived object
    @return     eores_OK in case of success or eores_NOK_nullpointer if object is NULL.
    @warning    This function cannot be used with a EOVstorage object but only with one object derived
                from it.
 **/
extern eOpurevirtual eOresult_t eov_strg_Reset(EOVstorageDerived *d);


/** @fn         extern eOpurevirtual eOresult_t eov_strg_Set(EOVstorageDerived *d, uint32_t offset, uint32_t size, const void *data)
    @brief      Releases the mutex. 
    @param      d               Pointer to the derived object
    @param      offset          offset from the beginning of the storage
    @param      size            size of the data to write
    @param      data            pointer of data to write
    @return     eores_OK in case of success. osal_res_NOK_generic upon failure of writing data, or 
                or eores_NOK_nullpointer if mutex is NULL.
    @warning    This function cannot be used with a EOVstorage object but only with one object derived
                from it.
 **/
extern eOpurevirtual eOresult_t eov_strg_Set(EOVstorageDerived *d, uint32_t offset, uint32_t size, const void *data);


/** @fn         extern eOpurevirtual eOresult_t eov_strg_Get(EOVstorageDerived *d, uint32_t offset, uint32_t size, void *data)
    @brief      Releases the mutex. 
    @param      d               Pointer to the derived object
    @param      offset          offset from the beginning of the storage
    @param      size            size of the data to read
    @param      data            pointer of data with result
    @return     eores_OK in case of success. osal_res_NOK_generic upon failure of writing data, or 
                or eores_NOK_nullpointer if mutex is NULL.
    @warning    This function cannot be used with a EOVstorage object but only with one object derived
                from it.
 **/
extern eOpurevirtual eOresult_t eov_strg_Get(EOVstorageDerived *d, uint32_t offset, uint32_t size, void *data);




/** @}            
    end of group eov_strg  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



