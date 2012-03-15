
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSMUTEX_H_
#define _EOSMUTEX_H_


/** @file       EOSmutex.h
    @brief      This header file implements public interface to a multitasking mutex.
    @author     marco.accame@iit.it
    @date       08/04/2011
**/

/** @defgroup eos_mutex Object EOSmutex
    The EOSmutex is an object for the singletask execution environment (SEE) derived from the abstract object EOVmutex.
    It allows mutual exclusion from ISRs with priority inversion. It requires external functions which takes and releases
    a critical section

    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOSmutex_hid EOSmutex
    @brief      EOSmutex is an opaque struct. It is used to implement data abstraction for the multi-task 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOSmutex_hid EOSmutex;


   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOSmutex * eos_mutex_New(void)
    @brief      Creates a new EOSmutex object by derivation from an abstract object EOVmutex. This mutex is to be used
                in the singletask environment. The underlying mechanism is based on the hal. 
    @param      hal_sys_criticalsection_take    The function which takes a critical section in the system. It can be
                                                implemented with a global disable interrupts or with a more sophisticated 
                                                mechanism.
    @param      hal_sys_criticalsection_release The function which releases a critical section in the system.
    @return     The pointer to the required EOSmutex. Never NULL.
    @warning    If any of the two functions are NULL then it is called the error manager.
 **/
extern EOSmutex * eos_mutex_New(eOres_fp_voidp_uint32_t hal_sys_criticalsection_take, eOres_fp_voidp_t hal_sys_criticalsection_release);


/** @fn         extern eOresult_t eos_mutex_Take(EOSmutex *const m, eOreltime_t tout)
    @brief      It takes a mutex with a given timeout. If the mutex is already taken,
                the function waits until it is released.
    @param      m               The mutex
    @param      tout            .....     
    @return     eores_OK in case of success. eores_NOK_timeout upon failure to take the mutex, or 
                or eores_NOK_nullpointer if mutex is NULL.
 **/
extern eOresult_t eos_mutex_Take(EOSmutex *const m, eOreltime_t tout);


/** @fn         extern eOresult_t eos_mutex_Release(EOSmutex *const m)
    @brief      It releases a mutex previously taken. 
    @param      m               The mutex
    @return     eores_OK in case of success. osal_res_NOK_generic upon failure to release the mutex, or 
                or eores_NOK_nullpointer if mutex is NULL.
 **/
extern eOresult_t eos_mutex_Release(EOSmutex *const m); 





/** @}            
    end of group eos_mutex  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




