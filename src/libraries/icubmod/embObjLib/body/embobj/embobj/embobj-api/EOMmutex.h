
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMMUTEX_H_
#define _EOMMUTEX_H_


/** @file       EOMmutex.h
    @brief      This header file implements public interface to a multitasking mutex.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eom_mutex Object EOMmutex
    The EOMmutex is an object for the multitask execution environment derived from the abstract object EOVmutex.
    It allows mutual exclusion in the MEE with priority inversion. The underlying mechanism
    is based on the osal_mutex_t.  

    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOMmutex_hid EOMmutex
    @brief      EOMmutex is an opaque struct. It is used to implement data abstraction for the multi-task 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMmutex_hid EOMmutex;


   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOMmutex * eom_mutex_New(void)
    @brief      Creates a new EOMmutex object by derivation from an abstract object EOVmutex. This mutex is to be used
                in the multitask environment. The underlying mechanism is based on the osal_mutex_t and has priority 
                inversion. 
    @return     The pointer to the required EOMmutex. Never NULL.
 **/
extern EOMmutex * eom_mutex_New(void);


/** @fn         extern eOresult_t eom_mutex_Take(EOMmutex *const m, eOreltime_t tout)
    @brief      It takes a multitask mutex with a given timeout. If the mutex is already taken by another task,
                the function waits its release accoring to the value of @e tout. The mutex has priority inversion. 
                See also osal_mutex_take().
    @param      m               The mutex
    @param      tout            The required timeout in micro-seconds. If eok_reltimeZERO the mutex does not wait
                                and if another task has already taken it, it returns immediately with a failure.
                                If eok_reltimeINFINITE the mutex waits indefinitely until the mutex is released.     
    @return     eores_OK in case of success. eores_NOK_timeout upon failure to take the mutex, or 
                or eores_NOK_nullpointer if mutex is NULL.
 **/
extern eOresult_t eom_mutex_Take(EOMmutex *const m, eOreltime_t tout);


/** @fn         extern eOresult_t eom_mutex_Release(EOMmutex *const m)
    @brief      It releases a multitask mutex previously taken by the same task. See also osal_mutex_release().
    @param      m               The mutex
    @return     eores_OK in case of success. osal_res_NOK_generic upon failure to release the mutex, or 
                or eores_NOK_nullpointer if mutex is NULL.
 **/
extern eOresult_t eom_mutex_Release(EOMmutex *const m); 





/** @}            
    end of group eom_mutex  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




