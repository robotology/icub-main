
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHESYSTEM_H_
#define _EOMTHESYSTEM_H_


/** @file       EOMtheSystem.h
    @brief      This header file implements public interface to the system singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eom_thesystem Singleton EOMtheSystem
    The EOMtheSystem is derived from abstract object EOVtheSystem to give to embOBJ a multitask execution environment
    (MEE) based on a fully preemptive RTOS, the OSAL.  The EOMtheSystem must be first initialised and then started. 
    Its start initialise some objects of embOBJ such as the EOtheMemoryPool, the EOtheErrorManager, the EOMtheTimerManager
    and the EOMtheCallbackManager. It is responsibility of EOMtheSystem to also initialise HAL, OSAL, and FSAL. 
 
    When the EOMtheSystem is later started, it will launch the OSAL and run a user-defined function which will launch
    other services of the embOBJ, and also launch proper user tasks using the EOMtask object.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheMemoryPool.h"
#include "EOtheErrorManager.h"
#include "EOMtheTimerManager.h"
#include "EOMtheCallbackManager.h"
#include "hal.h"
#include "fsal.h"
#include "osal.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct eOmsystem_cfg_t
    @brief      eOmsystem_cfg_t contains the specification for the system singleton in the multi-task execution
                environment (MEE).
 **/  
typedef struct
{
    uint32_t                    codespaceoffset;
    const hal_cfg_t*            halcfg;
    const osal_cfg_t*           osalcfg;
    const fsal_params_cfg_t*    fsalcfg;
} eOmsystem_cfg_t;
 

/** @typedef    typedef struct EOMtheSystem_hid EOMtheSystem
    @brief      EOMtheSystem is an opaque struct. It is used to implement data abstraction for the timer manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMtheSystem_hid EOMtheSystem;


/** @typedef    typedef void EOMtheSystemDerived
    @brief      EOMtheSystemDerived is used to implement polymorphism in the objects derived from EOMtheSystem
 **/
typedef void EOMtheSystemDerived;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------


/** @fn         extern EOMtheSystem * eom_sys_Initialise(const eOmsystem_cfg_t *syscfg, 
                                         const eOmempool_cfg_t *mpoolcfg, 
                                         const eOerrman_cfg_t *errmancfg,
                                         const eOmtimerman_cfg_t *tmrmancfg,
                                         const eOmcallbackman_cfg_t *cbkmancfg)
    @brief      Initialise the singleton EOMtheSystem and all the other entities that are required by the system. In order they are:
                the error manager, the the memory pool, the HAL, the OSAL, and optionally the FSAL. 
                The EOMtheTimerManager and EOMtheCallbackManager are initialised later by eom_sys_Start().
    @param      syscfg          The configuration of the system (HAL, OSAL, FSAL).  Only the config of FSAL can be NULL. In such a case
                                the FSAL is not initialised.
    @param      mpoolcfg        The configuration of the EOtheMemoryPool.  If NULL, it is used the default configuration
                                (@e eom_mempool_DefaultCfg), which uses the heap.  The function eo_mempool_Initialise() is called internally.
    @param      errmancfg       The configuration of the EOtheErrorManager  If NULL, it is used the default configuration @e eom_errman_DefaultCfg.
                                The function eo_errman_Initialise() is called internally.
    @param      tmrmancfg       The configuration of the EOMtheTimerManager  If NULL, it is used the default configuration @e eom_timerman_DefaultCfg
                                The function eom_timerman_Initialise() is called internally by the OSAL init task.
    @param      cbkmancfg       The configuration of the EOMtheCallbackManager  If NULL, it is used the default configuration @e eom_callbackman_DefaultCfg
                                The function eom_callbackman_Initialise() is called internally by the OSAL init task.
    @return     A not NULL handle to the singleton.  In case of errors it is called the EOtheErrorManager
    @wa
 
 **/
extern EOMtheSystem * eom_sys_Initialise(const eOmsystem_cfg_t *syscfg, 
                                         const eOmempool_cfg_t *mpoolcfg, 
                                         const eOerrman_cfg_t *errmancfg,
                                         const eOmtimerman_cfg_t *tmrmancfg,
                                         const eOmcallbackman_cfg_t *cbkmancfg);
 
 
/** @fn         extern EOMtheSystem* eom_sys_GetHandle(void)
    @brief      Returns an handle to the singleton EOMtheSystem. The singleton must have been initialised otherwise 
                this function call will return NULL.
    @return     The pointer to the required EOMtheSystem (or NULL upon in-initialised singleton).
 **/
extern EOMtheSystem* eom_sys_GetHandle(void);


/** @fn         extern void eom_sys_Start(EOMtheSystem *p, eOvoid_fp_void_t userinit_fn)
    @brief      It starts EOMtheSystem.  The singleton must have been initialised otherwise 
                this function call will issue a fatal error to the EOtheErrorManager.
                The function shall start the OSAL with its init task at maximum priority. The init task 
                initialises the EOMtheTimerManager and EOMtheCallbackManager, and calls the function @e userinit_fn(),
                where the user can start other services (e.g., the GPIO manager or the IP net) and also other
                service tasks. When the function @e userinit_fn() is finished, the OSAL removes the init task and
                allows other tasks to run according to their priorities.
    @param      p               The handler to the singleton.
    @param      userinit_fn     The starting function executed by the OSAL.  It is responsibility of the
                                programmer to place inside userinit_fn() the proper calls to create MEE objects
                                such as EOtask, etc.
    @return     The function does not return and execution stays in there forever.
 **/
extern void eom_sys_Start(EOMtheSystem *p, eOvoid_fp_void_t userinit_fn);




/** @}            
    end of group eom_thesystem  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



