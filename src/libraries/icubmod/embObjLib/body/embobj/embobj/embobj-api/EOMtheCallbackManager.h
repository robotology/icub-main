
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHECALLBACKMANAGER_H_
#define _EOMTHECALLBACKMANAGER_H_


/** @file       EOMtheCallbackManager.h
    @brief      This header file implements public interface to the MEE callback manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eom_thecallbackmanager Object EOMtheCallbackManager
    The EOMtheCallbackManager allows to run callbacks in the multi-tasking execution environment. The callbacks
    are passed as function pointers to a callback-based task which executes them in order of reception and with the
    priority assigned to EOMtheCallbackManager.
    The request for a callback is done through a EOtimer or a EOioPinInputManaged.

    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOMtask.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOMtheCallbackManager_hid EOMtheCallbackManager
    @brief      EOMtheCallbackManager is an opaque struct. It is used to implement data abstraction for the 
                object callback manager for the multitask execution environment
                so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMtheCallbackManager_hid EOMtheCallbackManager;


/**	@typedef    typedef struct eOmcallbackman_cfg_t 
 	@brief      Contains the configuration for the EOMtheCallbackManager. 
 **/ 
typedef struct
{
    uint8_t         priority;           /**< The priority of the worker task */
    uint16_t        stacksize;          /**< The stack size of the worker task */
    uint8_t         queuesize;          /**< The size of the receviving queue in the task executing the callbacks*/
} eOmcallbackman_cfg_t;


   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOmcallbackman_cfg_t eom_callbackman_DefaultCfg; // = {202, 512, 8};


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOMtheCallbackManager * eom_callbackman_Initialise(const eOmcallbackman_cfg_t *cbkmancfg)
    @brief      Initialises the singleton EOMtheCallbackManager.
                When called the first time, the function creates all data structure required to guarantee the
                correct management of callback execution in the multitasking execution environment. 
    @param      cbkmancfg       The configuration. If NULL, it is used the eom_callback_DefaultCfg.
    @return     The handle to the callback manager.
 **/
extern EOMtheCallbackManager * eom_callbackman_Initialise(const eOmcallbackman_cfg_t *cbkmancfg); 


/** @fn         extern EOMtheCallbackManager * eom_callbackman_GetHandle(void)
    @brief      Returns an handle to the singleton EOMtheCallbackManager. The singleton must have been initialised
                with eom_callbackman_Initialise(), otherwise this function will return NULL.
    @return     The handle to the callback manager or NULL.
 **/
extern EOMtheCallbackManager * eom_callbackman_GetHandle(void);


/** @fn         extern eOresult_t eom_callbackman_Execute(EOMtheCallbackManager *p, eOcallback_t cbk, eOreltime_t tout)
    @brief      Allows a caller to specify a callback to be directly executed by the EOMtheCallbackManager
    @param      p               Pointer to the object
    @param      cbk             The callback to be executed
    @param      cbk             The argument of the callbalk to be executed
    @param      tout            The timeout for posting the message in the internal queue.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the  
                manager was busy.
 **/
extern eOresult_t eom_callbackman_Execute(EOMtheCallbackManager *p, eOcallback_t cbk, void *arg, eOreltime_t tout);


/** @fn         extern EOMtask * eom_callbackman_GetTask(EOMtheCallbackManager *p)
    @brief      Retrieves the working task of the EOMtheCallbackManager
    @param      p               Pointer to the object
    @return     The pointer to the EOMtask.
 **/
extern EOMtask * eom_callbackman_GetTask(EOMtheCallbackManager *p);


/** @}            
    end of group eom_thecallbackmanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



