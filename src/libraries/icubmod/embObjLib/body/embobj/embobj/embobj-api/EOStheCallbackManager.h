
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTHECALLBACKMANAGER_H_
#define _EOSTHECALLBACKMANAGER_H_


/** @file       EOStheCallbackManager.h
    @brief      This header file implements public interface to the ...
    @author     marco.accame@iit.it
    @date       08/04/2011
**/

/** @defgroup eos_thecallbackmanager Object EOStheCallbackManager
    The EOStheCallbackManager allows to run callbacks in the single-tasking execution environment. 

    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOStheFOOP.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/**	@typedef    typedef struct eOscallbackman_cfg_t 
 	@brief      Contains the configuration for the EOStheCallbackManager. 
 **/ 
typedef struct
{
    uint8_t         queuesize;          /**< The size of the receviving queue in the task executing the callbacks*/
} eOscallbackman_cfg_t;


/** @typedef    typedef struct EOStheCallbackManager_hid EOStheCallbackManager
    @brief      EOStheCallbackManager is an opaque struct. It is used to implement data abstraction for the 
                object callback manager for the multitask execution environment
                so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOStheCallbackManager_hid EOStheCallbackManager;


   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOscallbackman_cfg_t eos_callbackman_DefaultCfg; // = {8};


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOStheCallbackManager * eos_callbackman_Initialise(const eOmcallbackman_cfg_t *cbkmancfg)
    @brief      Initialises the singleton EOStheCallbackManager.
                When called the first time, the function creates all data structure required to guarantee the
                correct management of callback execution in the singletask execution environment. 
    @param      cbkmancfg       The configuration. If NULL, it is used the eom_callback_DefaultCfg.
    @return     The handle to the callback manager.
 **/
extern EOStheCallbackManager * eos_callbackman_Initialise(const eOscallbackman_cfg_t *cbkmancfg); 


/** @fn         extern EOStheCallbackManager * eos_callbackman_GetHandle(void)
    @brief      Returns an handle to the singleton EOStheCallbackManager. The singleton must have been initialised
                with eos_callbackman_Initialise(), otherwise this function will return NULL.
    @return     The handle to the callback manager or NULL.
 **/
extern EOStheCallbackManager * eos_callbackman_GetHandle(void);


/** @fn         extern eOresult_t eos_callbackman_Execute(EOStheCallbackManager *p, eOcallback_t cbk)
    @brief      Called to ask the callback manager to execute a callback. 
    @param      p               The singleton
    @param      cbk             The callback
    @param      arg             The argument
    @return     eores_NOK_nullpointer if called with a NULL singleton, eores_OK on success
 **/
extern eOresult_t eos_callbackman_Execute(EOStheCallbackManager *p, eOcallback_t cbk, void *arg);


/** @fn         extern EOStheFOOP * eos_callbackman_GetTask(EOStheCallbackManager *p)
    @brief      Gets the pointer to the task object whcih executes the callbacks: the EOStheFOOP. 
    @param      p               The singleton
    @return     Pointer to the EOStheFOOP
 **/
extern EOStheFOOP * eos_callbackman_GetTask(EOStheCallbackManager *p);
 




/** @}            
    end of group eos_thecallbackmanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



