
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTHETIMERMANAGER_H_
#define _EOMTHETIMERMANAGER_H_


/** @file       EOMtheTimerManager.h
    @brief      This header file implements public interface to the MEE timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eom_thetimermanager Object EOMtheTimerManager
    The EOMtheTimerManager is derived from EOVtheTimerManager and manages EOtimer objects in the multitasking execution
    environment. The EOMtheTimerManager uses an hidden task for the dispatch of events, messages, callback requests
    to the EOMtheCallbackManager.  The hidden task is a message-driven EOMtask which receives requests of expired
    timers directly from OSAL.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOMtheTimerManager_hid EOMtheTimerManager
    @brief      EOMtheTimerManager is an opaque struct. It is used to implement data abstraction for the multi-task 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMtheTimerManager_hid EOMtheTimerManager;


/**	@typedef    typedef struct eOmtimerman_cfg_t 
 	@brief      Contains the configuration for the EOMtheTimerManager. 
 **/ 
typedef struct
{
    uint8_t         priority;           /**< The priority of the worker task */
    uint16_t        stacksize;          /**< The stack size of the worker task */
    uint8_t         messagequeuesize;   /**< The size of the message queue in the worker task */
} eOmtimerman_cfg_t;
   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOmtimerman_cfg_t eom_timerman_DefaultCfg; // = {240, 512, 8};


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOMtheTimerManager * eom_timerman_Initialise(const eOmtimerman_cfg_t *tmrmancfg)
    @brief      Initialises the singleton EOMtheTimerManager.
                When called the first time, the function creates all data structure required to guarantee the
                correct execution of EOtimer in multitasking execution environment.  The function is
                also responsible to start a message driven task which is able to perform the actions associated to 
                the expiry of the EOtimer objects: send event / message to a task or ask the execution of a callback.
    @param      tmrmancfg       The configuration.
    @return     The handle to the timer manager.
    @warning    The number of managed EOtimer depends only by the configuration of the OSAL. 
 **/
extern EOMtheTimerManager * eom_timerman_Initialise(const eOmtimerman_cfg_t *tmrmancfg); 


/** @fn         extern EOMtheTimerManager* eom_timerman_GetHandle(void)
    @brief      Returns an handle to the singleton EOMtheTimerManager. The singleton must have been initialised
                with eom_timerman_Initialise(), otherwise this function call will return NULL.
    @return     The handle to the RTOS timer manager (or NULL upon in-initialised singleton)
    @warning    This function is of no utility until we dont have public functions which use the handle. However,
                we keep it for future use.
 **/
extern EOMtheTimerManager* eom_timerman_GetHandle(void);





/** @}            
    end of group eom_thetimermanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

