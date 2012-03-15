
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSTHETIMERMANAGER_H_
#define _EOSTHETIMERMANAGER_H_


/** @file       EOStheTimerManager.h
    @brief      This header file implements public interface to timer manager singleton for see
    @author     marco.accame@iit.it
    @date       08/04/2011
**/

/** @defgroup eos_thetimermanager Object EOStheTimerManager
    The EOStheTimerManager is derived from EOVtheTimerManager and manages EOtimer objects in the singletasking execution
    environment (SEE). The EOStheTimerManager directly sends the events, the messages, the callback requests to the
    specified task ... the only one is the EOStheFOOP.
 
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/**	@typedef    typedef struct eOmtimerman_cfg_t 
 	@brief      Contains the configuration for the EOStheTimerManager. 
 **/ 
typedef struct
{
    uint8_t                         timernum;   /**< the number of managed timers */
} eOstimerman_cfg_t;
 

/** @typedef    typedef struct EOStheTimerManager_hid EOStheTimerManager
    @brief      EOStheTimerManager is an opaque struct. It is used to implement data abstraction for the single-task 
                execution nevironment so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOStheTimerManager_hid EOStheTimerManager;

   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eOstimerman_cfg_t eos_timerman_DefaultCfg; // = {8};


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOStheTimerManager * eos_timerman_Initialise(const eOmtimerman_cfg_t *tmrmancfg)
    @brief      Initialises the singleton EOStheTimerManager.
                When called the first time, the function creates all data structure required to guarantee the
                correct execution of EOtimer in singletasking execution environment SEE.  
    @param      tmrmancfg       The configuration. 
    @return     The handle to the timer manager.
    @warning    The number of managed EOtimer cannote be zero or the EOtheErrorManager shall issue a fatal error. 
 **/
extern EOStheTimerManager * eos_timerman_Initialise(const eOstimerman_cfg_t *tmrmancfg); 


/** @fn         extern EOStheTimerManager* eos_timerman_GetHandle(void)
    @brief      Returns an handle to the singleton EOStheTimerManager. The singleton must have been initialised
                with eos_timerman_Initialise(), otherwise this function call will return NULL.
    @return     The handle to the timer manager (or NULL upon in-initialised singleton)
 **/
extern EOStheTimerManager* eos_timerman_GetHandle(void);


/** @fn         extern void eos_timerman_Tick(EOStheTimerManager *p)
    @brief      Increment the internal time of the singleton EOStheTimerManager and processes the object EOaction associated
                to the firing EOtimer. 
    @param      p           The handle to the timer manager.
    @warning    It should be called with regularity, possibly inside a callback of the EOStheSystem triggered by an eOvent 
                sent by the system tick. 
                The object EOStheTimerManager internally keeps tracks of the time by maintaining a counter and asking 
                for positive time differences to the singleton EOStheSystem.  In this way possible delays in the call of
                eos_timerman_Tick() do not generate cumulative delays or missed expiries. 
 **/
extern void eos_timerman_Tick(EOStheTimerManager *p);


/** @}            
    end of group eos_thetimermanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

