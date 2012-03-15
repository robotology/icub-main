
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEGPIOMANAGER_H_
#define _EOTHEGPIOMANAGER_H_


/** @file       EOtheGPIOManager.h
    @brief      This header file implements public interface to the GPIO manager singleton.
    @author     marco.accame@iit.it
    @date       08/24/2011
**/

/** @defgroup eo_thegpiomanager Singleton EOtheGPIOManager
    The EOtheGPIOManager is an object used to manage the GPIOs and can be used as it is or also in composition with
    other objects, for instance to obtain #EOMtheGPIOManager used in the MEE.
 
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOtheGPIOManager_hid EOtheGPIOManager
    @brief      EOtheGPIOManager is an opaque struct. It is used to implement data abstraction for the GPIO manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtheGPIOManager_hid EOtheGPIOManager;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

/** @fn         extern void eo_gpioman_Initialise(EOtheGPIO *gpio, EOVmutexDerived *mutex)
    @brief      Initialises the singleton gpio manager.
    @param      gpio            The GPIO singleton that we want to manage
    @param      mutex           Pointer to the mutex that will be used to gain exclusive access to the resource.
                                In SEE it is a EOSmutex (but can be NULL if eo_gpioman_Tick() is
                                not called by an ISR), otherwise in MEE it is a EOMmutex.
                                By setting a valid mutex, the user of the object EOtheGPIOManager is insured that
                                any extern function declared in the .h or _hid.h is protected vs councurrent access.
                                In this case, it is not required to take or release the singleton.
    @return     The EOtheGPIOManager singleton. 
 **/
extern EOtheGPIOManager * eo_gpioman_Initialise(EOtheGPIO *gpio, EOVmutexDerived *mutex);  
 
 
/** @fn         extern EOtheGPIOManager* eo_gpioman_GetHandle(void)
    @brief      Returns a handle to the singleton EOtheGPIOManager. The singleton must have been initialised
                with eo_gpioman_Initialise), otherwise this function call will return NULL.
    @return     pointer to the required EOtheGPIOManager object (or NULL upon in-initialised singleton)
 **/
extern EOtheGPIOManager* eo_gpioman_GetHandle(void);


/** @fn         extern void eo_gpioman_Tick(EOtheGPIOManager* p, eOreltime_t delta)
    @brief      Runs a single step of the GPIO processing. This function must be called regularly, maybe
                by a EOtimer-triggered callback (in SEE or MEE) or by a periodic task in the MEE.
    @param      p               pointer to the EOtheGPIOManager singleton. 
    @param      delta           Time elapsed from last call.
 **/
extern eOresult_t eo_gpioman_Tick(EOtheGPIOManager* p, eOreltime_t delta); 
 
 

/** @}            
    end of group eo_thegpiomanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



