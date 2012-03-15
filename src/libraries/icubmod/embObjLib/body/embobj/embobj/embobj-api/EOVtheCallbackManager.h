
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHECALLBACKMANAGER_H_
#define _EOVTHECALLBACKMANAGER_H_


/** @file       EOVtheCallbackManager.h
    @brief      This header file implements public interface to the virtual callback manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eov_thecallbackmanager Singleton EOVtheCallbackManager
    The EOVtheCallbackManager is an abstract object used to derive a callback manager object for the single-task or multi-task
    execution environment: EOStheCallbackManager or EOMtheCallbackManager.
    The EOVtheCallbackManager exposes only pure virtual functions which have to be defined inside the derived object.
 
    The normal user shall typically use only the methods of the derived object, i.e. only eom_callbackman_Initialise().

    However, if an object cannot be tied to a particular execution environment, then it can use some methods of the 
    EOVtheCallbackManager, provided that a properly derived object has been initialised for the correct execution enviromnent.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtask.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOVtheCallbackManager_hid EOVtheCallbackManager
    @brief      EOVtheCallbackManager is an opaque struct. It is used to implement data abstraction for the timer manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOVtheCallbackManager_hid EOVtheCallbackManager;


/** @typedef    typedef void EOVtheCallbackManagerDerived
    @brief      EOVtheCallbackManagerDerived is used to implement polymorphism in the objects derived from EOVtheCallbackManager
 **/
typedef void EOVtheCallbackManagerDerived;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOVtheCallbackManager* eov_callbackman_GetHandle(void)
    @brief      Returns an handle to the singleton EOVtheCallbackManager.  The singleton must have been initialised by
                the _Initialise() method of a derived type, otherwise this function call will return NULL.
    @return     The pointer to the required EOVtheCallbackManager (or NULL upon in-initialised singleton).
 **/
extern EOVtheCallbackManager* eov_callbackman_GetHandle(void);


/** @fn         extern eOresult_t eov_callbackman_Execute(EOVtheCallbackManager *p, eOcallback_t cbk, eOreltime_t tout)
    @brief      Allows a caller to specify a callback to be directly executed by the object derived from EOVtheCallbackManager
    @param      p               Pointer to the object
    @param      cbk             The callback to be executed
    @param      cbk             The argument of the callback to be executed
    @param      tout            The timeout for posting the message in the internal queue.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the  
                manager was busy.
 **/
extern eOpurevirtual eOresult_t eov_callbackman_Execute(EOVtheCallbackManager *p, eOcallback_t cbk, void *arg, eOreltime_t tout);


/** @fn         extern EOVtaskDerived * eov_callbackman_GetTask(EOVtheCallbackManager *p)
    @brief      Retrieves the working task of the object derived from EOVtheCallbackManager for use in the object EOaction
    @param      p               Pointer to the object
    @return     The pointer to the EOVtaskDerived.
 **/
extern eOpurevirtual EOVtaskDerived * eov_callbackman_GetTask(EOVtheCallbackManager *p);




/** @}            
    end of group eov_thecallbackmanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



