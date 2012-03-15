
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTASK_H_
#define _EOVTASK_H_


/** @file       EOVtask.h
    @brief      This header file implements public interface to a virtual task object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eov_task Object EOVtask
    The EOVtask is a basic object used for deriving execution entities. 
    
    The EOVtask is an abstract object used to derive a task object for the singletask or multitask execution 
    environment.
    The EOVtask exposes only pure virtual functions which have to be defined inside the derived object.
    The user shall create a derived object (e.g., EOMtask *tt) and then he/she can use it with the member functions 
    of the derived objects (e.g., eom_task_xxx(tt, ...)) or even with the member functions of the abstract object
    EOVtask (e.g., eov_task_xxx(tt, ...)).
    
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOVtask_hid EOVtask
    @brief      EOVtask is an opaque struct. It is used to implement data abstraction for the task 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOVtask_hid EOVtask;


/** @typedef    typedef void EOVtaskDerived
    @brief      EOVtaskDerived is used to implement polymorphism in the objects derived from EOVtask
 **/
typedef void EOVtaskDerived;

   
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 

/** @fn         extern eOpurevirtual eOresult_t eov_task_isrSetEvent(EOVtaskDerived *t, eOevent_t evt)
    @brief      Sets an event for the task. The function must be called from within an ISR.  
    @param      t               Pointer to the task-derived object.
    @param      evt             The event mask.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the derived 
                object did not have a proper function to call.
    @warning    This function cannot be used with a EOVtask object but only with one object derived
                from it.                
 **/ 
extern eOpurevirtual eOresult_t eov_task_isrSetEvent(EOVtaskDerived *t, eOevent_t evt);


/** @fn         extern eOpurevirtual eOresult_t eov_task_tskSetEvent(EOVtaskDerived *t, eOevent_t evt)
    @brief      Sets an event for the task. The function must be called from a task and cannot be 
                called from within an ISR.  
    @param      t               Pointer to the task-derived object.
    @param      evt             The event mask.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the derived 
                object did not have a proper function to call.
    @warning    This function cannot be used with a EOVtask object but only with one object derived
                from it.                     
 **/ 
extern eOpurevirtual eOresult_t eov_task_tskSetEvent(EOVtaskDerived *t, eOevent_t evt);


/** @fn         extern eOpurevirtual eOresult_t eov_task_isrSendMessage(EOVtaskDerived *t, eOmessage_t msg)
    @brief      Sends a message to the task. The function must be called from within an ISR.  
    @param      t               Pointer to the task-derived object.
    @param      msg             The message.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the derived 
                object did not have a proper function to call.
 **/ 
extern eOpurevirtual eOresult_t eov_task_isrSendMessage(EOVtaskDerived *t, eOmessage_t msg);


/** @fn         extern eOpurevirtual eOresult_t eov_task_tskSendMessage(EOVtaskDerived *t, eOmessage_t msg, eOreltime_t tout)
    @brief      Sends a message to the task. The function must be called from a task and cannot be called from 
                within an ISR.  
    @param      t               Pointer to the task-derived object.
    @param      msg             The message.
    @param      tout            The desired timeout
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the derived 
                object did not have a proper function to call.
 **/ 
extern eOpurevirtual eOresult_t eov_task_tskSendMessage(EOVtaskDerived *t, eOmessage_t msg, eOreltime_t tout);


/** @fn         extern eOpurevirtual eOresult_t eov_task_isrExecCallback(EOVtaskDerived *t, eOcallback_t cbk)
    @brief      Sends a callback request to the task. The function must be called from within an ISR.  
    @param      t               Pointer to the task-derived object.
    @param      cbk             The callback.
    @param      arg             The argument of the callback.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the derived 
                object did not have a proper function to call.
 **/ 
extern eOpurevirtual eOresult_t eov_task_isrExecCallback(EOVtaskDerived *t, eOcallback_t cbk, void *arg);


/** @fn         extern eOpurevirtual eOresult_t eov_task_tskExecCallback(EOVtaskDerived *t, eOcallback_t cbk, eOreltime_t tout)
    @brief      Sends a callback request to the task. The function must be called from a task and cannot be called from 
                within an ISR.  
    @param      t               Pointer to the task-derived object.
    @param      cbk             The callback.
    @param      arg             The argument of the callback.
    @param      tout            The desired timeout
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the derived 
                object did not have a proper function to call.
 **/ 
extern eOpurevirtual eOresult_t eov_task_tskExecCallback(EOVtaskDerived *t, eOcallback_t cbk, void *arg, eOreltime_t tout); 


/** @fn         extern eOpurevirtual eOid08_t eov_task_GetID(EOVtaskDerived *t)
    @brief      Gets the unique ID with which the system has identified the task.  
    @param      t               Pointer to the task-derived object.
    @return     The ID, or zero upon failure
 **/ 
extern eOpurevirtual eOid08_t eov_task_GetID(EOVtaskDerived *t);



/** @}            
    end of group eov_task  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

