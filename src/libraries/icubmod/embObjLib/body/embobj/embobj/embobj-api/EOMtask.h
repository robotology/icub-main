
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMTASK_H_
#define _EOMTASK_H_


/** @file       EOMtask.h
    @brief      This header file implements public interface to a task for MEE
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eom_task Object EOMtask
    The EOMtask object implements a task in the multi-tasking execution environment (MEE). The task is executed in 
    the MEE and scheduled for execution in a fully preemptive way and according to its priority and on availability 
    of the waited resources . There are tasks of types such as: event-driven, message-drive, callback-drive, periodic, 
    and with 
    user-defined behavaiour. The task first executes only once a startup function and then executes a run function.
    For user-defined behaviour, the run function is executed only once, thus it is responsibility of the user to place
    in its inside any forever loop which avoids the task termination and any control upon needed resources.  In this
    mode it is possible for instance to implement background tasks with a low priority and maybe working in round-robin.
    In periodic mode, the run function is scheduled for execution in a periodic manner.
    In event-driven and message-driven and callback-driven modes the run function is is scheduled for execution only when
    an event or a message or a callback are available for the task or a waiting timeout has expired. The execution at the 
    expiry of the timeout allows the execution of the run function in an asynchronous way but with a maximum delta time (e.g.,
    for a watchdog timer refresh).
    The EOMtask is derived from the abstract object EOVtask, and as such can be manipulated also by the base abstract
    object EOVtask by polymorphism.
    
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section


// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOMtask_hid EOMtask
    @brief      EOMtask is an opaque struct. It is used to implement data abstraction for the multi-task 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOMtask_hid EOMtask;


/** @typedef    typedef enum eOmtaskType_t
    @brief      eOmtaskType_t contains the types of a EOMtask
 **/
typedef enum
{
    eom_mtask_UserDefined = 0,
    eom_mtask_EventDriven,
    eom_mtask_MessageDriven,
    eom_mtask_CallbackDriven,
    eom_mtask_Periodic
} eOmtaskType_t;

/** @typedef    typedef enum eOmtaskPriorities_t                         
    @brief      eOmtaskPriorities_t contains the limits of priorities that a EOMtask can have
 **/ 
typedef enum
{
    eom_mtask_prio_min                      = 2,
    eom_mtask_prio_max                      = 251       
} eOmtaskPriorities_t;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------



/** @fn         extern EOMtask * eom_task_New(eOmtaskType_t type, uint8_t priority, uint16_t stacksize,
                                       void (*startup_fn)(EOMtask *tsk, uint32_t zero),
                                       void (*run_fn)(EOMtask *tsk, uint32_t evtmsgper), 
                                       uint8_t queuesize, eOreltime_t timeoutorperiod,
                                       void (*nameofthetask_fn)(void *tsk))
    @brief      Creates a new multi-tasking task object. 
    @param      type            The type of task.
    @param      priority        The priority of task, whose limits are eom_mtask_prio_min and eom_mtask_prio_max. If outside, the value
                                is clipped.
    @param      stacksize       The stack used for the execution of the task.
    @param      startup_fn      The startup function.  It is executed only once after the creation of the task. The first argument is
                                the EOMtask itself, the second is always zero.
    @param      run_fn          The run function.  Its execution depends on the type of task. 
                                If @e eom_mtask_UserDefined, it is executed a single time, thus the user must put a forever loop inside. 
                                If @e eom_mtask_Periodic it is executed periodically every @e timeoutorperiod  micro-seconds. 
                                If @e eom_mtask_MessageDriven it is executed every time the task receives a message on its internal queue
                                or at the expiry of a timeout specified by @e timeoutorperiod.
                                If @e eom_mtask_EventDriven it is executed every time the task receives an event or at the expiry of 
                                a timeout specified by @e timeoutorperiod.
                                If @e eom_mtask_CallbackDriven it is executed after the execution of the callback or at the expiry of 
                                a timeout specified by @e timeoutorperiod.
                                The arguments passed to the function are tsk which is the current EOMtask object and evtmsgper which
                                contains either the received event or message or execution period or zero in case of timeout or user-defined
                                task.
    @param      queuesize       It specifies the size of the internal receiving queue. The queue is created only when the @e type is 
                                @e eom_mtask_MessageDriven or @eom_mtask_CallbackDriven. In such a case, if the value is wrongly set 
                                to zero, the queue will be created with size one.
    @param      timeoutorperiod It specifies the timeout or the period depending on the type of task. 
	@param		extdata         It contains a pointer to external data which can be used by the startup_fn() and run_fn() functions.
    @param      nameofthetask_fn In case it is not NULL, then uVision 4 by Keil shall show the name of the function in its graphical
                                task monitor. This function shall be be defined to contain eom_task_Start((EOMtask*)tsk) as only instruction, 
                                where tsk is the argument of the function.
    @param      name            The name used when the task is started

    @return     The wanted object. Never NULL.
 **/ 
extern EOMtask * eom_task_New(eOmtaskType_t type, uint8_t priority, uint16_t stacksize,
                                       void (*startup_fn)(EOMtask *tsk, uint32_t zero),
                                       void (*run_fn)(EOMtask *tsk, uint32_t evtmsgper), 
                                       uint8_t queuesize, eOreltime_t timeoutorperiod,
									   void *extdata,
                                       void (*nameofthetask_fn)(void *tsk),
                                       const char *name);


/** @fn         extern void eom_task_Start(EOMtask *p) 
    @brief      To be called inside nameofthetask_fn() as only instruction.
    @param      p               Pointer to the EOMtask.
 **/
extern void eom_task_Start(EOMtask *p);


/** @fn         extern void* eom_task_GetExternalData(EOMtask *p) 
    @brief      Retrieves external data contained inside the task.
    @param      p               Pointer to the EOMtask.
 **/
extern void* eom_task_GetExternalData(EOMtask *p);


/** @fn         extern eOresult_t eom_task_SetEvent(EOMtask *p, eOevent_t evt)
    @brief      Set an event for the given task. It can be called from within another EOMtask object.
    @param      p               Pointer to the EOMtask object.  
    @param      evt             The mask wich contains the events to be set. 
    @return     eores_NOK_nullpointer if task is NULL, eores_NOK_generic if it does not manage events, or eores_OK.
 **/
extern eOresult_t eom_task_SetEvent(EOMtask *p, eOevent_t evt);


/** @fn         extern eOresult_t eom_task_isrSetEvent(EOMtask *p, eOevent_t evt)
    @brief      Lets an ISR send a message to the given task.
    @param      p               Pointer to the EOMtask object.  
    @param      evt             The mask wich contains the events to be set. 
    @return     eores_NOK_nullpointer if task is NULL, eores_NOK_generic if it does not manage events, or eores_OK.
 **/
extern eOresult_t eom_task_isrSetEvent(EOMtask *p, eOevent_t evt);


/** @fn         extern eOresult_t eom_task_SendMessage(EOMtask *p, eOmessage_t msg, eOreltime_t tout)
    @brief      Lets another EOMtask object send a message to the given task. It cannot  be called from within an 
                IRS, for which there is a proper funtion. The message will be stored into the message queue of tteh task.
                If the task does not have a message queue, then the message is lost.
    @param      p               Pointer to the EOMtask object. 
    @param      msg             The message to be sent. 
    @param      tout            The maximum time of wait in case the queue is full and cannot accept the message. 
    @return     eores_OK, eores_NOK_timeout, eres_NOK_nullpointer eores_NOK_generic if it does not manage messages
 **/
extern eOresult_t eom_task_SendMessage(EOMtask *p, eOmessage_t msg, eOreltime_t tout);


/** @fn         extern eOresult_t eom_task_isrSendMessage(EOMtask *p, eOmessage_t msg)
    @brief      Lets an ISR send a message to the given task. The message will be stored into a queue if the queue
                is not full, otherwise the message will be lost.
    @param      p               Pointer to the EOMtask object. 
    @param      msg             The message to be sent. 
    @return     eores_OK, eores_NOK_nowait, eores_NOK_nullpointer or eores_NOK_generic if it does not manage messages
 **/
extern eOresult_t eom_task_isrSendMessage(EOMtask *p, eOmessage_t msg);


/** @fn         extern eOresult_t eom_task_ExecCallback(EOMtask *p, eOcallback_t cbk, eOreltime_t tout)
    @brief      Lets another EOMtask object request an activity the given task. It cannot  be called from within an 
                IRS, for which there is a proper funtion. The activity request will be stored into a proper queue internal
                to the task. If the task is not activit-based, then the request is lost.
    @param      p               Pointer to the EOMtask object. 
    @param      cbk             The callback execution request. 
    @param      arg             The argument of the callback.
    @param      tout            The maximum time of wait in case the queue is full and cannot accept the request. 
    @return     eores_OK, eores_NOK_timeout, eres_NOK_nullpointer or eores_NOK_generic if it does not manage activities
 **/
extern eOresult_t eom_task_ExecCallback(EOMtask *p, eOcallback_t cbk, void *arg, eOreltime_t tout);


/** @fn         extern eOresult_t eom_task_isrSendMessage(EOMtask *p, eOmessage_t msg)
    @brief      Lets an ISR request teh execution of an activity to the given task. The request will be stored 
                into a proper queue internal to the task if the queue is not full, otherwise the message will be lost.
    @param      p               Pointer to the EOMtask object. 
    @param      cbk             The callback execution request. 
    @param      arg             The argument of the callback.
    @return     eores_OK, eores_NOK_nowait, eores_NOK_nullpointer or eores_NOK_generic if it does not manage activities
 **/
extern eOresult_t eom_task_isrExecCallback(EOMtask *p, eOcallback_t cbk, void *arg);


/** @fn         extern eOresult_t eom_task_PriorityGet(EOMtask *p, uint8_t *prio)
    @brief      Retrieves the priority of a given task
    @param      p               Pointer to the EOMtask object.  
    @param      prio            The pointer to teh retrieve priority value. 
    @return     eores_NOK_nullpointer if task is NULL, or eores_OK.
 **/
extern eOresult_t eom_task_PriorityGet(EOMtask *p, uint8_t *prio);


/** @fn         extern eOresult_t eom_task_PrioritySet(EOMtask *p, uint8_t prio)
    @brief      Sets the priority of a given task
    @param      p               Pointer to the EOMtask object.  
    @param      prio            The priority value. 
    @return     eores_NOK_nullpointer if task is NULL, or eores_OK.
 **/
extern eOresult_t eom_task_PrioritySet(EOMtask *p, uint8_t prio);


/** @}            
    end of group eom_task  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


