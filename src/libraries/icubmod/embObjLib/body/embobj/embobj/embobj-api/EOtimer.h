
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTIMER_H_
#define _EOTIMER_H_


/** @file       EOtimer.h
    @brief      This header file implements public interface to a timer object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eo_timer Object EOtimer
    The EOtimer object is used to perform an action at the expiry of a countdown. The action
    is specified by the EOaction variable passed to the object at start of the timer and
    can be one of the following: send a message to a task, send an event to a task, ask a teask to
    perform a callback function.  
    The EOtimer can be programmed to execute the action in two ways: to executed only once at 
    the expiry of the countdown, or to execute it forever by retriggering the countdown.
    It is possible to have multiple EOtimer objects at the same time.  They are managed by a
    timer manager singleton derived from EOVtheTimerManager which must be properly initialised
    (and ticked for the single-task variant).
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOaction.h"


// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

 

/** @typedef    typedef struct EOtimer_hid EOtimer
    @brief      EOtimer is an opaque struct. It is used to implement data abstraction for the timer 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOtimer_hid EOtimer;


/** @typedef    typedef enum eOtimerMode_t
    @brief      eOtimerMode_t contains the modes with which a timer can be started
 **/
typedef enum 
{
    eo_tmrmode_ONESHOT = 0,        /**< The countdown is executed only once        */    
    eo_tmrmode_FOREVER             /**< The countdown is automatically reloaded    */
} eOtimerMode_t;


/** @typedef        typedef enum eOtimerStatus_t
    @brief          eOtimerStatus_t contains the states in which a timer can be
 **/
typedef enum  
{
    eo_tmrstat_Idle = 0,           /**< No countdown is active (never started or stopped)              */
    eo_tmrstat_Running,            /**< The countdown is currently running                             */
    eo_tmrstat_Completed           /**< The countdown has expired with eo_tmrmode_ONESHOT mode            */
} eOtimerStatus_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOtimer* eo_timer_New(void)
    @brief      Creates a new timer object 
    @return     The pointer to the required object.
 **/
extern EOtimer* eo_timer_New(void);


/** @fn         extern eOresult_t eo_timer_Start(EOtimer *t, eOabstime_t startat, eOreltime_t countdown, eOtimerMode_t mode, EOaction *action)
    @brief      Starts a timer to execute an action @e action on the expiry of a countdown of @e countdown micro-seconds which begins at the 
                absolute time specified by @e startat. After the first expiry, the timer can simply stop itself or restart the countdown
                depending upon the value of @e mode.
    @param      t               The pointer to the timer object.
    @parm       startat         If the value is equal to eok_abstimeNOW, then the coundown is started using the current
                                lifetime as absolute reference and the timer is NOT synchronised. If the timer is not synchronised,
                                in case of  a change of lifetime, then the countdown will not be changed.
                                If instead the value is different from eok_abstimeNOW (it can be even zero), then the timer starts
                                its countdown exactly at absolute time @e startat and will expire exactly at absolute time @e startat +
                                @e countdown. If mode is eo_tmrmode_FOREVER it will expire at any absolute time equal to @e startat +
                                n * @e countdown, with n >= 1. 
                                In case of a change of lifetime, the internal counter shall be adjusted to make the timer expire exactly
                                at the same absolute time as it was originally programmed.
    @param      countdown       The countdown in micro-seconds. If @e startat is not eok_abstimeNOW it must be > 0.
    @param      mode            The execution mode.
    @param      action          What happens upon expiry of the timer.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the timer could 
                not be assigned to its manager.
    @warning    This function must not be called from inside an ISR because it may interfere with the
                timer manager in an unpredictable way. Also, one-shot calls will happens only if requested for the future.
 **/
extern eOresult_t eo_timer_Start(EOtimer *t, eOabstime_t startat, eOreltime_t countdown, eOtimerMode_t mode, EOaction *action);


/** @fn         extern eOresult_t eo_timer_Stop(EOtimer *t)
    @brief      Stop a running timer. The function can be safely called also if the timer is idle or in 
                completed status 
    @param      t               The pointer to the timer object.
    @return     eores_OK upon success, eores_NOK_nullpointer if p is NULL, eores_NOK_generic if the timer 
                manager was busy.
    @warning    This function must not be called from inside an ISR because it may interfere with the
                timer manager in an unpredictable way.
 **/
extern eOresult_t eo_timer_Stop(EOtimer *t);


/** @fn         extern eOabstime_t eo_timer_Remaining(EOtimer *t)
    @brief      Gets the remaining time before next trigger. 
    @param      t               The pointer to the timer object.
    @return     The remaining time in micro-seconds, or 0 if the timer is idle or in completed status
                or if t is NULL.
    @warning    The functionaly is not yet implemented and this function always returns 0
 **/
extern eOabstime_t eo_timer_Remaining(EOtimer *t);


/** @fn         extern eOtimerStatus_t eo_timer_GetStatus(EOtimer *t)
    @brief      Gets the status of the timer. 
    @param      t               The pointer to the timer object.
    @return     The status of the timer, or eo_tmrstat_Idle if t is NULL.
 **/
extern eOtimerStatus_t eo_timer_GetStatus(EOtimer *t);
 


/** @}            
    end of group eo_timer  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

