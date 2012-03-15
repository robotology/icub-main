
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHETIMERMANAGER_H_
#define _EOVTHETIMERMANAGER_H_


/** @file       EOVtheTimerManager.h
    @brief      This header file implements public interface to the timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eov_thetimermanager Singleton EOVtheTimerManager
    The EOVtheTimerManager is an abstract object used to derive a timer manager object for the single-task or multi-task
    execution environment.
    The EOVtheTimerManager exposes only pure virtual functions which have to be defined inside the derived object.
 
    The normal user shall typically use only the methods of the derived object, i.e. only eom_timerman_Initialise().

    In the implementaion of EOtimer, we use the virtual methods of EOVtheTimerManager (eov_timerman_GetHandle(), 
    eov_timerman_AddTimer() etc.), in order to make EOtimer able to work without code change in a single-task or in a
    multi-task execution environment.
     
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtimer.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOVtheTimerManager_hid EOVtheTimerManager
    @brief      EOVtheTimerManager is an opaque struct. It is used to implement data abstraction for the timer manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOVtheTimerManager_hid EOVtheTimerManager;


/** @typedef    typedef void EOVtheTimerManagerDerived
    @brief      EOVtheTimerManagerDerived is used to implement polymorphism in the objects derived from EOVtheTimerManager
 **/
typedef void EOVtheTimerManagerDerived;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOVtheTimerManager* eov_timerman_GetHandle(void)
    @brief      Returns an handle to the singleton EOVtheTimerManager.  The singleton must have been initialised by
                the _Initialise() method of a derived type, otherwise this function call will return NULL.
    @return     The pointer to the required EOVtheTimerManager (or NULL upon in-initialised singleton).
 **/
extern EOVtheTimerManager* eov_timerman_GetHandle(void);


/** @fn         extern eOpurevirtual eOresult_t eov_timerman_OnNewTimer(EOVtheTimerManager* p, EOtimer *t)
    @brief      It completes operations on creation of a new timer. The function is to be called only by a EOtimer method.
    @param      p               The pointer to the timer manager singleton. 
    @param      t               The pointer to the timer.
    @return     cecece
 **/
extern eOpurevirtual eOresult_t eov_timerman_OnNewTimer(EOVtheTimerManager* p, EOtimer *t);


/** @fn         extern eOpurevirtual eOresult_t eov_timerman_AddTimer(EOVtheTimerManager* p, EOtimer *t)
    @brief      Adds a timer to the timer manager. The function is to be called only by a EOtimer method.
    @param      p               The pointer to the timer manager singleton. 
    @param      t               The pointer to the timer. 
    @return     cecece
 **/
extern eOpurevirtual eOresult_t eov_timerman_AddTimer(EOVtheTimerManager* p, EOtimer *t);


/** @fn         extern eOpurevirtual eOresult_t eov_timerman_RemTimer(EOVtheTimerManager* p, EOtimer *t)
    @brief      Removes a timer from the timer manager. The function is to be called only by a EOtimer method.
    @param      p               The pointer to the timer manager singleton. 
    @param      t               The pointer to the timer. 
    @return     cecece
 **/
extern eOpurevirtual eOresult_t eov_timerman_RemTimer(EOVtheTimerManager *p, EOtimer *t);

/** @fn         extern eOresult_t eov_timerman_Take(EOVtheTimerManager *p, eOreltime_t tout)
    @brief      Take exclusive control of timer manager. The function is to be called only by a EOtimer method.
    @param      p               The pointer to the timer manager singleton. 
    @param      tout            The timeout. 
    @return     eores_OK upon success to take the mutex, eores_NOK_timeout upon expiry of timeout, but also
                eores_NOK_nullpointer if the timermanager deos not have a mutex.
 **/
extern eOresult_t eov_timerman_Take(EOVtheTimerManager *p, eOreltime_t tout);

/** @fn         extern eOresult_t eov_timerman_Release(EOVtheTimerManager *p)
    @brief      Release control upon the timer manager. The function is to be called only by a EOtimer method.
    @param      p               The pointer to the timer manager singleton. 
    @param      tout            The timeout. 
    @return     eores_OK upon success to take the mutex, eores_NOK_generic upon failure, but also
                eores_NOK_nullpointer if the timermanager deos not have a mutex.
     
 **/
extern eOresult_t eov_timerman_Release(EOVtheTimerManager *p);




/** @}            
    end of group eov_thetimermanager  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



