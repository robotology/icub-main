
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHESYSTEM_H_
#define _EOVTHESYSTEM_H_


/** @file       EOVtheSystem.h
    @brief      This header file implements public interface to the virtual system singleton.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/

/** @defgroup eov_thesystem Singleton EOVtheSystem
    The EOVtheSystem manages the functions of the virtual execution environment (VEE). The VEE does not live by itself
    but by means of a multitasking infrastructure (an RTOS) or a singletask scheduler (superloop or similar). 
    The EOVtheSystem is an abstract singleton whcih can be used only if another derived singleton has defined the behaviour
    of its pure virtual functions. Its services are thus offered through the parent singleton. 

    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVtask.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 
 

/** @typedef    typedef struct EOVtheSystem_hid EOVtheSystem
    @brief      EOVtheSystem is an opaque struct. It is used to implement data abstraction for the timer manager 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOVtheSystem_hid EOVtheSystem;


/** @typedef    typedef void EOVtheSystemDerived
    @brief      EOVtheSystemDerived is used to implement polymorphism in the objects derived from EOVtheSystem
 **/
typedef void EOVtheSystemDerived;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOVtheSystem* eov_sys_GetHandle(void)
    @brief      Returns an handle to the singleton EOVtheSystem.  The singleton must have been initialised by
                the Initialise() method of a derived type, otherwise this function call will return NULL.
    @return     The pointer to the required EOVtheSystem (or NULL upon in-initialised singleton).
 **/
extern EOVtheSystem* eov_sys_GetHandle(void);


/** @fn         extern eOpurevirtual EOVtaskDerived* eov_sys_GetRunningTask(EOVtheSystem* p)
    @brief      Returns the pointer of the currently running task (derived from EOVtask)
    @param      p               The pointer to the system singleton. 
    @return     The task pointer.
 **/
extern eOpurevirtual EOVtaskDerived* eov_sys_GetRunningTask(EOVtheSystem* p);


/** @fn         extern eOpurevirtual eOabstime_t eov_sys_LifeTimeGet(EOVtheSystem* p)
    @brief      Returns the time of life of the system in microseconds
    @param      p               The pointer to the system singleton. 
    @return     The time of life of the system, 0 id @e p is NULL.
 **/
extern eOpurevirtual eOabstime_t eov_sys_LifeTimeGet(EOVtheSystem* p);


/** @fn         extern eOpurevirtual eOresult_t eov_sys_LifeTimeSet(EOVtheSystem* p, eOabstime_t ltime
    @brief      Sets the time of life of the system in microseconds
    @param      p               The pointer to the system singleton. 
    @param      ltime           The time to use.
    @return     eores_OK on success of the operation, eores_NOK_nullpointer if @e p is NULL.
 **/
extern eOpurevirtual eOresult_t eov_sys_LifeTimeSet(EOVtheSystem* p, eOabstime_t ltime);


/** @fn         extern eOpurevirtual eOresult_t eov_sys_NanoTimeGet(EOVtheSystem* p, eOnanotime_t *nt)
    @brief      Extracts the time of life of the system in nanoseconds
    @param      p               The pointer to the system singleton.
    @param      nt              The nanotime 
    @return     The eores_NOK_nullpointer result if @e p is NULL, eores_OK in case of success.
 **/
extern eOpurevirtual eOresult_t eov_sys_NanoTimeGet(EOVtheSystem* p, eOnanotime_t *nt);


/** @fn         extern eOpurevirtual eOresult_t eov_sys_Stop(EOVtheSystem *p)
    @brief      Stops the system. This method is used only in extreme cases, for instance by the error manager
                in case of fatal error. The behavior of the function is defined by teh derived singleton, but it should
                at least stop the scheduling of tasks.
    @param      p               The pointer to the system singleton.
    @return     The eores_NOK_nullpointer result if @e p is NULL, eores_OK in case of success.
 **/
extern eOpurevirtual eOresult_t eov_sys_Stop(EOVtheSystem *p);



/** @}            
    end of group eov_thesystem  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



