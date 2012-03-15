
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHETIMERMANAGER_HID_H_
#define _EOVTHETIMERMANAGER_HID_H_


/* @file       EOVtheTimerManager_hid.h
    @brief      This header file implements hidden interface to the base timer manager singleton.
    @author     marco.accame@iit.it
    @date       10/06/2009
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOlist.h"
#include "EOVmutex.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOVtheTimerManager.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
#define VF00_onnewtimer             0
#define VF01_addtimer               1
#define VF02_remtimer               2
#define VTABLESIZE_timerman         3



// - definition of the hidden struct implementing the object ----------------------------------------------------------

//typedef     EOtimer *   (*eOtmrp_fp_tmrmanp_t)              (EOVtheTimerManager*);
typedef     eOresult_t  (*eOres_fp_tmrmanp_tmrp_t)          (EOVtheTimerManager*, EOtimer *);

/** @struct     EOVtheTimerManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOVtheTimerManager_hid 
{
    // - vtable: must be on top of the struct
    void * vtable[VTABLESIZE_timerman];

    // - other stuff
    EOVmutexDerived             *mutex;             /**< mutex which guarantees exclusive access to the manager     */
    // EOlist                      *activetimers;      // to be used only in EOStheTimerManager_hid   
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOVtheTimerManager * eov_timerman_hid_Initialise(eOres_fp_tmrmanp_tmrp_t onnewtimer_fn
                                                        eOres_fp_tmrmanp_tmrp_t addtimer_fn, 
                                                        eOres_fp_tmrmanp_tmrp_t remtimer_fn,
                                                        EOVmutexDerived *mutex)
    @brief      Initialise the singleton EOVtheTimerManager. The function is hidden because this singleton can be used only
                by a derived object.

    @return     The handle to the singleton, or never return if any argument is NULL.
 
 **/

extern EOVtheTimerManager * eov_timerman_hid_Initialise(eOres_fp_tmrmanp_tmrp_t  onnewtimer_fn,
                                                        eOres_fp_tmrmanp_tmrp_t addtimer_fn, 
                                                        eOres_fp_tmrmanp_tmrp_t remtimer_fn,
                                                        EOVmutexDerived *mutex);



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

