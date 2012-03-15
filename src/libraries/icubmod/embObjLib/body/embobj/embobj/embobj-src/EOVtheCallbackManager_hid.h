
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOVTHECALLBACKMANAGER_HID_H_
#define _EOVTHECALLBACKMANAGER_HID_H_


/* @file       EOVtheCallbackManager_hid.h
    @brief      This header file implements hidden interface to the base timer manager singleton.
    @author     marco.accame@iit.it
    @date       08/30/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOlist.h"
#include "EOVmutex.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOVtheCallbackManager.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
#define VF00_execute                0
#define VTABLESIZE_vcallbackman     1



// - definition of the hidden struct implementing the object ----------------------------------------------------------


typedef     eOresult_t  (*eOres_fp_vcbkmanp_cbk_voidp_uint32_t)          (EOVtheCallbackManager*, eOcallback_t, void*, uint32_t);

/** @struct     EOVtheCallbackManager_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOVtheCallbackManager_hid 
{
    // - vtable: must be on top of the struct
    void * vtable[VTABLESIZE_vcallbackman];

    // - other stuff
    EOVtaskDerived             *task;             // link to the task whcih executes the callback as defined by the derived object
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------


/** @fn         extern EOVtheCallbackManager * eov_callbackman_hid_Initialise(eOres_fp_vcbkmanp_cbk_voidp_uint32_t execute_fn, EOVtaskDerived *task)
    @brief      Initialise the singleton EOVtheCallbackManager. The function is hidden because this singleton can be used only
                by a derived object.

    @return     The handle to the singleton, or never return if any argument is NULL.
 
 **/

extern EOVtheCallbackManager * eov_callbackman_hid_Initialise(eOres_fp_vcbkmanp_cbk_voidp_uint32_t execute_fn, EOVtaskDerived *task);



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

