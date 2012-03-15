
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMMUTEX_HID_H_
#define _EOMMUTEX_HID_H_


/* @file       EOMmutex_hid.h
    @brief      This header file implements hidden interface to the rtos mutex object.
    @author     marco.accame@iit.it
    @date       08/03/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOMmutex.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOMmutex_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOMmutex_hid 
{ 
    // - base object
    EOVmutex                *mutex;

    // - other stuff
    osal_mutex_t            *osalmutex;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




