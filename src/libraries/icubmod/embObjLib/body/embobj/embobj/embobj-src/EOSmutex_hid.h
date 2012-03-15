
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSMUTEX_HID_H_
#define _EOSMUTEX_HID_H_


/* @file       EOSmutex_hid.h
    @brief      This header file keeps hidden interface to a mutex object for see
    @author     marco.accame@iit.it
    @date       08/04/2011
**/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOVmutex.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOSmutex.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOSmutex_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOSmutex_hid 
{ 
    // - base object
    EOVmutex                *mutex;

    // - other stuff
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




