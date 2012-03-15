
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOMUTEXFIFO_HID_H_
#define _EOMUTEXFIFO_HID_H_

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOdeque.h"
#include "EOVmutex.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOfifo.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/* @struct     EOfifo_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/   
 
struct EOfifo_hid 
{
    // contained objects
    EOdeque                 *dek;
    EOVmutexDerived         *mutex;
    // other stuff
};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



