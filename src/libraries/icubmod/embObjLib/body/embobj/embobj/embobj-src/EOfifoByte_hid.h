
#ifndef _EOFIFOBYTE_HID_H_
#define _EOFIFOBYTE_HID_H_

// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOfifo.h"

// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/*  @struct    EOfifoByte_hid
     @brief     Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/   
 
struct EOfifoByte_hid 
{
    // base object
    EOfifo     *fifo;
};

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



