
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOFAKESTORAGE_HID_H_
#define _EOFAKESTORAGE_HID_H_


/* @file       EOfakeStorage_hid.h
    @brief      This header file implements hidden interface to a datagram socket object.
    @author     marco.accame@iit.it
    @date       12/24/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

#include "EOVstorage.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOfakeStorage.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section

// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOfakeStorage_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOfakeStorage_hid 
{
    EOVstorage              *storage;            /**< the base storage */
    void                    *ram;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




