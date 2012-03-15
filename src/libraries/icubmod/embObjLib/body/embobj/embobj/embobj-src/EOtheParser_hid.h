

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHEPARSER_HID_H_
#define _EOTHEPARSER_HID_H_


/* @file       EOtheParser_hid.h
    @brief      This header file implements hidden interface to the EOtheParser singleton.
    @author     marco.accame@iit.it
    @date       09/06/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheParser.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/* @struct     EOtheParser_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOtheParser_hid 
{
    uint8_t initted;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







