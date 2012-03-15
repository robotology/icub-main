
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCONSTARRAY_HID_H_
#define _EOCONSTARRAY_HID_H_


/* @file        EOconstarray_hid.h
    @brief      This header file implements hidden interface to a array object.
    @author     marco.accame@iit.it
    @date       08/03/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOconstarray.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOconstarray_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOconstarray_hid 
{
    eOsizeitem_t    sizeofitem;
    eOsizecntnr_t   size;
    const uint8_t*  data;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




