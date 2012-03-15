
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCONSTVECTOR_HID_H_
#define _EOCONSTVECTOR_HID_H_


/* @file       EOconstvector_hid.h
    @brief      This header file implements hidden interface to a EOconstvector object.
    @author     marco.accame@iit.it
    @date       08/03/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOconstvector.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOconstvector_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOconstvector_hid 
{
    eOsizecntnr_t               size;                                                            
    eOsizeitem_t                item_size;                
    const void*                 item_array_data;        
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



