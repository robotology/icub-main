
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTRANSCEIVER_HID_H_
#define _EOTRANSCEIVER_HID_H_


/* @file       EOtransceiver_hid.h
    @brief      This header file implements hidden interface to a packet object.
    @author     marco.accame@iit.it
    @date       0111/2010
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOreceiver.h"
#include "EOtransmitter.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtransceiver.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOtransceiver_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOtransceiver_hid 
{
    eo_transceiver_cfg_t        cfg;
    EOreceiver*                 receiver;
    EOtransmitter*              transmitter;    
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------




#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




