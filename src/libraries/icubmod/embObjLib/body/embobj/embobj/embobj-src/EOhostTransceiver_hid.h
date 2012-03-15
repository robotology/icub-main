

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOONEHOSTTRANSCEIVER_HID_H_
#define _EOONEHOSTTRANSCEIVER_HID_H_


/* @file       EOhostTransceiver_hid.h
    @brief      This header file implements hidden interface to the EOhostTransceiver singleton.
    @author     marco.accame@iit.it
    @date       09/03/2010
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtransceiver.h"
#include "EOnvsCfg.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOhostTransceiver.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOhostTransceiver_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOhostTransceiver_hid 
{
    EOtransceiver*          transceiver;
    EOnvsCfg*               nvscfg;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section

#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







