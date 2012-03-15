
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSYNCHROSOCKET_HID_H_
#define _EOSYNCHROSOCKET_HID_H_


/* @file       EOsynchroSocket_hid.h
    @brief      This header file implements hidden interface to a datagram socket object.
    @author     marco.accame@iit.it
    @date       12/24/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtimer.h"
#include "EOsocket.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOsynchroSocket.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section

// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOsynchroSocket_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOsynchroSocket_hid 
{
    EOsocket                *socket;
    EOtimer                 *timertx;
    EOpacket                *txpkt;
    EOpacket                *rxpkt;
    eOreltime_t             period;
    eOabstime_t             starttime;
    uint8_t                 txenable;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




