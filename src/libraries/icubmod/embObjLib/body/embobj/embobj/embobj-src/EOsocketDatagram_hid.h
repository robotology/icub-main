
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOSOCKETDATAGRAM_HID_H_
#define _EOSOCKETDATAGRAM_HID_H_


/* @file       EOsocketDatagram_hid.h
    @brief      This header file implements hidden interface to a datagram socket object.
    @author     marco.accame@iit.it
    @date       08/25/2011
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOfifo.h"
#include "EOsocket.h"
#include "EOtimer.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOsocketDatagram.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section

// - definition of the hidden struct implementing the object ----------------------------------------------------------

/* @struct     EOsocketDatagram_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOsocketDatagram_hid 
{
    EOsocket                *socket;            /**< the base socket */
    EOfifo                  *dgramfifoinput;    /**< fifo of input datagrams */
    EOfifo                  *dgramfifooutput;   /**< fifo of output datagrams */
    eOreltime_t             toutfifos;          /**< timeout for fifo manipulation */
    EOtimer                 *txtimer;
    eOsktdtgTXmode_t        txmode;  
    EOaction                *actiontx;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




