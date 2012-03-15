
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EORECEIVER_HID_H_
#define _EORECEIVER_HID_H_


/* @file       EOreceiver_hid.h
    @brief      This header file implements hidden interface to a packet object.
    @author     marco.accame@iit.it
    @date       0111/2010
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOpacket.h"
#include "EOropframe.h"
#include "EOrop.h"
#include "EOnvsCfg.h"
#include "EOtheAgent.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOreceiver.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOreceiver_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOreceiver_hid 
{
    //EOpacket*                   rxpacket;
    EOropframe*                 ropframeinput;
    EOropframe*                 ropframereply;    
    EOrop*                      ropinput;
    EOrop*                      ropreply;
    EOnvsCfg*                   nvscfg;
    EOtheAgent*                 theagent;
    eOipv4addr_t                ipv4addr;
    eOipv4port_t                ipv4port;
    uint8_t*                    bufferropframereply;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




