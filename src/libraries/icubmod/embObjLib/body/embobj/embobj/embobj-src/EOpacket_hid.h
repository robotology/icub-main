
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOPACKET_HID_H_
#define _EOPACKET_HID_H_


/* @file       EOpacket_hid.h
    @brief      This header file implements hidden interface to a packet object.
    @author     marco.accame@iit.it
    @date       0111/2010
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOpacket.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section


// - definition of the hidden struct implementing the object ----------------------------------------------------------

/** @struct     EOpacket_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOpacket_hid 
{
    eOipv4addr_t        remoteaddr;
    eOipv4port_t        remoteport;
    uint8_t             incomplete_flag;
    uint8_t             externaldatastorage; 
    uint16_t            size;
    uint16_t            capacity;
    uint16_t            write_index;
    uint16_t            read_index;
    uint8_t             *data;
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------

extern eOresult_t eo_packet_hid_DefInit(void *p, uint32_t a);

extern eOresult_t eo_packet_hid_DefCopy(void *d, void *s);

extern eOresult_t eo_packet_hid_DefClear(void *p);


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




