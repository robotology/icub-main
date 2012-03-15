
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTRANSMITTER_HID_H_
#define _EOTRANSMITTER_HID_H_


/* @file       EOtransmitter_hid.h
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
#include "EOlist.h"

// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtransmitter.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - definition of the hidden struct implementing the object ----------------------------------------------------------

typedef struct      // 16 bytes ... 
{
    eOropcode_t     ropcode;
    eOnvID_t        nvid;
    uint16_t        capacity;       
    uint16_t        ropstarthere;   // the index where the rop starts inside teh ropframe. if data is available, then it is placed at ropstarthere+8
    uint16_t        ropsize;
    uint16_t        timeoffsetinsiderop;     // if time is not present its value is 0xffff 
    void*           nvloc;          // the pointer to the where the real value of the nv is stored. if null, then the rop does not contain data (an ask for example)
    EOnv*           nv;
} eo_transm_permrops_info_t;


/** @struct     EOtransmitter_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/  
 
struct EOtransmitter_hid 
{
    EOpacket*                   txpacket;
    EOropframe*                 ropframereadytotx;
    EOropframe*                 ropframepermanent;
    EOropframe*                 ropframetemporary;    
    EOropframe*                 ropframereplies;
    EOrop*                      roptmp;
    EOnvsCfg*                   nvscfg;
    EOtheAgent*                 theagent;
    eOipv4addr_t                ipv4addr;
    eOipv4port_t                ipv4port;
    uint8_t*                    bufferropframepermanent;
    uint8_t*                    bufferropframetemporary;
    uint8_t*                    bufferropframereplies;
    EOlist*                     listofpermropsinfo; 
    eOabstime_t                 currenttime;    
}; 


// - declaration of extern hidden functions ---------------------------------------------------------------------------




#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




