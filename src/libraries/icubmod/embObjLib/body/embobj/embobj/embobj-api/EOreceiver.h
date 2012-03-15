
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EORECEIVER_H_
#define _EORECEIVER_H_


/** @file       EOreceiver.h
    @brief      This header file implements public interface to a frame.
    @author     marco.accame@iit.it
    @date       01/11/2010
**/

/** @defgroup eo_receiver Object EOreceiver
    The EOreceiver object is used as ...
         
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOropframe.h"
#include "EOpacket.h"
#include "EOnvsCfg.h"




// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 


/** @typedef    typedef struct EOreceiver_hid EOreceiver
    @brief      EOreceiver is an opaque struct. It is used to implement data abstraction for the datagram 
                object so that the user cannot see its private fields and he/she is forced to manipulate the
                object only with the proper public functions. 
 **/  
typedef struct EOreceiver_hid EOreceiver;



typedef struct
{
    uint16_t        capacityofropframereply; // or of packetreply in case we want to use a apcket whcih also has ipaddr and port  
    uint16_t        capacityofropinput;
    uint16_t        capacityofropreply;
    EOnvsCfg*       nvscfg;
} eo_receiver_cfg_t;


    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------

extern const eo_receiver_cfg_t eo_receiver_cfg_default; //= {256, 128, 128, NULL};


// - declaration of extern public functions ---------------------------------------------------------------------------
 
 
/** @fn         extern EOreceiver* eo_receiver_New(const eo_receiver_cfg_t *cfg)
    @brief      Creates a new receiver.
    @param      cfg   the configuration. If NULL, the default is used.
    @return     The pointer to the required object.
 **/
extern EOreceiver* eo_receiver_New(const eo_receiver_cfg_t *cfg);


/** @fn         extern eOresult_t eo_receiver_Process(EOreceiver *p, EOpacket *packet, EOnvsCfg *nvscfg, eObool_t *thereisareply)
    @brief      Accepts the reference to a received packet from a given remote host, uses a given NVs configuration, and process
                the ropframe contained inside the packet (if valid). For each ROP it searches the NV(endpoint, id) if local operation
                or the NV(remoteip, endpoint, id) if remote operation and if found it processes it.
                If there are any reply ROPs it sets the return boolean.   
    @param      p               the object.
    @param      packet          teh received packet
    @param      nvscfg          if not NULL it is the NVs configuration to use, else it is used teh one passed to teh eo_receiver_New() method.
    @param      thereisareply   if not NULL its contains information about teh presence of a reply frame whoch shall be retrieved
                                with the eo_receiver_GetReply() method.
    @return     eores_OK only if the packet is valid and contains a valid ropframe, even if empty. eores_NOK_nullpointer or
                eores_NOK_generic in case of errors.
 **/
extern eOresult_t eo_receiver_Process(EOreceiver *p, EOpacket *packet, EOnvsCfg *nvscfg, uint16_t *numberofrops, eObool_t *thereisareply, eOabstime_t *transmittedtime);


/** @fn         extern eOresult_t eo_receiver_GetReply(EOreceiver *p, EOropframe **ropframereply, eOipv4addr_t *ipv4addr, eOipv4port_t *ipv4port)
    @brief      returns the frame to be transmitted back and the destination ip address and port.
    @param      p               the object.
    @param      ropframereply   handle of the reply frame. if no rop insde, then it is just teh default empty frame.
    @param      ipv4addr        the destination IP address.
    @param      ipv4port        the destination IP port.
    @return     eores_OK only if there is a non-empty ropframe to be transmitted, eores_NOK_generic if teh ropframe is available
                but it is empty, eores_NOK_nullpointer for NULL pointer errors.
 **/
extern eOresult_t eo_receiver_GetReply(EOreceiver *p, EOropframe **ropframereply, eOipv4addr_t *ipv4addr, eOipv4port_t *ipv4port);


/** @}            
    end of group eo_receiver  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

