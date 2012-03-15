
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _HOSTTRANSCEIVER_H_
#define _HOSTTRANSCEIVER_H_


/** @file       hostTransceiver.h
    @brief      This header file implements public interface to ...
    @author     marco.accame@iit.it
    @date       04/20/2011
**/

/** @defgroup eo_cedcew Library hostTransceiver
    It is an example of how the embOBJ can operate as host trasceiver.
     
    @{        
 **/



// - external dependencies --------------------------------------------------------------------------------------------
#ifdef __cplusplus
	extern "C" {
#endif

#include "EoCommon.h"
#include "EOhostTransceiver.h"

#ifdef __cplusplus
}
#endif



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
 

// - declaration of public user-defined types -------------------------------------------------------------------------    
// empty-section

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
 

extern void hostTransceiver_Init(uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize);

// as an alternative ... create one methd which clears the remote vars, one which pushes one ack, and another the confirms them all.
extern void hostTransceiver_ConfigureRegularsOnRemote(void);

void s_eom_hostprotoc_extra_protocoltransceiver_load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid);
void s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board(void);

// somebody adds a set-rop  plus data.
extern void hostTransceiver_AddSetROP(uint16_t endpoint, uint16_t id, uint8_t* data, uint16_t size);

// create a get nv rop
static void s_hostTransceiver_AddGetROP(uint16_t ep, uint16_t id);

// somebody passes the received packet
extern void hostTransceiver_SetReceived(uint8_t *data, uint16_t size);


// somebody retrieves what must be transmitted
extern void hostTransceiver_GetTransmit(uint8_t **data, uint16_t *size);



/** @}            
    end of group eo_cedcew 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

