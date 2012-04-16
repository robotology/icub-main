
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _TRANSCEIVER_INTERFACE_H_
#define _TRANSCEIVER_INTERFACE_H_


/** @file       hostTransceiver.h
    @brief      This header file implements public interface to ...
    @author     marco.accame@iit.it
    @date       04/20/2011
 **/

class ITransceiver
{
public:
	virtual ~ITransceiver(){}

	// initialize the transceiver
	virtual void init( uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize) =0;

	// Do something with the packet received from the socket
	virtual void onMsgReception(uint8_t *data, uint16_t size) =0;


	// get the whole packet to be transmitted
	virtual	void getTransmit(uint8_t **data, uint16_t *size) =0;
} ;

#endif  // include-guard _TRANSCEIVER_INTERFACE_H_


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

