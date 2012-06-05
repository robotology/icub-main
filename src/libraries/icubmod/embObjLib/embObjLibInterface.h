
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EMBOBJLIB_INTERFACE_H_
#define _EMBOBJLIB_INTERFACE_H_


/** @file       embObjLibInterface.h
    @brief      This header file implements public interface to ...
    @author     alberto.cardellino@iit.it
    @date       04/20/2011
 **/

typedef struct
{
		char 		name[64];
		uint8_t		ip1;
		uint8_t		ip2;
		uint8_t		ip3;
		uint8_t		ip4;
}EMS_ID;

class IEmbObjResList
{
public:

	virtual	~IEmbObjResList() {};

	// initialize the transceiver
	virtual uint8_t * find(EMS_ID &id ) =0;

	// Do something with the packet received from the socket
	//virtual void onMsgReception();
} ;

#endif  // include-guard _EMBOBJLIB_INTERFACE_H_


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

