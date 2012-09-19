/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _HOSTTRANSCEIVER_H_
#define _HOSTTRANSCEIVER_H_


/** @file       hostTransceiver.h
    @brief      This header file implements public interface to ...
    @author     marco.accame@iit.it
    @date       04/20/2011
 **/

/** @defgroup  Library hostTransceiver
    It is an example of how the embOBJ can operate as host trasceiver.

    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOhostTransceiver.h"
//#include "transceiverInterface.h"

	// Boards configurations

#include "eOcfg_nvsEP_mn.h"
#include "eOcfg_nvsEP_mc.h"
#include "eOcfg_nvsEP_sk.h"

#include "eOcfg_EPs_board.h"


#include <yarp/dev/DeviceDriver.h>

using namespace yarp::dev;

class hostTransceiver : public DeviceDriver
//						public ITransceiver
{
private:
	EOhostTransceiver*  	hosttxrx;
	EOtransceiver*      	pc104txrx;
	EOnvsCfg*           	pc104nvscfg;
	uint32_t            	localipaddr;
	uint32_t            	remoteipaddr;
	uint16_t            	ipport;
	EOpacket*           	pktTx;
	EOpacket*           	pktRx;



public:
	hostTransceiver();
    ~hostTransceiver();
	const EOconstvector* 	EPvector;
	eOuint16_fp_uint16_t 	EPhash_function_ep2index;
	void init( uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize, uint8_t board_n);

	// as an alternative ... create one methd which clears the remote vars, one which pushes one ack, and another the confirms them all.
	void hostTransceiver_ConfigureRegularsOnRemote(void);

	void load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid);
	void s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board(void);

	// somebody adds a set-rop  plus data.
	void hostTransceiver_AddSetROP(uint16_t endpoint, uint16_t id, uint8_t* data, uint16_t size);

	// create a get nv rop
	void s_hostTransceiver_AddGetROP(uint16_t ep, uint16_t id);

	void s_hostTransceiver_AddSetROP_with_data_already_set(uint16_t ep, uint16_t id);

	// somebody passes the received packet
	void SetReceived(uint8_t *data, uint16_t size);
	// and Processes it
	virtual void onMsgReception(uint8_t *data, uint16_t size);

	// somebody retrieves what must be transmitted
	void getTransmit(uint8_t **data, uint16_t *size);

	void getNVvalue(EOnv *nvRoot, uint8_t* data, uint16_t* size);
	EOnv* getNVhandler(uint16_t endpoint, uint16_t id);
	void askNV(uint16_t endpoint, uint16_t id, uint8_t* data, uint16_t* size);

	void getHostData(const EOconstvector **pEPvector, eOuint16_fp_uint16_t *pEPhash_function);

	uint16_t getNVnumber(int boardNum, eOnvEP_t ep);
	uint16_t translate_NVid2index(uint8_t boardNum, eOnvEP_t ep, eOnvID_t nvid);
};

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

