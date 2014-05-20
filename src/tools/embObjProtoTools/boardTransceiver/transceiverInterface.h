/* Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
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
	virtual void init( uint32_t localipaddr, uint32_t remoteipaddr, uint16_t ipport, uint16_t pktsize, uint8_t borad_n) =0;

	// Do something with the packet received from the socket
	virtual void onMsgReception(uint8_t *data, uint16_t size) =0;


	// get the whole packet to be transmitted
	virtual	void getTransmit(uint8_t **data, uint16_t *size) =0;
} ;

#endif  // include-guard _TRANSCEIVER_INTERFACE_H_


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

