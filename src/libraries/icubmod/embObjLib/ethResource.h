// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup TheEthManager TheEthManager
 *
*/
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

//
// $Id: TheEthManager.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __ethResource__
#define __ethResource__

// System includes
#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

// Yarp includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

// embObj includes

#include "hostTransceiver.hpp"
#include "debugFunctions.h"
#include "FeatureInterface.h"


// ACE stuff
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/SOCK_Dgram_Bcast.h>

// iCub Debug class
#include "Debug.h"

// Risky place fot those defines!!
#warning acemor-> removed EMPTY_PACKET_SIZE from here
//#define EMPTY_PACKET_SIZE     EOK_HOSTTRANSCEIVER_emptyropframe_dimension
#warning acemor-> RECV_BUFFER_SIZE must be higher or equal EOK_HOSTTRANSCEIVER_capacityofrxpacket. it can safely be 1500
#define	RECV_BUFFER_SIZE      EOK_HOSTTRANSCEIVER_capacityofrxpacket
#define	SIZE_INFO             126
#define MAX_ICUB_EP           32

namespace yarp{
    namespace dev{
        class ethResources;
        class TheEthManager;
    }
}

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev;
using namespace std;


class yarp::dev::ethResources:  public DeviceDriver,
                                public hostTransceiver
{
private:
//     static yarp::os::Semaphore  ethResMutex;

    char              info[SIZE_INFO];
    int               how_many_features;    //!< Keep track of how many high level class registered. onto this EMS
    ACE_INET_Addr     remote_dev;           //!< IP address of the EMS this class is talking to.
//     ACE_INET_Addr     remote_broadcast;
//     bool                isRunning;

public:
//     hostTransceiver   *transceiver;         //!< Pointer to the protocol handler   non più usato. ora c'è derivazione diretta.
    yarp::dev::TheEthManager     *ethManager;          //!< Pointer to the Singleton handling the UDP socket

    ethResources();
    ~ethResources();

    uint8_t         recv_msg[RECV_BUFFER_SIZE];   // buffer for the reveived messages
    ACE_UINT16      recv_size;                    // size of the incoming message

    ethResources *  already_exists(yarp::os::Searchable &config);
    bool            open(FEAT_ID request);
    bool            close();

    /*!   @fn       registerFeature(void);
     *    @grief    tells the EMS a new device requests its services.
     *    @return   true.
     */
    bool            registerFeature(FEAT_ID *request);

    /*!   @fn       unregisterFeature();
     *    @brief    tell the EMS a user has been closed. If was the last one, close the EMS
     */
    int             deregisterFeature(FEAT_ID request);

    ACE_INET_Addr   getRemoteAddress(void);

    /*!   @fn       void getPointer2txPack(uint8_t **pack, uint16_t *size);
     *    @brief    Create the new packet to be sent with all requests collected so far, and return the pointer to
     *              this fresh new packet.
     *    @param    pack  this pointer will be modified to make it point to the new packet. No memory needs to be associated
     *                    with it, just a pointer.
     *    @param    size  retutn the dimension of the packet.
     */
    void            getPointer2TxPack(uint8_t **pack, uint16_t *size);

    /*!   @fn       void getPack(uint8_t *pack, uint16_t *size);
     *    @brief    Create the new packet to be sent with all requests collected so far, and copy it in data. External memory
     *              needs to be provided.
     *    @param    size  retutn the dimension of the packet.
     */
    void            getTxPack(uint8_t *pack, uint16_t *size);

    //! Return the dimension of the receiving buffer.
    int             getBufferSize();

    void            onMsgReception(uint8_t *data, uint16_t size);


    /*!   @fn       goToRun(void);
     *    @brief    Tells the EMS to start the 1ms loop.
     */
    bool            goToRun(void);


    /*!   @fn       goToConfig(void);
     *    @brief    Tells the EMS to stop the 1ms loop and go into configuration mode.
     */
    bool            goToConfig(void);

//     //! Send a sporadic message. Do not use now.
//     int             send(void *data, size_t len);
};


#endif
