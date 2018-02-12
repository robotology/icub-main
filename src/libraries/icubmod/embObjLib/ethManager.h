// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Alberto Cardellino, Marco Accame
 * email:   alberto.cardellino@iit.it, marco.accame@iit.it
 * website: www.robotcub.org
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

#ifndef _THEETHMANAGER_H_
#define _THEETHMANAGER_H_

// Standard includes
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <stdio.h>
//#include <map>


// ACE includes
#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/Thread.h>
#include <ace/SOCK_Dgram_Bcast.h>
#include "ace/OS_main.h"
#include "ace/OS_NS_string.h"
#include "ace/OS_NS_sys_socket.h"

// YARP includes
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

// embobjlib includes
#include "hostTransceiver.hpp"
#include "FeatureInterface.h"
#include "IethResource.h"
#include "abstractEthResource.h"

// embobj includes
#include "EoProtocol.h"
#include "EoBoards.h"

// others
#include <yarp/os/LogStream.h>              // iCub debug class include


using namespace yarp::os;
using namespace std;




#include <ethBoards.h>
#include <ethSender.h>
#include <ethReceiver.h>


// -- class TheEthManager
// -- it is the main singleton which delas with eth communication.
// -- it holds the two tx and rx threads (classes EthSender and EthReceiver)
// -- it holds class EthBoards which stores references to EthResource (what is used to form / parse UDP packets for the eth board)
// -- and to the interfaces which use the EthResource of a given board.

namespace eth {

    class TheEthManager
    {

    public:
        // this is the maximum number of boards that the singleton can manage.
        enum { maxBoards = eth::EthBoards::maxEthBoards };

        enum { maxRXpacketsize = 1496, maxTXpacketsize = 1496 };

        // these are the boards, their use is protected by txSem or rxSem or both of them.
        eth::EthBoards* ethBoards;

    private:

        // singletons have private constructor / destructor
        TheEthManager();
        ~TheEthManager();

    public:

        // singletons use this method to retrieve an handle to the object
        static TheEthManager* instance();

        // we use this method to for a destruction of the object.
        static bool killYourself();


        // useful for printing times relative to start of the object
        double getLifeTime(void);

        bool verifyEthBoardInfo(yarp::os::Searchable &cfgtotal, eOipv4addr_t &boardipv4, string boardipv4string, string boardname);


        eth::AbstractEthResource* requestResource2(IethResource *interface, yarp::os::Searchable &cfgtotal);

        int releaseResource2(eth::AbstractEthResource* ethresource, IethResource* interface);

        const eOipv4addressing_t& getLocalIPV4addressing(void);

        bool Transmission(void);

        bool CheckPresence(void);

        bool Reception(eOipv4addr_t from, uint64_t* data, ssize_t size);

        eth::AbstractEthResource* getEthResource(eOipv4addr_t ipv4);

        IethResource* getInterface(eOipv4addr_t ipv4, eOprotID32_t id32);

        int getNumberOfResources(void);

        const string & getName(eOipv4addr_t ipv4);

        int sendPacket(const void *udpframe, size_t len, const eOipv4addressing_t &toaddressing);

        eOipv4addr_t toipv4addr(const ACE_INET_Addr &aceinetaddr);

        ACE_INET_Addr toaceinet(const eOipv4addressing_t &ipv4addressing);

    private:


        bool isCommunicationInitted(void);

        bool createCommunicationObjects(const eOipv4addressing_t &localaddress, int txrate, int rxrate);

        bool initCommunication(yarp::os::Searchable &cfgtotal);

        bool stopCommunicationThreads(void);

        bool lock(bool on);

        bool lockTX(bool on);
        bool lockRX(bool on);
        bool lockTXRX(bool on);


    private:

        void initEOYsystem(void);

        // this semaphore is used to ....
        static yarp::os::Semaphore managerSem;
        // the following two semaphore are used separately or together to stop tx and rx if a change is done on ethboards (in startup and shutdown phases)
        static yarp::os::Semaphore txSem;
        static yarp::os::Semaphore rxSem;

        static eth::TheEthManager* handle;

        // contains the start-up time of the system so that time measures / prints can be done relative.
        double startUpTime;

        eOipv4addressing_t ipv4local;

        bool communicationIsInitted;

        // periodic threads which use methods of class TheEthManager to transmit / receive + the udp socket
        eth::EthSender* sender;
        eth::EthReceiver* receiver;
        ACE_SOCK_Dgram* UDP_socket;
        bool embBoardsConnected;

    };

} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------






