
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

#ifndef _ETHRESOURCE_H_
#define _ETHRESOURCE_H_


#include <abstractEthResource.h>
#include <hostTransceiver.hpp>

#include <ethMonitorPresence.h>
#include <EoBoards.h>
#include <yarp/os/Semaphore.h>

#include <ethManager.h>


//#include "can_string_eth.h" // its inclusion in here produces multiple definition of CanFrame
class can_string_eth;

namespace eth {

    class EthResource: public eth::AbstractEthResource
    {

    public:

        EthResource();
        ~EthResource();


        bool            open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal);
        bool            close();
        bool            isID32supported(eOprotID32_t id32);

        eOipv4addr_t getIPv4(void);
        bool getIPv4addressing(eOipv4addressing_t &addressing);

        const string & getName(void);
        const string & getIPv4string(void);
        eObrd_ethtype_t getBoardType(void);
        const string & getBoardTypeString(void);

        // the function returns true if the packet can be transmitted.
        // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
        bool getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops);

        bool canProcessRXpacket(uint64_t *data, uint16_t size);

        void processRXpacket(uint64_t *data, uint16_t size);

        bool getRemoteValue(const eOprotID32_t id32, void *value, const double timeout = 0.100, const unsigned int retries = 0);

        bool setRemoteValue(const eOprotID32_t id32, void *value);

        bool setcheckRemoteValue(const eOprotID32_t id32, void *value, const unsigned int retries = 10, const double waitbeforecheck = 0.001, const double timeout = 0.050);

        //bool getLocalValue(eOprotID32_t id32,  void *value);  // local means ... what we have received
        //bool setLocalValue(eOprotID32_t id32,  void *value);  //
        bool readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size);


        bool verifyEPprotocol(eOprot_endpoint_t ep);

        bool CANPrintHandler(eOmn_info_basic_t* infobasic);


        bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500);

        bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500);

        bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500);

        bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500);

        bool Tick();
        bool Check();



        bool addSetMessage(eOprotID32_t id32, uint8_t* data);
        bool addGetMessage(eOprotID32_t id32);
        bool addGetMessage(eOprotID32_t id32, std::uint32_t signature);


        bool addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data);
        bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size);

        EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv);


        bool isFake();


    private:

//        ACE_INET_Addr     remote_dev;
        eOipv4addressing_t ipv4addressing;
        eOipv4addr_t        ipv4addr;
        string              ipv4addrstring;
        string              boardName;
        string              boardTypeString;
        eObrd_ethtype_t     ethboardtype;

        bool			  isInRunningMode;        //!< say if goToRun cmd has been sent to EMS

        yarp::os::Semaphore* objLock;



        bool                verifiedEPprotocol[eoprot_endpoints_numberof];
        bool                verifiedBoardPresence;
        bool                askedBoardVersion;
        eOdate_t            boardDate;
        eOversion_t         boardVersion;
        eoprot_version_t    boardMNprotocolversion;
        eObrd_ethtype_t     detectedBoardType;

        bool                verifiedBoardTransceiver; // transceiver capabilities (size of rop, ropframe, etc.) + MN protocol version
        bool                txrateISset;
        bool                cleanedBoardBehaviour;    // the board is in config mode and does not have any regulars
        uint16_t            usedNumberOfRegularROPs;

        can_string_eth*     c_string_handler[16];

        eth::TheEthManager *ethManager;

        eth::EthMonitorPresence monitorpresence;

        HostTransceiver transceiver;

        bool regularsAreSet;

    private:


        bool verifyBoard();
        bool setTimingOfRunningCycle();
        bool verifyBoardPresence();
        bool verifyBoardTransceiver();
        bool cleanBoardBehaviour(void);
        bool askBoardVersion(void);
        // we keep isRunning() and we add a field in the reply of serviceStart()/Stop() which tells if the board is in run mode or not.
        bool isRunning(void);

        // lock of the object: on / off
        bool lock(bool on);

        bool serviceCommand(eOmn_serv_operation_t operation, eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout, int times);

        bool verbosewhenok;
    };


} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


