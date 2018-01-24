// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
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

#ifndef _FAKEETHRESOURCE_H_
#define _FAKEETHRESOURCE_H_



#include<abstractEthResource.h>

#include <yarp/os/Semaphore.h>
#include <ethManager.h>


namespace eth {
           
    class FakeEthResource : public eth::AbstractEthResource
    {

    public:

        FakeEthResource();
        ~FakeEthResource();


        bool open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal) override;
        bool close();

        const Properties & getProperties();

        const void * getUDPtransmit(eOipv4addressing_t &destination, size_t &sizeofpacket, uint16_t &numofrops);


        bool processRXpacket(const void *data, const size_t size);


        bool getRemoteValue(const eOprotID32_t id32, void *value, const double timeout = 0.100, const unsigned int retries = 0);

        bool getRemoteValues(const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout = 0.500);

        bool setRemoteValue(const eOprotID32_t id32, void *value);

        bool setcheckRemoteValue(const eOprotID32_t id32, void *value, const unsigned int retries = 10, const double waitbeforecheck = 0.001, const double timeout = 0.050);

        bool getLocalValue(const eOprotID32_t id32,  void *value);

        bool setLocalValue(const eOprotID32_t id32,  const void *value, bool overrideROprotection = false);

        bool verifyEPprotocol(eOprot_endpoint_t ep);

        bool CANPrintHandler(eOmn_info_basic_t* infobasic);


        bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500);

        bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500);

        bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500);

        bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500);

        bool Tick();
        bool Check();

//        bool addSetMessage(eOprotID32_t id32, uint8_t* data);
//        bool addGetMessage(eOprotID32_t id32);
//        bool addGetMessage(eOprotID32_t id32, std::uint32_t signature);
//        bool addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data);
//        bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size);
//        EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv);


        bool isFake();

        HostTransceiver * getTransceiver();


    private: //FAKE
        eOipv4addressing_t ipv4addressing;
        eOipv4addr_t      ipv4addr;
        string ipv4addrstring;
        string boardName;
        string boardTypeString;
        eObrd_ethtype_t   ethboardtype;
        double            lastRecvMsgTimestamp;   //! stores the system time of the last received message, gettable with getLastRecvMsgTimestamp()
        bool              isInRunningMode;        //!< say if goToRun cmd has been sent to EMS

        yarp::os::Semaphore* objLock;

        bool                verifiedEPprotocol[eoprot_endpoints_numberof];
        bool                verifiedBoardPresence;

        bool                verifiedBoardTransceiver; // transceiver capabilities (size of rop, ropframe, etc.) + MN protocol version
        eOmn_comm_status_t  boardCommStatus;
        uint16_t            usedNumberOfRegularROPs;

        TheEthManager *ethManager;
        HostTransceiver transceiver;

        Properties properties;


    private:


        bool verifyBoard();



        bool cleanBoardBehaviour(void);
        // we keep isRunning() and we add a field in the reply of serviceStart()/Stop() which tells if the board is in run mode or not.
        bool isRunning(void);

        // lock of the object: on / off
        bool lock(bool on);


        bool verbosewhenok;
    };


} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







