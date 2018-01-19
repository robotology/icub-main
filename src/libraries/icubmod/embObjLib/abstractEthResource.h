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

#ifndef _ABSTRACTETHRESOURCE_H_
#define _ABSTRACTETHRESOURCE_H_


#include "EoProtocol.h"
#include <vector>
#include <yarp/os/Searchable.h>

#include "ace/INET_Addr.h"

using namespace std;


namespace eth {
           
    class AbstractEthResource
    {
    public:

        //AbstractEthResource();
       // virtual ~AbstractEthResource() = 0;


        virtual bool open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal) = 0;
        virtual bool close() = 0;

        virtual bool isID32supported(eOprotID32_t id32) = 0;

        virtual eOipv4addr_t getIPv4(void) = 0;
        virtual bool getIPv4addressing(eOipv4addressing_t &addressing) = 0;

        virtual const string & getIPv4string(void) = 0;

        virtual const string & getName(void) = 0;
        virtual eObrd_ethtype_t getBoardType(void) = 0;
        virtual const string & getBoardTypeString(void) = 0;


        // the function returns true if the packet can be transmitted.
        // it returns false if it cannot be transmitted: either it is with no rops inside in mode donttrxemptypackets, or there is an error somewhere
        virtual bool getTXpacket(uint8_t **packet, uint16_t *size, uint16_t *numofrops) = 0;

        virtual bool canProcessRXpacket(uint64_t *data, uint16_t size) = 0;

        virtual void processRXpacket(uint64_t *data, uint16_t size) = 0;

        virtual bool getRemoteValue(const eOprotID32_t id32, void *value, const double timeout = 0.100, const unsigned int retries = 0) = 0;

        virtual bool setRemoteValue(const eOprotID32_t id32, void *value) = 0;

        virtual bool setcheckRemoteValue(const eOprotID32_t id32, void *value, const unsigned int retries = 10, const double waitbeforecheck = 0.001, const double timeout = 0.050) = 0;


        virtual bool verifyEPprotocol(eOprot_endpoint_t ep) = 0;

        virtual bool CANPrintHandler(eOmn_info_basic_t* infobasic) = 0;

        virtual bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500) = 0;

        virtual bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500) = 0;

        virtual bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500) = 0;

        virtual bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500) = 0;

        virtual bool Tick() = 0;
        virtual bool Check() = 0;

        virtual bool readBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size) = 0;
        virtual bool addSetMessage(eOprotID32_t id32, uint8_t* data) = 0;
        virtual bool addGetMessage(eOprotID32_t id32) = 0;
        virtual bool addGetMessage(eOprotID32_t id32, std::uint32_t signature) = 0;
        virtual EOnv* getNVhandler(eOprotID32_t id32, EOnv* nv) = 0;
        virtual bool addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data) = 0;
        virtual bool readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size) = 0;
        virtual bool isFake() = 0;

    };

} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







