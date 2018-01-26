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


#include "hostTransceiver.hpp"


using namespace std;


namespace eth {
           
    class AbstractEthResource
    {
    public:

        struct Properties
        {
            eOipv4addr_t        ipv4addr;
            eOipv4addressing_t  ipv4addressing;
            eObrd_ethtype_t     boardtype;
            eOversion_t         firmwareversion;
            eOdate_t            firmwaredate;
            string              ipv4addrString;
            string              ipv4addressingString;
            string              boardtypeString;
            string              boardnameString;
            string              firmwareString;
        };

        //AbstractEthResource();
       // virtual ~AbstractEthResource() = 0;


        virtual bool open2(eOipv4addr_t remIP, yarp::os::Searchable &cfgtotal) = 0;
        virtual bool close() = 0;


        virtual const Properties & getProperties() = 0;


        virtual const void * getUDPtransmit(eOipv4addressing_t &destination, size_t &sizeofpacket, uint16_t &numofrops) = 0;

        virtual bool processRXpacket(const void *data, const size_t size) = 0;

        virtual bool getRemoteValue(const eOprotID32_t id32, void *value, const double timeout = 0.100, const unsigned int retries = 0) = 0;

        virtual bool getRemoteValues(const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout = 0.500) = 0;
       
        virtual bool setRemoteValue(const eOprotID32_t id32, void *value) = 0;

        virtual bool setcheckRemoteValue(const eOprotID32_t id32, void *value, const unsigned int retries = 10, const double waitbeforecheck = 0.001, const double timeout = 0.050) = 0;

        virtual bool getLocalValue(const eOprotID32_t id32, void *value) = 0;

        virtual bool setLocalValue(eOprotID32_t id32, const void *value, bool overrideROprotection = false) = 0;

        virtual bool verifyEPprotocol(eOprot_endpoint_t ep) = 0;

        virtual bool CANPrintHandler(eOmn_info_basic_t* infobasic) = 0;

        virtual bool serviceVerifyActivate(eOmn_serv_category_t category, const eOmn_serv_parameter_t* param, double timeout = 0.500) = 0;

        virtual bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500) = 0;

        virtual bool serviceStart(eOmn_serv_category_t category, double timeout = 0.500) = 0;

        virtual bool serviceStop(eOmn_serv_category_t category, double timeout = 0.500) = 0;

        virtual bool Tick() = 0;
        virtual bool Check() = 0;

        virtual bool isFake() = 0;

        virtual HostTransceiver * getTransceiver() = 0;

    };

} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







