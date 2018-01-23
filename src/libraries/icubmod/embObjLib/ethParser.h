// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
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

#ifndef _ETHPARSER_H_
#define _ETHPARSER_H_

#include <string>

#include "EoProtocol.h"
#include <yarp/os/Searchable.h>


namespace eth { namespace parser {

    // -- library eth parser

    struct boardProperties
    {
        eOipv4addressing_t  ipv4addressing;
        eObrd_ethtype_t     type;
        std::uint16_t       maxSizeRXpacket;
        std::uint16_t       maxSizeROP;
        std::string         ipv4addressingstring;
        std::string         ipv4string;
        std::string         typestring;

        boardProperties() {
            reset();
        }
        void reset() {
            type = eobrd_ethtype_none; ipv4addressing.addr = 0; ipv4addressing.port = 0;
            maxSizeRXpacket = 768; maxSizeROP = 384; ipv4string = ""; ipv4addressingstring = "";
        }
    };

    struct boardSettings
    {
        std::string         name;
        eOmn_appl_config_t  txconfig;

        boardSettings()  {
            reset();
        }
        void reset() {
            name = "none"; txconfig.cycletime = 1000; txconfig.txratedivider = 1;
            txconfig.maxtimeRX = 400; txconfig.maxtimeDO = 300; txconfig.maxtimeTX = 300;
        }
    };

    struct boardActions
    {
        bool    monitorpresence_enabled;
        double  monitorpresence_timeout;
        double  monitorpresence_periodofmissingreport;

        boardActions()  {
            reset();
        }
        void reset() {
            monitorpresence_enabled = true;
            monitorpresence_timeout = 0.020; monitorpresence_periodofmissingreport = 60.0;
        }
    };

    struct boardData
    {
        boardProperties         properties;
        boardSettings           settings;
        boardActions            actions;
        void reset() {
            properties.reset();
            settings.reset();
            actions.reset();
        }
    };


    struct pc104Data
    {
        bool embBoardsConnected;
        eOipv4addressing_t localaddressing;
        std::uint16_t  txrate;
        std::uint16_t rxrate;
        std::string addressingstring;
        void reset() {
            embBoardsConnected = true;
            localaddressing.addr = eo_common_ipv4addr(10, 0, 1, 104); localaddressing.port = 12345;
            txrate = 1; rxrate = 5;
            addressingstring = "10.0.1.104:12345";
        }
        void setdefault() {
            embBoardsConnected = true;
            localaddressing.addr = eo_common_ipv4addr(10, 0, 1, 104); localaddressing.port = 12345;
            txrate = 1; rxrate = 5;
            addressingstring = "10.0.1.104:12345";
        }
    };

    bool read(yarp::os::Searchable &cfgtotal, pc104Data &pc104data);
    bool read(yarp::os::Searchable &cfgtotal, boardData &boarddata);

    bool print(const pc104Data &pc104data);
    bool print(const boardData &boarddata);


}} // namespace eth { namespace parser {



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------










