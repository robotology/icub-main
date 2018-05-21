// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


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


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "ethParser.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
using yarp::os::Log;

#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>

using namespace yarp::os;


// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------


bool eth::parser::print(const pc104Data &pc104data)
{

    yDebug() << "eth::parser::print(pc104Data) for PC104:";

    yDebug() << "PC104/PC104IpAddress:PC104IpPort = " << pc104data.addressingstring;
    yDebug() << "PC104/PC104TXrate = " << pc104data.txrate;
    yDebug() << "PC104/PC104RXrate = " << pc104data.rxrate;

    return true;
}

bool eth::parser::print(const boardData &boarddata)
{

    yDebug() << "eth::parser::print(boardData) for BOARD" << boarddata.settings.name;

    yDebug() << "ETH_BOARD/ETH_BOARD_PROPERTIES:";
    yDebug() << "ETH_BOARD/ETH_BOARD_PROPERTIES/IpAddress = " << boarddata.properties.ipv4string;
    yDebug() << "ETH_BOARD/ETH_BOARD_PROPERTIES/IpPort = " << boarddata.properties.ipv4addressing.port;
    yDebug() << "ETH_BOARD/ETH_BOARD_PROPERTIES/Type = " << boarddata.properties.typestring;
    yDebug() << "ETH_BOARD/ETH_BOARD_PROPERTIES/maxSizeRXpacket = " << boarddata.properties.maxSizeRXpacket;
    yDebug() << "ETH_BOARD/ETH_BOARD_PROPERTIES/maxSizeROP = " << boarddata.properties.maxSizeROP;

    yDebug() << "ETH_BOARD/ETH_BOARD_SETTINGS:";
    yDebug() << "ETH_BOARD/ETH_BOARD_SETTINGS/Name = " << boarddata.settings.name;
    yDebug() << "ETH_BOARD/ETH_BOARD_SETTINGS/RUNNINGMODE/(period, maxTimeOfRXactivity, maxTimeOfDOactivity, maxTimeOfTXactivity, TXrateOfRegularROPs) = " <<
                boarddata.settings.txconfig.cycletime << boarddata.settings.txconfig.maxtimeRX << boarddata.settings.txconfig.maxtimeDO << boarddata.settings.txconfig.maxtimeTX << boarddata.settings.txconfig.txratedivider;
    yDebug() << "ETH_BOARD/ETH_BOARD_ACTIONS/MONITOR_ITS_PRESENCE";
    yDebug() << "ETH_BOARD/ETH_BOARD_ACTIONS/MONITOR_ITS_PRESENCE/(enabled, timeout, periodOfMissingReport) = " <<
                boarddata.actions.monitorpresence_enabled << boarddata.actions.monitorpresence_timeout << boarddata.actions.monitorpresence_periodofmissingreport;

    return true;
}

bool eth::parser::read(yarp::os::Searchable &cfgtotal, pc104Data &pc104data)
{
    pc104data.setdefault();

    Bottle groupDEBUG  = cfgtotal.findGroup("DEBUG");
    if ((! groupDEBUG.isNull()) && (groupDEBUG.check("embBoardsConnected")))
    {
        pc104data.embBoardsConnected = groupDEBUG.find("embBoardsConnected").asBool();
    }

    if(!pc104data.embBoardsConnected)
    {
        yError() << "ATTENTION: NO EMBEDDED BOARDS CONNECTED. YOU ARE IN DEBUG MODE";
    }

    // localaddress

    Bottle groupPC104  = Bottle(cfgtotal.findGroup("PC104"));
    if (groupPC104.isNull())
    {
        yError() << "eth::parser::read() cannot find PC104 group in config files";
        return false;
    }

    Value *value;

    if (!groupPC104.check("PC104IpAddress", value))
    {
        yError() << "eth::parser::read(): missing PC104/PC104IpAddress in config files";
        return false;
    }
    if (!groupPC104.check("PC104IpPort", value))
    {
        yError() << "eth::parser::read(): missing PC104/PC104IpPort in config files";
        return false;
    }

    Bottle paramIPaddress(groupPC104.find("PC104IpAddress").asString());
    uint16_t port = groupPC104.find("PC104IpPort").asInt();              // .get(1).asInt();
    char strIP[64] = {0};


    snprintf(strIP, sizeof(strIP), "%s", paramIPaddress.toString().c_str());
    // strIP is now "10.0.1.104" ... i want to remove the "".... VERY IMPORTANT: use \" in sscanf
    int ip1, ip2, ip3, ip4;
    sscanf(strIP, "\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);

    pc104data.localaddressing.addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    pc104data.localaddressing.port = port;

    char ss[30];
    snprintf(ss, sizeof(ss), "%d.%d.%d.%d:%d", ip1, ip2, ip3, ip4, port);

    pc104data.addressingstring = ss;


    // txrate
    if(cfgtotal.findGroup("PC104").check("PC104TXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104TXrate").asInt();
        if(value > 0)
        {
            pc104data.txrate = value;
        }
    }
    else
    {
        yWarning () << "eth::parser::read() cannot find ETH/PC104TXrate. thus using default value" << pc104data.txrate;
    }

    // rxrate
    if(cfgtotal.findGroup("PC104").check("PC104RXrate"))
    {
        int value = cfgtotal.findGroup("PC104").find("PC104RXrate").asInt();
        if(value > 0)
        {
            pc104data.rxrate = value;
        }
    }
    else
    {
        yWarning () << "eth::parser::read() cannot find ETH/PC104RXrate. thus using default value" << pc104data.rxrate;
    }

    // now i print all the found values

    //print(pc104data);

    return true;
}


bool eth::parser::read(yarp::os::Searchable &cfgtotal, boardData &boarddata)
{
    Bottle groupEthBoard  = Bottle(cfgtotal.findGroup("ETH_BOARD"));
    if(groupEthBoard.isNull())
    {
        yError() << "eth::parser::read() cannot find ETH_BOARD group in config files";
        return false;
    }
    Bottle groupEthBoardProps = Bottle(groupEthBoard.findGroup("ETH_BOARD_PROPERTIES"));
    if(groupEthBoardProps.isNull())
    {
        yError() << "eth::parser::read() cannot find ETH_BOARD_PROPERTIES group in config files";
        return false;
    }
    Bottle groupEthBoardSettings = Bottle(groupEthBoard.findGroup("ETH_BOARD_SETTINGS"));
    if(groupEthBoardSettings.isNull())
    {
        yError() << "eth::parser::read() cannot find ETH_BOARD_PROPERTIES group in config files";
        return false;
    }

    // -> ETH_BOARD/ETH_BOARD_PROPERTIES

    boarddata.properties.reset();

    // IpAddress:
    if(true == groupEthBoardProps.check("IpAddress"))
    {
        Bottle paramIPboard(groupEthBoardProps.find("IpAddress").asString());
        char str[64] = {0};
        strcpy(str, paramIPboard.toString().c_str());
        int ip1, ip2, ip3, ip4;
        sscanf(str, "\"%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);
        boarddata.properties.ipv4addressing.addr = eo_common_ipv4addr(ip1, ip2, ip3, ip4);
    }
    else
    {
        yError() << "eth::parser::read() cannot find ETH_BOARD_PROPERTIES/IpAddress group in config files";
        return false;
    }

    char ipinfo[20] = {0};
    eo_common_ipv4addr_to_string(boarddata.properties.ipv4addressing.addr, ipinfo, sizeof(ipinfo));
    boarddata.properties.ipv4string = ipinfo;


    // IpPort:
    if(true == groupEthBoardProps.check("IpPort"))
    {
        boarddata.properties.ipv4addressing.port = groupEthBoardProps.find("IpPort").asInt();;
    }
    else
    {
        boarddata.properties.ipv4addressing.port = 12345;
        yWarning() << "eth::parser::read() cannot find ETH_BOARD_PROPERTIES/IpPort group in config files." << " using:" << boarddata.properties.ipv4addressing.port;
    }

    snprintf(ipinfo, sizeof(ipinfo), ":%d", boarddata.properties.ipv4addressing.port);
    boarddata.properties.ipv4addressingstring = boarddata.properties.ipv4string + ipinfo;


    // Type:
    Bottle b_ETH_BOARD_PROPERTIES_Type = groupEthBoardProps.findGroup("Type");
    std::string Type = b_ETH_BOARD_PROPERTIES_Type.get(1).asString();
    const char *strType = Type.c_str();
    // 1. compare with the exceptions which may be in some old xml files ("EMS4", "MC4PLUS", "MC2PLUS"), and then then call proper functions
    if(0 == strcmp(strType, "EMS4"))
    {
        boarddata.properties.type = eobrd_ethtype_ems4;
    }
    else if(0 == strcmp(strType, "MC4PLUS"))
    {
        boarddata.properties.type = eobrd_ethtype_mc4plus;
    }
    else if(0 == strcmp(strType, "MC2PLUS"))
    {
        boarddata.properties.type = eobrd_ethtype_mc2plus;
    }
    else
    {
        eObrd_type_t brd = eobrd_unknown;
        if(eobrd_unknown == (brd = eoboards_string2type2(strType, eobool_true)))
        {
            brd = eoboards_string2type2(strType, eobool_false);
        }

        // if not found in compact or extended string format, we accept that the board is unknown

        boarddata.properties.type = eoboards_type2ethtype(brd);
    }
    char boardTypeString[30];
    snprintf(boardTypeString, sizeof(boardTypeString), "%s", eoboards_type2string2(eoboards_ethtype2type(boarddata.properties.type), eobool_true));

    boarddata.properties.typestring = boardTypeString;

    // maxSizeRXpacket:
    if(true == groupEthBoardProps.check("maxSizeRXpacket"))
    {
        boarddata.properties.maxSizeRXpacket = groupEthBoardProps.find("maxSizeRXpacket").asInt();
        //yDebug() << "eth::parser::read() has detected capacityofTXpacket =" << boarddata.properties.maxSizeRXpacket << "for BOARD w/ IP" << boarddata.properties.ipv4string;
    }
    else
    {
        boarddata.properties.maxSizeRXpacket = 768;
        yWarning() << "eth::parser::read() in BOARD w/ IP" << boarddata.properties.ipv4string << "cannot find: capacityofTXpacket. using:" << boarddata.properties.maxSizeRXpacket;
    }

    // maxSizeROP:
    if(true == groupEthBoardProps.check("maxSizeROP"))
    {
        boarddata.properties.maxSizeROP = groupEthBoardProps.find("maxSizeROP").asInt();
        //yDebug() << "eth::parser::read() has detected maxSizeOfROP =" << boarddata.properties.maxSizeROP << "for BOARD w/ IP" << boarddata.properties.ipv4string;
    }
    else
    {
        boarddata.properties.maxSizeROP = 384;
        yWarning() << "eth::parser::read() in BOARD w/ IP" << boarddata.properties.ipv4string << "cannot find: maxSizeROP. using:" << boarddata.properties.maxSizeROP;
    }



    // <- ETH_BOARD/ETH_BOARD_PROPERTIES


    // -> ETH_BOARD/ETH_BOARD_SETTINGS

    Bottle paramNameBoard(groupEthBoardSettings.find("Name").asString());
    char xmlboardname[64] = {0};
    snprintf(xmlboardname, sizeof(xmlboardname), "%s", paramNameBoard.toString().c_str());


    if(0 != strlen(xmlboardname))
    {
        boarddata.settings.name = xmlboardname;
    }
    else
    {
        boarddata.settings.name = "NOT-NAMED";
    }


    // -> ETH_BOARD/ETH_BOARD_SETTINGS/RUNNINGMODE

    Bottle groupEthBoardSettings_RunningMode = Bottle(groupEthBoardSettings.findGroup("RUNNINGMODE"));
    if(groupEthBoardSettings_RunningMode.isNull())
    {
        yWarning() << "eth::parser::read(): cannot find ETH_BOARD_PROPERTIES/RUNNINGMODE group in config files for BOARD w/ IP" << boarddata.properties.ipv4string << " and will use default values";
        yWarning() << "Default values for ETH_BOARD_PROPERTIES/RUNNINGMODE group: (period, maxTimeOfRXactivity, maxTimeOfDOactivity, maxTimeOfTXactivity, TXrateOfRegularROPs) = " <<
                      boarddata.settings.txconfig.cycletime << boarddata.settings.txconfig.maxtimeRX << boarddata.settings.txconfig.maxtimeDO << boarddata.settings.txconfig.maxtimeTX << boarddata.settings.txconfig.txratedivider;
    }
    else
    {

        if(true == groupEthBoardSettings_RunningMode.check("period"))
        {
            int tmp = groupEthBoardSettings_RunningMode.find("period").asInt();

            if(1000 != tmp)
            {
                yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "ETH_BOARD_SETTINGS::RUNNINGMODE::period can be only 1000 (so far) and it was:" << tmp;
                tmp = 1000;
            }
            boarddata.settings.txconfig.cycletime = tmp;
        }

        if(true == groupEthBoardSettings_RunningMode.check("maxTimeOfRXactivity"))
        {
            int tmp = groupEthBoardSettings_RunningMode.find("maxTimeOfRXactivity").asInt();

            if((tmp < 5) || (tmp > 990))
            {
                yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "ETH_BOARD_SETTINGS::RUNNINGMODE::maxTimeOfRXactivity must be in [5, 990] (so far) and it was:" << tmp << "Using default value";
                tmp = 400;
            }

            boarddata.settings.txconfig.maxtimeRX = tmp;
        }

        if(true == groupEthBoardSettings_RunningMode.check("maxTimeOfDOactivity"))
        {
            int tmp = groupEthBoardSettings_RunningMode.find("maxTimeOfDOactivity").asInt();

            if((tmp < 5) || (tmp > 990))
            {
                yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "ETH_BOARD_SETTINGS::RUNNINGMODE::maxtimeOfDOactivity must be in [5, 990] (so far) and it was:" << tmp << "Using default value";
                tmp = 300;
            }

            boarddata.settings.txconfig.maxtimeDO = tmp;
        }

        if(true == groupEthBoardSettings_RunningMode.check("maxTimeOfTXactivity"))
        {
            int tmp = groupEthBoardSettings_RunningMode.find("maxTimeOfTXactivity").asInt();

            if((tmp < 5) || (tmp > 990))
            {
                yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "ETH_BOARD_SETTINGS::RUNNINGMODE::maxTimeOfTXactivity must be in [5, 990] (so far) and it was:" << tmp << "Using default value";
                tmp = 300;
            }

            boarddata.settings.txconfig.maxtimeTX = tmp;
        }


        if(true == groupEthBoardSettings_RunningMode.check("TXrateOfRegularROPs"))
        {
            int tmp = groupEthBoardSettings_RunningMode.find("TXrateOfRegularROPs").asInt();

            if(tmp <=0)
            {
                yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "ETH_BOARD_SETTINGS::RUNNINGMODE::TXrateOfRegularROPs must be in [1, 20] and it was:" << tmp << "Using value = 1";
                tmp = 1;
            }
            if(tmp >200)
            {
                yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "ETH_BOARD_SETTINGS::RUNNINGMODE::TXrateOfRegularROPs must be in [1, 20] and it was:" << tmp << "Using value = 20";
                tmp = 20;
            }
            boarddata.settings.txconfig.txratedivider = tmp;
        }

        // consistency check
        if((boarddata.settings.txconfig.maxtimeRX+boarddata.settings.txconfig.maxtimeDO+boarddata.settings.txconfig.maxtimeTX) != boarddata.settings.txconfig.cycletime)
        {
            yWarning() << "eth::parser::read() for BOARD" << boarddata.properties.ipv4string << "In ETH_BOARD_SETTINGS::RUNNINGMODE sum(maxTimeOfRXactivity, maxTimeOfDOactivity, maxTimeOfTXactivity) != period !!! Using default values";

            yError() << boarddata.settings.txconfig.maxtimeRX+boarddata.settings.txconfig.maxtimeDO+boarddata.settings.txconfig.maxtimeTX;
            boarddata.settings.txconfig.cycletime = 1000;
            boarddata.settings.txconfig.maxtimeRX = 400;
            boarddata.settings.txconfig.maxtimeDO = 300;
            boarddata.settings.txconfig.maxtimeTX = 300;
        }

    }



    // <- ETH_BOARD/ETH_BOARD_SETTINGS/RUNNINGMODE


    // <- ETH_BOARD/ETH_BOARD_SETTINGS


    // -> ETH_BOARD/ETH_BOARD_ACTIONS
    // -> ETH_BOARD/ETH_BOARD_ACTIONS/MONITOR_ITS_PRESENCE


    // do we have a proper section ETH_BOARD_ACTIONS/MONITOR_ITS_PRESENCE? if so we change its config

    Bottle groupEthBoardActions = Bottle(groupEthBoard.findGroup("ETH_BOARD_ACTIONS"));
    if(!groupEthBoardActions.isNull())
    {
        Bottle groupEthBoardActions_Monitor = Bottle(groupEthBoardActions.findGroup("MONITOR_ITS_PRESENCE"));
        if(!groupEthBoardActions_Monitor.isNull())
        {
            Bottle groupEthBoardActions_Monitor_enabled = groupEthBoardActions_Monitor.findGroup("enabled");
            std::string Ena = groupEthBoardActions_Monitor_enabled.get(1).asString();
            const char *strEna = Ena.c_str();

            if(0 == strcmp(strEna, "true"))
            {
                boarddata.actions.monitorpresence_enabled = true;
            }
            else
            {
                boarddata.actions.monitorpresence_enabled = false;
            }

            if(true == groupEthBoardActions_Monitor.check("timeout"))
            {
                double presenceTimeout = groupEthBoardActions_Monitor.find("timeout").asDouble();

                if(presenceTimeout <= 0)
                {
                    presenceTimeout = 0;
                    boarddata.actions.monitorpresence_enabled = false;
                }

                if(presenceTimeout > 0.100)
                {
                    presenceTimeout = 0.100;
                }

                boarddata.actions.monitorpresence_timeout = presenceTimeout;

            }


            if(true == groupEthBoardActions_Monitor.check("periodOfMissingReport"))
            {
                double reportMissingPeriod = groupEthBoardActions_Monitor.find("periodOfMissingReport").asDouble();

                if(reportMissingPeriod <= 0)
                {
                    reportMissingPeriod = 0.0;
                }

                if(reportMissingPeriod > 600)
                {
                    reportMissingPeriod = 600;
                }

                boarddata.actions.monitorpresence_periodofmissingreport = reportMissingPeriod;
            }
        }
    }

    // <- ETH_BOARD/ETH_BOARD_ACTIONS/MONITOR_ITS_PRESENCE
    // <- ETH_BOARD/ETH_BOARD_ACTIONS

    return true;
}


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


