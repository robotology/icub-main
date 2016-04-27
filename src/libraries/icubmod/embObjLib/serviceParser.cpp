
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2016 Robotcub Consortium
* Author: Marco Accame
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// general purpose stuff.
#include <string>
#include <iostream>
#include <string.h>

// Yarp Includes
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>


// specific to this device driver.
#include <serviceParser.h>

#include <yarp/os/LogStream.h>
#include "EoAnalogSensors.h"


#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"

#include <yarp/os/NetType.h>

#ifdef WIN32
#pragma warning(once:4355)
#endif



using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;


ServiceParser::ServiceParser()
{
    // how do i reset variable as_service?

    boards.resize(0);

    as_service.type = eomn_serv_NONE;

    as_service.properties.canboards.resize(0);
    as_service.properties.sensors.resize(0);

    as_service.settings.acquisitionrate = 0;
    as_service.settings.enabledsensors.resize(0);
}


bool ServiceParser::parseBoardVersions(Searchable &config)
{
    // format is BOARDVERSIONS{ type, PROTOCOL{ major, minor }, FIRMWARE{ major, minor, build } }

    Bottle b_BOARDVERSIONS(config.findGroup("BOARDVERSIONS"));
    if(b_BOARDVERSIONS.isNull())
    {
        yWarning() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS group. must use locally defined PROTOCOL and FIRMWARE";
        boards.resize(0);
        return false;
    }


    // now get type, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. the must be all equal.
    // for mais and strain and so far for intertials it must be numboards = 1.

    Bottle b_BOARDVERSIONS_type = b_BOARDVERSIONS.findGroup("type");
    if(b_BOARDVERSIONS_type.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.type";
        return false;
    }
    Bottle b_BOARDVERSIONS_PROTOCOL = Bottle(b_BOARDVERSIONS.findGroup("PROTOCOL"));
    if(b_BOARDVERSIONS_PROTOCOL.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.PROTOCOL";
        return false;
    }
    Bottle b_BOARDVERSIONS_PROTOCOL_major = Bottle(b_BOARDVERSIONS_PROTOCOL.findGroup("major"));
    if(b_BOARDVERSIONS_PROTOCOL_major.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.PROTOCOL.major";
        return false;
    }
    Bottle b_BOARDVERSIONS_PROTOCOL_minor = Bottle(b_BOARDVERSIONS_PROTOCOL.findGroup("minor"));
    if(b_BOARDVERSIONS_PROTOCOL_minor.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.PROTOCOL.minor";
        return false;
    }
    Bottle b_BOARDVERSIONS_FIRMWARE = Bottle(b_BOARDVERSIONS.findGroup("FIRMWARE"));
    if(b_BOARDVERSIONS_FIRMWARE.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.FIRMWARE";
        return false;
    }
    Bottle b_BOARDVERSIONS_FIRMWARE_major = Bottle(b_BOARDVERSIONS_FIRMWARE.findGroup("major"));
    if(b_BOARDVERSIONS_FIRMWARE_major.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.FIRMWARE.major";
        return false;
    }
    Bottle b_BOARDVERSIONS_FIRMWARE_minor = Bottle(b_BOARDVERSIONS_FIRMWARE.findGroup("minor"));
    if(b_BOARDVERSIONS_FIRMWARE_minor.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.FIRMWARE.minor";
        return false;
    }
    Bottle b_BOARDVERSIONS_FIRMWARE_build = Bottle(b_BOARDVERSIONS_FIRMWARE.findGroup("build"));
    if(b_BOARDVERSIONS_FIRMWARE_build.isNull())
    {
        yError() << "ServiceParser::parseBoardVersions() cannot find BOARDVERSIONS.FIRMWARE.build";
        return false;
    }

    int tmp = b_BOARDVERSIONS_type.size();
    int numboards = tmp - 1;    // first position of bottle contains the tag "type"

    // check if all other fields have the same size.
    if( (tmp != b_BOARDVERSIONS_PROTOCOL_major.size()) ||
        (tmp != b_BOARDVERSIONS_PROTOCOL_minor.size()) ||
        (tmp != b_BOARDVERSIONS_FIRMWARE_major.size()) ||
        (tmp != b_BOARDVERSIONS_FIRMWARE_minor.size()) ||
        (tmp != b_BOARDVERSIONS_FIRMWARE_build.size())
      )
    {
        yError() << "ServiceParser::parseBoardVersions() in BOARDVERSIONS some param has inconsistent length";
        return false;
    }


    boards.resize(0);

    bool formaterror = false;
    for(int i=0; i<numboards; i++)
    {
        servBoard_t item = { .type = eobrd_none, .protocol = {0}, .firmware = {0} };


        convert(b_BOARDVERSIONS_type.get(i+1).asString(), item.type, formaterror);

        convert(b_BOARDVERSIONS_PROTOCOL_major.get(i+1).asInt(), item.protocol.major, formaterror);
        convert(b_BOARDVERSIONS_PROTOCOL_minor.get(i+1).asInt(), item.protocol.minor, formaterror);

        convert(b_BOARDVERSIONS_FIRMWARE_major.get(i+1).asInt(), item.firmware.major, formaterror);
        convert(b_BOARDVERSIONS_FIRMWARE_minor.get(i+1).asInt(), item.firmware.minor, formaterror);
        convert(b_BOARDVERSIONS_FIRMWARE_build.get(i+1).asInt(), item.firmware.build, formaterror);

        boards.push_back(item);
    }

    // in here we could decide to return false if any previous conversion function has returned error
    // bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

    if(true == formaterror)
    {
        yError() << "ServiceParser::parseBoardVersions() has detected an illegal format for some of the params of BOARDVERSIONS some param has inconsistent length";
        boards.resize(0);
        return false;
    }


    return true;
}


bool ServiceParser::convert(ConstString const &fromstring, eOmn_serv_type_t& toservicetype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    toservicetype = eomn_string2servicetype(t);

    if(eomn_serv_UNKNOWN == toservicetype)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmn_serv_type_t";
        formaterror = true;
        return false;
    }

    return true;

}


bool ServiceParser::convert(ConstString const &fromstring, eOas_sensor_t &tosensortype, bool &formaterror)
{
    const char *t = fromstring.c_str();

    tosensortype = eoas_string2sensor(t);

    if(eoas_unknown == tosensortype)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOas_sensor_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(ConstString const &fromstring, eObrd_type_t& tobrdtype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    tobrdtype = eoboards_string2type(t);
    if(eobrd_unknown == tobrdtype)
    {
        yWarning() << "ServiceParser::convert(): string" << t << "cannot be converted into a proper eObrd_type_t";
        formaterror = true;
        return false;
    }

    return true;
}



bool ServiceParser::convert(ConstString const &fromstring, eObrd_cantype_t& tobrdcantype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObrd_type_t type = eoboards_string2type(t);
    if(eobrd_unknown == type)
    {
        yWarning() << "ServiceParser::convert(): string" << t << "cannot be converted into a proper eObrd_type_t";
        formaterror = true;
        return false;
    }

    tobrdcantype = eoboards_type2cantype(type);
    if(eobrd_cantype_unknown == tobrdcantype)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eObrd_cantype_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::convert(ConstString const &fromstring, bool& tobool, bool& formaterror)
{
    const char *t = fromstring.c_str();

    if(0 == strcmp(t, "true"))
    {
        tobool = true;
    }
    else if(0 == strcmp(t, "false"))
    {
        tobool = false;
    }
    else if(0 == strcmp(t, "TRUE"))
    {
        tobool = true;
    }
    else if(0 == strcmp(t, "FALSE"))
    {
        tobool = false;
    }
    else if(0 == strcmp(t, "1"))
    {
        tobool = true;
    }
    else if(0 == strcmp(t, "0"))
    {
        tobool = false;
    }
    else
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for bool";
        tobool = false;
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(const int number, uint8_t& tou8, bool& formaterror)
{
    if((number >= 0) && (number < 256))
    {
        tou8 = number;
    }
    else
    {
        yWarning() << "ServiceParser::convert():" << number << "is not a legal value for uint8_t";
        tou8 = 0;
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::convert(const int number, uint16_t& tou16, bool& formaterror)
{
    if((number >= 0) && (number < (64*1024)))
    {
        tou16 = number;
    }
    else
    {
        yWarning() << "ServiceParser::convert():" << number << "is not a legal value for uint16_t";
        tou16 = 0;
        formaterror = true;
        return false;
    }

    return true;
}



bool ServiceParser::convert(ConstString const &fromstring, const uint8_t strsize, char *str, bool &formaterror)
{
    const char *t = fromstring.c_str();

    if((NULL == str) || (0 == strsize))
    {
        yWarning() << "ServiceParser::convert(): there is an attempt to convert string" << t << "into a NULL or zero-sized char *str: len = " << strsize;
        formaterror = true;
        return false;
    }

    if(strsize >= strlen(t))
    {
        snprintf(str, strsize, "%s", t);
    }
    else
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a char[] of max size" << strsize;
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(ConstString const &fromstring, string &str, bool &formaterror)
{
    const char *t = fromstring.c_str();

    if(0 != strlen(t))
    {
        str = t;
    }
    else
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string";
        formaterror = true;
        return false;
    }

    return true;
}



bool ServiceParser::convert(ConstString const &fromstring, eObrd_location_t &location, bool &formaterror)
{
    // it is actually a micro-parser: PRE-num
    // at

    const char *t = fromstring.c_str();
    int len = strlen(t);

    if(len > 15)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a eObrd_location_t because it is too long with size =" << len;
        formaterror = true;
        return false;
    }

    char prefix[16] = {0};
    sscanf(t, "%3c", prefix);
    if(0 == strcmp(prefix, "ETH"))
    {
        int adr = 0;
        sscanf(t, "%3c:%d", prefix, &adr);
        location.any.place = eobrd_place_eth;
        location.eth.id = adr;
    }
    else if(0 == strcmp(prefix, "CAN"))
    {
        int bus = 0;
        int adr = 0;
        char cc = 'x';
        int sub = 9;
        int numberofreaditems = sscanf(t, "%3c%d:%d%c%d", prefix, &bus, &adr, &cc, &sub);

        if((3 != numberofreaditems) && (5 != numberofreaditems))
        {
            yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a eObrd_location_t because we dont have correct number of sections separated by :";
            formaterror = true;
            return false;
        }

        // verify bus being eitehr 1 or 2, and adr being 1 ----- 14
        if((1 != bus) && (2 != bus))
        {
            yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a eObrd_location_t because we can have either CAN1 or CAN2";
            formaterror = true;
            return false;
        }
        if((0 == adr) || (adr > 14))
        {
            yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a eObrd_location_t because CAN address is in range [1, 14]";
            formaterror = true;
            return false;
        }

        location.any.place = (3 == numberofreaditems) ? (eobrd_place_can) : (eobrd_place_extcan);
        if(eobrd_place_can == location.any.place)
        {
            location.can.port = (1 == bus) ? (eOcanport1) : (eOcanport2);
            location.can.addr = adr;
            location.can.ffu = 0;
        }
        else
        {
            location.extcan.port = (1 == bus) ? (eOcanport1) : (eOcanport2);
            location.extcan.addr = adr;
            if((0 != sub) && (1 != sub))
            {
                yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a eObrd_location_t because in CANx:adr:SUB, SUB address must be in range [0, 1]";
                formaterror = true;
                return false;
            }
            location.extcan.index = (0 == sub) ? (eobrd_caninsideindex_first) : (eobrd_caninsideindex_second);
        }

    }
    else
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a eObrd_location_t because it does not begin with ETH or CAN";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::convert(eObrd_location_t const &loc, char *str, int len)
{
    if((NULL == str) || (0 == len))
    {
        return false;
    }

    if(eobrd_place_can == loc.any.place)
    {
        snprintf(str, len, "CAN%d:%d", (eOcanport1 == loc.can.port) ? 1 : 2, loc.can.addr);
    }
    else if(eobrd_place_extcan == loc.any.place)
    {
        snprintf(str, len, "CAN%d:%d:%d", (eOcanport1 == loc.extcan.port) ? 1 : 2, loc.extcan.addr, loc.extcan.index);
    }
    else if(eobrd_place_eth == loc.any.place)
    {   // it must be eobrd_place_eth
        snprintf(str, len, "ETH:%d", loc.eth.id);
    }
    else
    {
        return false;
    }

    return true;
}


bool ServiceParser::convert(eObrd_canlocation_t const &canloc, char *str, int len)
{
    if((NULL == str) || (0 == len))
    {
        return false;
    }

    if(eobrd_caninsideindex_none == canloc.insideindex)
    {
        snprintf(str, len, "CAN%d:%d", (eOcanport1 == canloc.port) ? 1 : 2, canloc.addr);
    }
    else
    {
        snprintf(str, len, "CAN%d:%d:%d", (eOcanport1 == canloc.port) ? 1 : 2, canloc.addr, canloc.insideindex);
    }

    return true;
}

bool ServiceParser::convert(eObrd_firmwareversion_t const &firm, char *str, int len)
{
    if((NULL == str) || (0 == len))
    {
        return false;
    }

    snprintf(str, len, "(%d, %d, %d)", firm.major, firm.minor, firm.build);

    return true;
}

bool ServiceParser::convert(eObrd_protocolversion_t const &prot, char *str, int len)
{
    if((NULL == str) || (0 == len))
    {
        return false;
    }

    snprintf(str, len, "(%d, %d)", prot.major, prot.minor);

    return true;
}

bool ServiceParser::check_analog(Searchable &config, eOmn_serv_type_t type)
{
    bool formaterror = false;
    // so far we check for eomn_serv_AS_mais / strain / inertial only
    if((eomn_serv_AS_mais != type) && (eomn_serv_AS_strain != type) && (eomn_serv_AS_inertials != type))
    {
        yError() << "ServiceParser::check() is called with wrong type";
        return false;
    }

    // i parse the global board versions. if section is not found we can safely continue but we have boards.size() equal to zero.
    parseBoardVersions(config);

    // format is SERVICE{ type, PROPERTIES{ CANBOARDS, SENSORS }, SETTINGS }

    Bottle b_SERVICE(config.findGroup("SERVICE"));
    if(b_SERVICE.isNull())
    {
        yError() << "ServiceParser::check() cannot find SERVICE group";
        return false;
    }

    // check whether we have the proper type

    if(false == b_SERVICE.check("type"))
    {
        yError() << "ServiceParser::check() cannot find SERVICE.type";
        return false;
    }
    else
    {
        Bottle b_type(b_SERVICE.find("type").asString());
        if(false == convert(b_type.toString(), as_service.type, formaterror))
        {
            yError() << "ServiceParser::check() has found unknown SERVICE.type = " << b_type.toString();
            return false;
        }
        if(type != as_service.type)
        {
            yError() << "ServiceParser::check() has found wrong SERVICE.type = " << as_service.type << "it must be" << "TODO: tostring() function";
            return false;
        }
    }

    // check whether we have the proper groups

    Bottle b_PROPERTIES = Bottle(b_SERVICE.findGroup("PROPERTIES"));
    if(b_PROPERTIES.isNull())
    {
        yError() << "ServiceParser::check() cannot find PROPERTIES";
        return false;
    }
    else
    {
        Bottle b_PROPERTIES_CANBOARDS = Bottle(b_PROPERTIES.findGroup("CANBOARDS"));
        if(b_PROPERTIES_CANBOARDS.isNull())
        {
            yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS";
            return false;
        }
        else
        {
            // now get type, useGlobalParams, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. the must be all equal.
            // for mais and strain and so far for intertials it must be numboards = 1.

            Bottle b_PROPERTIES_CANBOARDS_type = b_PROPERTIES_CANBOARDS.findGroup("type");
            if(b_PROPERTIES_CANBOARDS_type.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.type";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_useGlobalParams = b_PROPERTIES_CANBOARDS.findGroup("useGlobalParams");
            if(b_PROPERTIES_CANBOARDS_useGlobalParams.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.useGlobalParams";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_PROTOCOL = Bottle(b_PROPERTIES_CANBOARDS.findGroup("PROTOCOL"));
            if(b_PROPERTIES_CANBOARDS_PROTOCOL.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_major = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("major"));
            if(b_PROPERTIES_CANBOARDS_PROTOCOL_major.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL.major";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_minor = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("minor"));
            if(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.PROTOCOL.minor";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE = Bottle(b_PROPERTIES_CANBOARDS.findGroup("FIRMWARE"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_major = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("major"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE_major.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.major";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_minor = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("minor"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.minor";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_build = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("build"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE_build.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.FIRMWARE.build";
                return false;
            }

            int tmp = b_PROPERTIES_CANBOARDS_type.size();
            int numboards = tmp - 1;    // first position of bottle contains the tag "type"

            // check if all other fields have the same size.
            if( (tmp != b_PROPERTIES_CANBOARDS_useGlobalParams.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_major.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_minor.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_major.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_minor.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_build.size())
              )
            {
                yError() << "ServiceParser::check() in PROPERTIES.CANBOARDS some param has inconsistent length";
                return false;
            }


            as_service.properties.canboards.resize(0);

            formaterror = false;
            for(int i=0; i<numboards; i++)
            {
                servCanBoard_t item = { .type = eobrd_cantype_none, .useglobalparams = false, .protocol = {0}, .firmware = {0} };

                convert(b_PROPERTIES_CANBOARDS_type.get(i+1).asString(), item.type, formaterror);
                convert(b_PROPERTIES_CANBOARDS_useGlobalParams.get(i+1).asString(), item.useglobalparams, formaterror);

                if(true == item.useglobalparams)
                {
                    // must search boards an entry which matches eObrd_type_t tt;
                    eObrd_type_t tt = eobrd_none;
                    servBoard_t brd = {.type = eobrd_none, .protocol = {0}, .firmware = {0} };
                    convert(b_PROPERTIES_CANBOARDS_type.get(i+1).asString(), tt, formaterror);

                    if(0 == boards.size())
                    {
                        yWarning() << "ServiceParser::check() in PROPERTIES.CANBOARDS: useGlobalParams is true for board" << eoboards_type2string(tt) << "but we dont have global params for boards. we shall use prot = (0,0) and firm = (0, 0, 0) ";
                    }
                    else
                    {
                        for(int i=0; i<boards.size(); i++)
                        {
                            servBoard_t cur = boards.at(i);
                            if(tt == cur.type)
                            {
                                brd = cur;
                                break;
                            }
                        }

                        if(eobrd_none == brd.type)
                        {
                            yWarning() << "ServiceParser::check() in PROPERTIES.CANBOARDS: useGlobalParams is true for board" << eoboards_type2string(tt) << "but we cannot find its global params. we shall use prot = (0,0) and firm = (0, 0, 0) ";
                        }
                    }

                    item.protocol.major = brd.protocol.major;
                    item.protocol.minor = brd.protocol.minor;

                    item.firmware.major = brd.firmware.major;
                    item.firmware.minor = brd.firmware.minor;
                    item.firmware.build = brd.firmware.build;

                }
                else
                {
                    convert(b_PROPERTIES_CANBOARDS_PROTOCOL_major.get(i+1).asInt(), item.protocol.major, formaterror);
                    convert(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.get(i+1).asInt(), item.protocol.minor, formaterror);

                    convert(b_PROPERTIES_CANBOARDS_FIRMWARE_major.get(i+1).asInt(), item.firmware.major, formaterror);
                    convert(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.get(i+1).asInt(), item.firmware.minor, formaterror);
                    convert(b_PROPERTIES_CANBOARDS_FIRMWARE_build.get(i+1).asInt(), item.firmware.build, formaterror);
                }

                as_service.properties.canboards.push_back(item);
            }

            // in here we could decide to return false if any previous conversion function has returned error
            // bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

            if(true == formaterror)
            {
                yError() << "ServiceParser::check() has detected an illegal format for some of the params of PROPERTIES.CANBOARDS some param has inconsistent length";
                return false;
            }
        }

        Bottle b_PROPERTIES_SENSORS = Bottle(b_PROPERTIES.findGroup("SENSORS"));
        if(b_PROPERTIES_SENSORS.isNull())
        {
            yError() << "ServiceParser::check() cannot find PROPERTIES.SENSORS";
            return false;
        }
        else
        {

            Bottle b_PROPERTIES_SENSORS_id = Bottle(b_PROPERTIES_SENSORS.findGroup("id"));
            if(b_PROPERTIES_SENSORS_id.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.SENSORS.id";
                return false;
            }
            Bottle b_PROPERTIES_SENSORS_type = Bottle(b_PROPERTIES_SENSORS.findGroup("type"));
            if(b_PROPERTIES_SENSORS_type.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.SENSORS.type";
                return false;
            }
            Bottle b_PROPERTIES_SENSORS_location = Bottle(b_PROPERTIES_SENSORS.findGroup("location"));
            if(b_PROPERTIES_SENSORS_location.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.SENSORS.location";
                return false;
            }

            int tmp = b_PROPERTIES_SENSORS_id.size();
            int numsensors = tmp - 1;    // first position of bottle contains the tag "id"

            // check if all other fields have the same size.
            if( (tmp != b_PROPERTIES_SENSORS_type.size()) ||
                (tmp != b_PROPERTIES_SENSORS_location.size())
              )
            {
                yError() << "ServiceParser::check() in PROPERTIES.SENSORS some param has inconsistent length";
                return false;
            }


            as_service.properties.sensors.resize(0);

            formaterror = false;
            for(int i=0; i<numsensors; i++)
            {
                servAnalogSensor_t item;
                item.type = eoas_none;
                item.location.any.place = eobrd_place_none;

                convert(b_PROPERTIES_SENSORS_id.get(i+1).asString(), item.id, formaterror);
                convert(b_PROPERTIES_SENSORS_type.get(i+1).asString(), item.type, formaterror);
                convert(b_PROPERTIES_SENSORS_location.get(i+1).asString(), item.location, formaterror);

                as_service.properties.sensors.push_back(item);
            }

            // in here we could decide to return false if any previous conversion function has returned error
            // bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

            if(true == formaterror)
            {
                yError() << "ServiceParser::check() has detected an illegal format for some of the params of PROPERTIES.SENSORS some param has inconsistent length";
                return false;
            }

        }

    }

    Bottle b_SETTINGS = Bottle(b_SERVICE.findGroup("SETTINGS"));
    if(b_SETTINGS.isNull())
    {
        yError() << "ServiceParser::check() cannot find SETTINGS";
        return false;
    }
    else
    {

        Bottle b_SETTINGS_acquisitionRate = Bottle(b_SETTINGS.findGroup("acquisitionRate"));
        if(b_SETTINGS_acquisitionRate.isNull())
        {
            yError() << "ServiceParser::check() cannot find SETTINGS.acquisitionRate";
            return false;
        }
        Bottle b_SETTINGS_enabledSensors = Bottle(b_SETTINGS.findGroup("enabledSensors"));
        if(b_SETTINGS_enabledSensors.isNull())
        {
            yError() << "ServiceParser::check() cannot find SETTINGS.enabledSensors";
            return false;
        }

        int numenabledsensors = b_SETTINGS_enabledSensors.size() - 1;    // first position of bottle contains the tag "enabledSensors"

        // the enabled must be <= the sensors.
        if( numenabledsensors > as_service.properties.sensors.size() )
        {
            yError() << "ServiceParser::check() in SETTINGS.enabledSensors there are too many items with respect to supported sensors:" << numenabledsensors << "vs." << as_service.properties.sensors.size();
            return false;
        }

        convert(b_SETTINGS_acquisitionRate.get(1).asInt(), as_service.settings.acquisitionrate, formaterror);


        as_service.settings.enabledsensors.resize(0);

        for(int i=0; i<numenabledsensors; i++)
        {
            servAnalogSensor_t founditem;

            ConstString s_enabled_id = b_SETTINGS_enabledSensors.get(i+1).asString();
//            const char *str = s_enabled_id.c_str();
//            std::string cpp_str = str;

            // we must now search inside the whole vector<> as_service.properties.sensors if we find an id which matches s_enabled_id ....
            // if we dont, ... we issue a warning.
            // if we find, ... we do a pushback of it inside
            bool found = false;
            // i decide to use a brute force search ... for now
            for(int n=0; n<as_service.properties.sensors.size(); n++)
            {
                servAnalogSensor_t item = as_service.properties.sensors.at(n);
                //if(item.id == cpp_str)
                if(item.id == s_enabled_id)
                {
                    found = true;
                    founditem = item;
                    break;
                }
            }

            if(true == found)
            {
                as_service.settings.enabledsensors.push_back(founditem);
            }

        }

        // in here we issue an error if we dont have at least one enabled sensor

        if(0 == as_service.settings.enabledsensors.size())
        {
            yError() << "ServiceParser::check() could not find any item in SETTINGS.enabledSensors which matches what in PROPERTIES.SENSORS.id";
            return false;
        }

    }

    // now we may have one or more sections which are specific of the device ...

    // only strain so far.

    if(eomn_serv_AS_strain == type)
    {
        Bottle b_STRAIN_SETTINGS = Bottle(b_SERVICE.findGroup("STRAIN_SETTINGS"));
        if(b_STRAIN_SETTINGS.isNull())
        {
            yError() << "ServiceParser::check() cannot find STRAIN_SETTINGS";
            return false;
        }
        else
        {

            Bottle b_STRAIN_SETTINGS_useCalibration = Bottle(b_STRAIN_SETTINGS.findGroup("useCalibration"));
            if(b_STRAIN_SETTINGS_useCalibration.isNull())
            {
                yError() << "ServiceParser::check() cannot find STRAIN_SETTINGS.useCalibration";
                return false;
            }

            formaterror = false;
            convert(b_STRAIN_SETTINGS_useCalibration.get(1).asString(), as_strain_settings.useCalibration, formaterror);

            if(true == formaterror)
            {
                yError() << "ServiceParser::check() has detected an illegal format for paramf STRAIN_SETTINGS.useCalibration";
                return false;
            }
        }
    }




    // we we are in here we have the struct filled with all variables ... some validations are still due to the calling device.
    // for instance, if embObjMais

    return true;
}


bool ServiceParser::parseService(Searchable &config, servConfigMais_t &maisconfig)
{
    if(false == check_analog(config, eomn_serv_AS_mais))
    {
        yError() << "ServiceParser::parseService() has received an invalid SERVICE group for mais";
        return false;
    }

    // now we extract values ... so far we dont make many checks ... we just assume the vector<> are of size 1.
    servCanBoard_t themais_props = as_service.properties.canboards.at(0);
    servAnalogSensor_t themais_sensor = as_service.settings.enabledsensors.at(0);


    // first check we do is about themais_props.type
    if(eobrd_cantype_mais != themais_props.type)
    {
        yError() << "ServiceParser::parseService() has detected an invalid type of board. it should be a eobrd_mais but is a:" << eoboards_type2string(eoboards_cantype2type(themais_props.type));
        return false;
    }

    maisconfig.acquisitionrate = as_service.settings.acquisitionrate;

    maisconfig.nameOfMais = themais_sensor.id;

    memset(&maisconfig.ethservice.configuration, 0, sizeof(maisconfig.ethservice.configuration));

    maisconfig.ethservice.configuration.type = eomn_serv_AS_mais;
    memcpy(&maisconfig.ethservice.configuration.data.as.mais.version.protocol, &themais_props.protocol, sizeof(eObrd_protocolversion_t));
    memcpy(&maisconfig.ethservice.configuration.data.as.mais.version.firmware, &themais_props.firmware, sizeof(eObrd_firmwareversion_t));

    // second check we do is about themais_sensor.location
    if(eobrd_place_can != themais_sensor.location.any.place)
    {
        yError() << "ServiceParser::parseService() has received an invalid location for its mais. it is not a CANx:adr location";
        return false;
    }
    maisconfig.ethservice.configuration.data.as.mais.canloc.port = themais_sensor.location.can.port;
    maisconfig.ethservice.configuration.data.as.mais.canloc.addr = themais_sensor.location.can.addr;
    maisconfig.ethservice.configuration.data.as.mais.canloc.insideindex = eobrd_caninsideindex_none;


    return true;
}


bool ServiceParser::parseService(Searchable &config, servConfigStrain_t &strainconfig)
{
    if(false == check_analog(config, eomn_serv_AS_strain))
    {
        yError() << "ServiceParser::parseService() has received an invalid SERVICE group for strain";
        return false;
    }

    // now we extract values ... so far we dont make many checks ... we just assume the vector<> are of size 1.
    servCanBoard_t thestrain_props = as_service.properties.canboards.at(0);
    servAnalogSensor_t thestrain_sensor = as_service.settings.enabledsensors.at(0);


    // first check we do is about thestrain_props.type
    if(eobrd_cantype_strain != thestrain_props.type)
    {
        yError() << "ServiceParser::parseService() has detected an invalid type of board. it should be a eobrd_strain but is a:" << eoboards_type2string(eoboards_cantype2type(thestrain_props.type));
        return false;
    }

    strainconfig.acquisitionrate = as_service.settings.acquisitionrate;
    strainconfig.useCalibration = as_strain_settings.useCalibration;
    strainconfig.nameOfStrain = thestrain_sensor.id;

    memset(&strainconfig.ethservice.configuration, 0, sizeof(strainconfig.ethservice.configuration));

    strainconfig.ethservice.configuration.type = eomn_serv_AS_strain;
    memcpy(&strainconfig.ethservice.configuration.data.as.strain.version.protocol, &thestrain_props.protocol, sizeof(eObrd_protocolversion_t));
    memcpy(&strainconfig.ethservice.configuration.data.as.strain.version.firmware, &thestrain_props.firmware, sizeof(eObrd_firmwareversion_t));

    // second check we do is about thestrain_sensor.location
    if(eobrd_place_can != thestrain_sensor.location.any.place)
    {
        yError() << "ServiceParser::parseService() has received an invalid location for strain. it is not a CANx:adr location";
        return false;
    }
    strainconfig.ethservice.configuration.data.as.strain.canloc.port = thestrain_sensor.location.can.port;
    strainconfig.ethservice.configuration.data.as.strain.canloc.addr = thestrain_sensor.location.can.addr;
    strainconfig.ethservice.configuration.data.as.strain.canloc.insideindex = eobrd_caninsideindex_none;



    return true;
}


bool ServiceParser::parseService(Searchable &config, servConfigInertials_t &inertialsconfig)
{
    if(false == check_analog(config, eomn_serv_AS_inertials))
    {
        yError() << "ServiceParser::parseService() has received an invalid SERVICE group for inertials";
        return false;
    }

    // now we extract values ... so far we dont make many checks ... we just assume the vector<> are of size 1.
    servCanBoard_t themtb_props = as_service.properties.canboards.at(0);

    // it must be an mtb
    if(eobrd_cantype_mtb != themtb_props.type)
    {
        yError() << "ServiceParser::parseService() has detected an invalid type of board. it should be a eobrd_mtb but is a:" << eoboards_type2string(eoboards_cantype2type(themtb_props.type));
        return false;
    }

    inertialsconfig.acquisitionrate = as_service.settings.acquisitionrate;

    memset(&inertialsconfig.ethservice.configuration, 0, sizeof(inertialsconfig.ethservice.configuration));

    inertialsconfig.ethservice.configuration.type = eomn_serv_AS_inertials;
    memcpy(&inertialsconfig.ethservice.configuration.data.as.inertial.mtbversion.protocol, &themtb_props.protocol, sizeof(eObrd_protocolversion_t));
    memcpy(&inertialsconfig.ethservice.configuration.data.as.inertial.mtbversion.firmware, &themtb_props.firmware, sizeof(eObrd_firmwareversion_t));

    // now, for all the sensors we must fill:
    // - inertialsconfig.ethservice.configuration.data.as.inertial.canmap[2] with mask of location of mtb boards... no, i dont do it
    // - the vector of ...


    inertialsconfig.inertials.resize(0);

    EOarray* array = eo_array_New(eOas_inertials_maxnumber, sizeof(eOas_inertial_descriptor_t), &inertialsconfig.ethservice.configuration.data.as.inertial.arrayofsensors);
    for(int i=0; i<as_service.settings.enabledsensors.size(); i++)
    {
        servAnalogSensor_t sensor = as_service.settings.enabledsensors.at(i);
        eOas_sensor_t type = sensor.type;

        if((eoas_accel_mtb_int != type) && (eoas_accel_mtb_ext != type) && (eoas_gyros_mtb_ext != type))
        {
            yWarning() << "ServiceParser::parseService() has detected a wrong inertial sensor:" << eoas_sensor2string(type) << " ...  we drop it";
            continue;
        }
        // if ok, i copy it inside ...

        eOas_inertial_descriptor_t des = {0};
        des.type = type;
        memcpy(&des.on, &sensor.location, sizeof(eObrd_location_t));

        eo_array_PushBack(array, &des);
        inertialsconfig.inertials.push_back(des);
        inertialsconfig.id.push_back(sensor.id);
    }


    return true;
}

// eof

