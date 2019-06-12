
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
using namespace std;


ServiceParser::ServiceParser()
{
    // how do i reset variable as_service?


    as_service.type = eomn_serv_NONE;

    as_service.properties.canboards.resize(0);
    as_service.properties.sensors.resize(0);

    as_service.settings.acquisitionrate = 0;
    as_service.settings.enabledsensors.resize(0);
}

bool ServiceParser::convert(std::string const &fromstring, eOmn_serv_type_t& toservicetype, bool& formaterror)
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


// bool ServiceParser::convert(std::string const &fromstring, eOmc_ctrlboard_t &controllerboard, bool &formaterror)
// {
//     const char *t = fromstring.c_str();
//     eObool_t usecompactstring = eobool_false;
//     controllerboard = eomc_string2controllerboard(t, usecompactstring);
//
//     if(eomc_ctrlboard_unknown == controllerboard)
//     {   // attempting to retrieve the compact form
//         usecompactstring = eobool_true;
//         controllerboard = eomc_string2controllerboard(t, usecompactstring);
//     }
//
//     if(eomc_ctrlboard_unknown == controllerboard)
//     {
//         yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmc_ctrlboard_t";
//         formaterror = true;
//         return false;
//     }
//
//     return true;
// }

bool ServiceParser::convert(std::string const &fromstring, eOas_sensor_t &tosensortype, bool &formaterror)
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


bool ServiceParser::convert(std::string const &fromstring, eObrd_type_t& tobrdtype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    tobrdtype = eoboards_string2type2(t, usecompactstring);

    if(eobrd_unknown == tobrdtype)
    {
        usecompactstring = eobool_true;
        tobrdtype = eoboards_string2type2(t, usecompactstring);
    }

    if(eobrd_unknown == tobrdtype)
    {
        yWarning() << "ServiceParser::convert(): string" << t << "cannot be converted into a proper eObrd_type_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(std::string const &fromstring, eOmc_pidoutputtype_t& pidoutputtype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    pidoutputtype = eomc_string2pidoutputtype(t, usecompactstring);

    if(eomc_pidoutputtype_unknown == pidoutputtype)
    {
        usecompactstring = eobool_true;
        pidoutputtype = eomc_string2pidoutputtype(t, usecompactstring);
    }

    if(eomc_pidoutputtype_unknown == pidoutputtype)
    {
        yWarning() << "ServiceParser::convert(): string" << t << "cannot be converted into a proper eOmc_pidoutputtype_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::convert(std::string const &fromstring, eOmc_jsetconstraint_t &jsetconstraint, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    jsetconstraint = eomc_string2jsetconstraint(t, usecompactstring);

    if(eomc_jsetconstraint_unknown == jsetconstraint)
    {
        usecompactstring = eobool_true;
        jsetconstraint = eomc_string2jsetconstraint(t, usecompactstring);
    }

    if(eomc_jsetconstraint_unknown == jsetconstraint)
    {
        yWarning() << "ServiceParser::convert(): string" << t << "cannot be converted into a proper eOmc_jsetconstraint_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::convert(std::string const &fromstring, eObrd_cantype_t& tobrdcantype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    eObrd_type_t type = eoboards_string2type2(t, usecompactstring);

    if(eobrd_unknown == type)
    {
        usecompactstring = eobool_true;
        type = eoboards_string2type2(t, usecompactstring);
    }

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

bool ServiceParser::convert(std::string const &fromstring, eObrd_ethtype_t& tobrdethtype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObool_t usecompactstring = eobool_false;
    eObrd_type_t type = eoboards_string2type2(t, usecompactstring);

    if(eobrd_unknown == type)
    {
        usecompactstring = eobool_true;
        type = eoboards_string2type2(t, usecompactstring);
    }

    if(eobrd_unknown == type)
    {
        yWarning() << "ServiceParser::convert(): string" << t << "cannot be converted into a proper eObrd_type_t";
        formaterror = true;
        return false;
    }

    tobrdethtype = eoboards_type2ethtype(type);
    if(eobrd_ethtype_unknown == tobrdethtype)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eObrd_ethtype_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(std::string const &fromstring, bool& tobool, bool& formaterror)
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



bool ServiceParser::convert(std::string const &fromstring, const uint8_t strsize, char *str, bool &formaterror)
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


bool ServiceParser::convert(std::string const &fromstring, string &str, bool &formaterror)
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



bool ServiceParser::convert(std::string const &fromstring, eObrd_location_t &location, bool &formaterror)
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
    else if(0 == strcmp(prefix, "LOC"))
    {
        //Add here parsing for local port (both PWM port (P7, P8, etc and "index_proximal"for hand))
        //in xml file we have: LOC:P7 or LOC:index_proximal
        
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
    if((eomn_serv_AS_mais != type) && (eomn_serv_AS_strain != type) && (eomn_serv_AS_inertials != type) && (eomn_serv_AS_inertials3 != type) && (eomn_serv_AS_psc != type))
    {
        yError() << "ServiceParser::check() is called with wrong type";
        return false;
    }

    // i parse the global board versions. if section is not found we can safely continue but we have boards.size() equal to zero.

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
            // now get type, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. the must be all equal.
            // for mais and strain and so far for intertials it must be numboards = 1.

            Bottle b_PROPERTIES_CANBOARDS_type = b_PROPERTIES_CANBOARDS.findGroup("type");
            if(b_PROPERTIES_CANBOARDS_type.isNull())
            {
                yError() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS.type";
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
            if( (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_major.size()) ||
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
                servCanBoard_t item;

                convert(b_PROPERTIES_CANBOARDS_type.get(i+1).asString(), item.type, formaterror);
                convert(b_PROPERTIES_CANBOARDS_PROTOCOL_major.get(i+1).asInt(), item.protocol.major, formaterror);
                convert(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.get(i+1).asInt(), item.protocol.minor, formaterror);

                convert(b_PROPERTIES_CANBOARDS_FIRMWARE_major.get(i+1).asInt(), item.firmware.major, formaterror);
                convert(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.get(i+1).asInt(), item.firmware.minor, formaterror);
                convert(b_PROPERTIES_CANBOARDS_FIRMWARE_build.get(i+1).asInt(), item.firmware.build, formaterror);

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
            Bottle b_PROPERTIES_SENSORS_boardtype;
            if(type == eomn_serv_AS_inertials3)
            {
                
                b_PROPERTIES_SENSORS_boardtype = Bottle(b_PROPERTIES_SENSORS.findGroup("boardType"));
                if(b_PROPERTIES_SENSORS_boardtype.isNull())
                {
                    yError() << "ServiceParser::check() cannot find PROPERTIES.SENSORS.boardType";
                    return false;
                }
            }
            else
            {
                b_PROPERTIES_SENSORS_boardtype.clear();
            }

            int tmp = b_PROPERTIES_SENSORS_id.size();
            int numsensors = tmp - 1;    // first position of bottle contains the tag "id"

            // check if all other fields have the same size.
            if( (tmp != b_PROPERTIES_SENSORS_type.size()) ||
                (tmp != b_PROPERTIES_SENSORS_location.size()) ||
                ((type == eomn_serv_AS_inertials3) && (b_PROPERTIES_SENSORS_boardtype.size() != tmp))
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
                if(type == eomn_serv_AS_inertials3)
                {
                    convert(b_PROPERTIES_SENSORS_boardtype.get(i+1).asString(), item.boardtype, formaterror);
                }
                else
                {
                    item.boardtype = eobrd_none;
                }

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

            std::string s_enabled_id = b_SETTINGS_enabledSensors.get(i+1).asString();
//            const char *str = s_enabled_id.c_str();
//            std::string cpp_str = str;

            // we must now search inside the whole vector<> as_service.properties.sensors if we find an id which matches s_enabled_id ....
            // if we dont, ... we issue a warning.
            // if we find, ... we do a pushback of it inside
            bool found = false;
            // i decide to use a brute force search ... for now
            for(size_t n=0; n<as_service.properties.sensors.size(); n++)
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

bool ServiceParser::check_skin(Searchable &config)
{
    const eOmn_serv_type_t type = eomn_serv_SK_skin;
    bool formaterror = false;

    // the format so far is:
    // SERVICE{ type, PROPERTIES{ CANBOARDS } }
    // later on we can add SERVICE.PROPERTIES.SENSORS and SERVICE.SETTINGS

    // however: if w dont have the SERVICE group we return false so that the caller can use default values


    Bottle b_SERVICE(config.findGroup("SERVICE"));
    if(b_SERVICE.isNull())
    {
        // yWarning() << "ServiceParser::check_skin() cannot find SERVICE group";
        return false;
    }

    // check whether we have the proper type

    if(false == b_SERVICE.check("type"))
    {
        // yWarning() << "ServiceParser::check_skin() cannot find SERVICE.type";
        return false;
    }
    else
    {
        Bottle b_type(b_SERVICE.find("type").asString());
        if(false == convert(b_type.toString(), sk_service.type, formaterror))
        {
            yWarning() << "ServiceParser::check_skin() has found unknown SERVICE.type = " << b_type.toString();
            return false;
        }
        if(type != sk_service.type)
        {
            yWarning() << "ServiceParser::check_skin() has found wrong SERVICE.type = " << sk_service.type << "it must be" << "TODO: tostring() function";
            return false;
        }
    }

    // check whether we have the proper groups

    Bottle b_PROPERTIES = Bottle(b_SERVICE.findGroup("PROPERTIES"));
    if(b_PROPERTIES.isNull())
    {
        yWarning() << "ServiceParser::check_skin() cannot find PROPERTIES";
        return false;
    }
    else
    {
        Bottle b_PROPERTIES_CANBOARDS = Bottle(b_PROPERTIES.findGroup("CANBOARDS"));
        if(b_PROPERTIES_CANBOARDS.isNull())
        {
            yWarning() << "ServiceParser::check() cannot find PROPERTIES.CANBOARDS";
            return false;
        }
        else
        {
            // now get type, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. the must be all equal.
            // for skin it must be numboards = 1.

            Bottle b_PROPERTIES_CANBOARDS_type = b_PROPERTIES_CANBOARDS.findGroup("type");
            if(b_PROPERTIES_CANBOARDS_type.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.type";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_PROTOCOL = Bottle(b_PROPERTIES_CANBOARDS.findGroup("PROTOCOL"));
            if(b_PROPERTIES_CANBOARDS_PROTOCOL.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.PROTOCOL";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_major = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("major"));
            if(b_PROPERTIES_CANBOARDS_PROTOCOL_major.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.PROTOCOL.major";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_minor = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("minor"));
            if(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.PROTOCOL.minor";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE = Bottle(b_PROPERTIES_CANBOARDS.findGroup("FIRMWARE"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.FIRMWARE";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_major = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("major"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE_major.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.FIRMWARE.major";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_minor = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("minor"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.FIRMWARE.minor";
                return false;
            }
            Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_build = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("build"));
            if(b_PROPERTIES_CANBOARDS_FIRMWARE_build.isNull())
            {
                yError() << "ServiceParser::check_skin() cannot find PROPERTIES.CANBOARDS.FIRMWARE.build";
                return false;
            }

            int tmp = b_PROPERTIES_CANBOARDS_type.size();
            int numboards = tmp - 1;    // first position of bottle contains the tag "type"

            // check if all other fields have the same size.
            if( (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_major.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_minor.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_major.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_minor.size()) ||
                (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_build.size())
              )
            {
                yError() << "ServiceParser::check_skin() in PROPERTIES.CANBOARDS some param has inconsistent length";
                return false;
            }

            if(1 != numboards)
            {
                yError() << "ServiceParser::check_skin() in PROPERTIES.CANBOARDS has found more than one board";
                return false;
            }

            formaterror = false;

            sk_service.properties.canboard.clear();

            convert(b_PROPERTIES_CANBOARDS_type.get(1).asString(), sk_service.properties.canboard.type, formaterror);
            convert(b_PROPERTIES_CANBOARDS_PROTOCOL_major.get(1).asInt(), sk_service.properties.canboard.protocol.major, formaterror);
            convert(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.get(1).asInt(), sk_service.properties.canboard.protocol.minor, formaterror);

            convert(b_PROPERTIES_CANBOARDS_FIRMWARE_major.get(1).asInt(), sk_service.properties.canboard.firmware.major, formaterror);
            convert(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.get(1).asInt(), sk_service.properties.canboard.firmware.minor, formaterror);
            convert(b_PROPERTIES_CANBOARDS_FIRMWARE_build.get(1).asInt(), sk_service.properties.canboard.firmware.build, formaterror);



            // in here we could decide to return false if any previous conversion function has returned error
            // bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

            if(true == formaterror)
            {
                yError() << "ServiceParser::check_skin() has detected an illegal format for some of the params of PROPERTIES.CANBOARDS some param has inconsistent length";
                return false;
            }
        }

#if 0
        // we dont have this group yet

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
            Bottle b_PROPERTIES_SENSORS_boardtype;
            if(type == eomn_serv_AS_inertials3)
            {

                b_PROPERTIES_SENSORS_boardtype = Bottle(b_PROPERTIES_SENSORS.findGroup("boardType"));
                if(b_PROPERTIES_SENSORS_boardtype.isNull())
                {
                    yError() << "ServiceParser::check() cannot find PROPERTIES.SENSORS.boardType";
                    return false;
                }
            }
            else
            {
                b_PROPERTIES_SENSORS_boardtype.clear();
            }

            int tmp = b_PROPERTIES_SENSORS_id.size();
            int numsensors = tmp - 1;    // first position of bottle contains the tag "id"

            // check if all other fields have the same size.
            if( (tmp != b_PROPERTIES_SENSORS_type.size()) ||
                (tmp != b_PROPERTIES_SENSORS_location.size()) ||
                ((type == eomn_serv_AS_inertials3) && (b_PROPERTIES_SENSORS_boardtype.size() != tmp))
              )
            {
                yError() << "ServiceParser::check() in PROPERTIES.SENSORS some param has inconsistent length";
                return false;
            }


            sk_service.properties.sensors.resize(0);

            formaterror = false;
            for(int i=0; i<numsensors; i++)
            {
                servAnalogSensor_t item;
                item.type = eoas_none;
                item.location.any.place = eobrd_place_none;

                convert(b_PROPERTIES_SENSORS_id.get(i+1).asString(), item.id, formaterror);
                convert(b_PROPERTIES_SENSORS_type.get(i+1).asString(), item.type, formaterror);
                convert(b_PROPERTIES_SENSORS_location.get(i+1).asString(), item.location, formaterror);
                if(type == eomn_serv_AS_inertials3)
                {
                    convert(b_PROPERTIES_SENSORS_boardtype.get(i+1).asString(), item.boardtype, formaterror);
                }
                else
                {
                    item.boardtype = eobrd_none;
                }

                sk_service.properties.sensors.push_back(item);
            }

            // in here we could decide to return false if any previous conversion function has returned error
            // bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

            if(true == formaterror)
            {
                yError() << "ServiceParser::check() has detected an illegal format for some of the params of PROPERTIES.SENSORS some param has inconsistent length";
                return false;
            }

        }

#endif

    }

#if 0

    // we dont have this group yet

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
        if( numenabledsensors > sk_service.properties.sensors.size() )
        {
            yError() << "ServiceParser::check() in SETTINGS.enabledSensors there are too many items with respect to supported sensors:" << numenabledsensors << "vs." << sk_service.properties.sensors.size();
            return false;
        }

        convert(b_SETTINGS_acquisitionRate.get(1).asInt(), sk_service.settings.acquisitionrate, formaterror);


        sk_service.settings.enabledsensors.resize(0);

        for(int i=0; i<numenabledsensors; i++)
        {
            servAnalogSensor_t founditem;

            std::string s_enabled_id = b_SETTINGS_enabledSensors.get(i+1).asString();
//            const char *str = s_enabled_id.c_str();
//            std::string cpp_str = str;

            // we must now search inside the whole vector<> sk_service.properties.sensors if we find an id which matches s_enabled_id ....
            // if we dont, ... we issue a warning.
            // if we find, ... we do a pushback of it inside
            bool found = false;
            // i decide to use a brute force search ... for now
            for(size_t n=0; n<sk_service.properties.sensors.size(); n++)
            {
                servAnalogSensor_t item = sk_service.properties.sensors.at(n);
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
                sk_service.settings.enabledsensors.push_back(founditem);
            }

        }

        // in here we issue an error if we dont have at least one enabled sensor

        if(0 == sk_service.settings.enabledsensors.size())
        {
            yError() << "ServiceParser::check() could not find any item in SETTINGS.enabledSensors which matches what in PROPERTIES.SENSORS.id";
            return false;
        }

    }

#endif


#if 0
    // we dont have this group yet
    // now we may have one or more sections which are specific of the device ...



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
            convert(b_STRAIN_SETTINGS_useCalibration.get(1).asString(), sk_skin_settings.useCalibration, formaterror);

            if(true == formaterror)
            {
                yError() << "ServiceParser::check() has detected an illegal format for paramf STRAIN_SETTINGS.useCalibration";
                return false;
            }
        }
    }


#endif


    // we we are in here we have the struct filled with all variables ... some validations are still due to the calling device.

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
        yError() << "ServiceParser::parseService() has detected an invalid type of board. it should be a eobrd_mais but is a:" << eoboards_type2string2(eoboards_cantype2type(themais_props.type), eobool_false);
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
    if((eobrd_cantype_strain != thestrain_props.type) && (eobrd_cantype_strain2 != thestrain_props.type))
    {
        yError() << "ServiceParser::parseService() has detected an invalid type of board. it should be a eobrd_strain or eobrd_strain2 but is a:" << eoboards_type2string2(eoboards_cantype2type(thestrain_props.type), eobool_false);
        return false;
    }

    strainconfig.acquisitionrate = as_service.settings.acquisitionrate;
    strainconfig.useCalibration = as_strain_settings.useCalibration;
    strainconfig.nameOfStrain = thestrain_sensor.id;

    memset(&strainconfig.ethservice.configuration, 0, sizeof(strainconfig.ethservice.configuration));

    strainconfig.ethservice.configuration.type = eomn_serv_AS_strain;
    strainconfig.ethservice.configuration.data.as.strain.boardtype.type = thestrain_props.type;
    memcpy(&strainconfig.ethservice.configuration.data.as.strain.boardtype.protocol, &thestrain_props.protocol, sizeof(eObrd_protocolversion_t));
    memcpy(&strainconfig.ethservice.configuration.data.as.strain.boardtype.firmware, &thestrain_props.firmware, sizeof(eObrd_firmwareversion_t));

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

bool ServiceParser::parseService(Searchable &config, servConfigFTsensor_t &ftconfig)
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
    if(eobrd_cantype_strain2 != thestrain_props.type)
    {
        yError() << "ServiceParser::parseService() for embObjFTsensor has detected an invalid type of board. it should be a eobrd_strain2 but is a:" << eoboards_type2string2(eoboards_cantype2type(thestrain_props.type), eobool_false);
        return false;
    }
    
    ftconfig.acquisitionrate = as_service.settings.acquisitionrate;
    ftconfig.useCalibration = as_strain_settings.useCalibration;
    ftconfig.nameOfStrain = thestrain_sensor.id;
    
    memset(&ftconfig.ethservice.configuration, 0, sizeof(ftconfig.ethservice.configuration));
    
    ftconfig.ethservice.configuration.type = eomn_serv_AS_strain;
    ftconfig.ethservice.configuration.data.as.strain.boardtype.type = thestrain_props.type;
    memcpy(&ftconfig.ethservice.configuration.data.as.strain.boardtype.protocol, &thestrain_props.protocol, sizeof(eObrd_protocolversion_t));
    memcpy(&ftconfig.ethservice.configuration.data.as.strain.boardtype.firmware, &thestrain_props.firmware, sizeof(eObrd_firmwareversion_t));
    
    // second check we do is about thestrain_sensor.location
    if(eobrd_place_can != thestrain_sensor.location.any.place)
    {
        yError() << "ServiceParser::parseService() has received an invalid location for strain. it is not a CANx:adr location";
        return false;
    }
    ftconfig.ethservice.configuration.data.as.strain.canloc.port = thestrain_sensor.location.can.port;
    ftconfig.ethservice.configuration.data.as.strain.canloc.addr = thestrain_sensor.location.can.addr;
    ftconfig.ethservice.configuration.data.as.strain.canloc.insideindex = eobrd_caninsideindex_none;
    
    
    
    Bottle b_SERVICE(config.findGroup("SERVICE")); //b_SERVICE and b_SETTINGS could not be null, otherwise parseService function would have returned false
    Bottle b_SETTINGS = Bottle(b_SERVICE.findGroup("SETTINGS"));
    Bottle b_SETTINGS_temp = Bottle(b_SETTINGS.findGroup("temperature-acquisitionRate"));
    if(b_SETTINGS_temp.isNull())
    {
        yError() << "ServiceParser::parseService() for embObjFTsensor device cannot find SETTINGS.temperature-acquisitionRate";
        return false;
    }
    else
    {
        ftconfig.temperatureAcquisitionrate = b_SETTINGS_temp.get(1).asInt();
        //TODO: chek that the acquisition rate is inside a reasonable range
    }
    
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
        yError() << "ServiceParser::parseService() has detected an invalid type of board. it should be a eobrd_mtb but is a:" << eoboards_type2string2(eoboards_cantype2type(themtb_props.type), eobool_false);
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
    for(size_t i=0; i<as_service.settings.enabledsensors.size(); i++)
    {
        servAnalogSensor_t sensor = as_service.settings.enabledsensors.at(i);
        eOas_sensor_t type = sensor.type;

        if((eoas_accel_mtb_int != type) && (eoas_accel_mtb_ext != type) && (eoas_gyros_mtb_ext != type) && (eoas_gyros_st_l3g4200d != type))
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


bool ServiceParser::parseService(Searchable &config, servConfigImu_t &imuconfig)
{
    if(false == check_analog(config, eomn_serv_AS_inertials3))
    {
        yError() << "ServiceParser::parseService(IMU) has received an invalid SERVICE group for IMU";
        return false;
    }
    
    
    //check the num of type of boards. At max we have 4 board type  (see eOas_inertials3_boardinfos_maxnumber)
    
    if(as_service.properties.canboards.size() > eOas_inertials3_boardinfos_maxnumber)
    {
        yError() << "ServiceParser::parseService(IMU): too many type board info are configured. The max num is " << eOas_inertials3_boardinfos_maxnumber;
        return false;
    }

    //reset configuration service
    memset(&imuconfig.ethservice.configuration, 0, sizeof(imuconfig.ethservice.configuration));
    
    //set type of service
    imuconfig.ethservice.configuration.type = eomn_serv_AS_inertials3;
    
    
    //get acquisition rate 
    imuconfig.acquisitionrate = as_service.settings.acquisitionrate;
    
    //get enabled sensor and fill canboard array. Note that we get only the enabled sensor, not all configured sensors !!!
    
    imuconfig.inertials.resize(0);
    
    eOas_inertial3_setof_boardinfos_t * boardInfoSet_ptr = &imuconfig.ethservice.configuration.data.as.inertial3.setofboardinfos;
    eOresult_t res = eoas_inertial3_setof_boardinfos_clear(boardInfoSet_ptr);
    if(res != eores_OK)
    {
        yError() << "ServiceParser::parseService(IMU). Error in eoas_inertial3_setof_boardinfos_clear()";
        return false;
    }
    
    EOarray* array = eo_array_New(eOas_inertials3_descriptors_maxnumber, sizeof(eOas_inertial3_descriptor_t), &imuconfig.ethservice.configuration.data.as.inertial3.arrayofdescriptor);
    for(size_t i=0; i<as_service.settings.enabledsensors.size(); i++)
    {
        servAnalogSensor_t sensor = as_service.settings.enabledsensors.at(i);
        eOas_sensor_t type = sensor.type;

        //TODO: temperature???
        if( (eoas_imu_acc != type) && (eoas_imu_mag != type) && (eoas_imu_gyr != type) && (eoas_imu_eul != type) && 
            (eoas_imu_qua != type) && (eoas_imu_lia != type) && (eoas_imu_grv != type) && (eoas_imu_status != type) )
        {
            yWarning() << "ServiceParser::parseService() has detected a wrong inertial sensor:" << eoas_sensor2string(type) << " ...  we drop it";
            continue;
        }
        // if ok, i copy it inside ...
        
        eOas_inertial3_descriptor_t des = {0};
        des.typeofsensor = type;
        memcpy(&des.on, &sensor.location, sizeof(eObrd_location_t));
        
        const eObrd_info_t *boardInfo_ptr =  eoas_inertial3_setof_boardinfos_find(boardInfoSet_ptr, eoboards_type2cantype(sensor.boardtype));
        if(nullptr == boardInfo_ptr)//if I did not already insert the borad info with type == sensor.boardtype, now I insert it
        {
            //first of all I need to find the board info for this board type
            int b;
            bool found=false;
            for(b=0; b<as_service.properties.canboards.size(); b++)
            {
                if(as_service.properties.canboards.at(b).type == eoboards_type2cantype(sensor.boardtype))
                {
                    found=true;
                    break;
                }
            }
            if(!found)
            {
                yError() << "ServiceParser::parseService(IMU). The sensor " << i << "with type "<<  eoas_sensor2string(static_cast<eOas_sensor_t> (des.typeofsensor)) << "has borad type  " << eoboards_type2string2(sensor.boardtype, false) << " that is not declared in the SERVICE.PROPERTIES.CANBOARDS tag";
                return false;
            }
            eObrd_info_t boardInfo = {0};
            boardInfo.type =  as_service.properties.canboards.at(b).type;
            memcpy(&boardInfo.protocol , &as_service.properties.canboards.at(b).protocol, sizeof(eObrd_protocolversion_t));
            memcpy(&boardInfo.firmware, &as_service.properties.canboards.at(b).firmware, sizeof(eObrd_firmwareversion_t));
            res = eoas_inertial3_setof_boardinfos_add(boardInfoSet_ptr, &boardInfo);
            if(eores_OK != res)
            {
                yError() << "ServiceParser::parseService(IMU). Error in eoas_inertial3_setof_boardinfos_add()";
                return false;
            }
        }
        des.typeofboard = sensor.boardtype;
        
        eo_array_PushBack(array, &des);
        imuconfig.inertials.push_back(des);
        imuconfig.id.push_back(sensor.id);
    }
    
    
    return true;
}


bool ServiceParser::parseService(Searchable &config, servConfigSkin_t &skinconfig)
{

    skinconfig.canboard.type = eobrd_cantype_mtb;
    skinconfig.canboard.firmware.major = 0;
    skinconfig.canboard.firmware.minor = 0;
    skinconfig.canboard.firmware.build = 0;
    skinconfig.canboard.protocol.major = 0;
    skinconfig.canboard.protocol.minor = 0;


    if(false == check_skin(config))
    {
        yWarning() << "ServiceParser::parseService(SKIN) has received an invalid SERVICE group: using defaults";
        return true;
    }


    //check the type of board. it must be mtb or mtb4

    if((eobrd_cantype_mtb != sk_service.properties.canboard.type) && (eobrd_cantype_mtb4 != sk_service.properties.canboard.type) && (eobrd_cantype_psc != sk_service.properties.canboard.type))
    {
        yError() << "ServiceParser::parseService(SK): only mtb / mtb4 / psc boards are allowed: using defaults";
        return false;
    }


    // fill canboard
    skinconfig.canboard.type = sk_service.properties.canboard.type;
    skinconfig.canboard.firmware.major = sk_service.properties.canboard.firmware.major;
    skinconfig.canboard.firmware.minor = sk_service.properties.canboard.firmware.minor;
    skinconfig.canboard.firmware.build = sk_service.properties.canboard.firmware.build;
    skinconfig.canboard.protocol.major = sk_service.properties.canboard.protocol.major;
    skinconfig.canboard.protocol.minor = sk_service.properties.canboard.protocol.minor;

    return true;
}


bool ServiceParser::parseService(Searchable &config, servConfigPSC_t &pscconfig)
{
    if(false == check_analog(config, eomn_serv_AS_psc))
    {
        yError() << "ServiceParser::parseService(PSC) has received an invalid SERVICE group for PSC";
        return false;
    }


    //check the num of type of boards. At max we have 1 board type

    if(as_service.properties.canboards.size() > 1)
    {
        yError() << "ServiceParser::parseService(PSC): too many type board info are configured. The max num is " << 1;
        return false;
    }

    if(as_service.settings.enabledsensors.size() > eOas_psc_boards_maxnumber)
    {
        yError() << "ServiceParser::parseService(PSC): too many enabled sensors are configured. The max num is " << eOas_psc_boards_maxnumber;
        return false;
    }

    //reset configuration service
    memset(&pscconfig.ethservice.configuration, 0, sizeof(pscconfig.ethservice.configuration));

    //set type of service
    pscconfig.ethservice.configuration.type = eomn_serv_AS_psc;


    //get acquisition rate
    pscconfig.acquisitionrate = as_service.settings.acquisitionrate;

    servCanBoard_t *asServBoardInfo_ptr = &as_service.properties.canboards[0];
    eOmn_serv_config_data_as_psc_t *pscBoardConfig_ptr = &pscconfig.ethservice.configuration.data.as.psc;

    //get firmware and protocol info
    pscBoardConfig_ptr->version.firmware.major = asServBoardInfo_ptr->firmware.major;
    pscBoardConfig_ptr->version.firmware.minor = asServBoardInfo_ptr->firmware.minor;
    pscBoardConfig_ptr->version.firmware.build = asServBoardInfo_ptr->firmware.build;
    pscBoardConfig_ptr->version.protocol.major = asServBoardInfo_ptr->protocol.major;
    pscBoardConfig_ptr->version.protocol.minor = asServBoardInfo_ptr->protocol.minor;

    for(size_t i=0; i<as_service.settings.enabledsensors.size(); i++)
    {
        servAnalogSensor_t sensor = as_service.settings.enabledsensors.at(i);

        if(eoas_psc_angle != sensor.type)
        {
            yWarning() << "ServiceParser::parseService() has detected a wrong psc sensor:" << eoas_sensor2string(sensor.type) << " ...  we drop it";
            continue;
        }

        // if ok, i copy it inside ...
        pscBoardConfig_ptr->boardInfo.canloc[i].addr= sensor.location.can.addr;
        pscBoardConfig_ptr->boardInfo.canloc[i].port= sensor.location.can.port;

    }

    return true;
}


#if defined(SERVICE_PARSER_USE_MC)



bool ServiceParser::parse_encoder_port(std::string const &fromstring, eObrd_ethtype_t const ethboard, eOmc_encoder_t type, uint8_t &toport, bool &formaterror)
{
    const char *t = fromstring.c_str();

    bool ret = false;
    switch(type)
    {
        case eomc_enc_unknown:
        {
            yWarning() << "ServiceParser::parse_encoder_port():" << t << "cannot be converted into a port because argument type is" << eomc_encoder2string(type, eobool_false);
            toport = eobrd_port_none;
            ret = false;
        } break;


        case eomc_enc_none:
        {
            toport = eobrd_port_none;
            ret = true;
        } break;

        case eomc_enc_aea:
        case eomc_enc_amo:
        case eomc_enc_qenc:
        case eomc_enc_spichainof2:
        case eomc_enc_absanalog:
        case eomc_enc_spichainof3:
        {
            uint8_t toport1 = eobrd_port_unknown;
            bool result = parse_port_conn(fromstring, ethboard, toport1, formaterror);

            if(false == result)
            {
                yWarning() << "ServiceParser::parse_encoder_port():" << t << "is not a legal string for an encoder connector port";
                formaterror = true;
                ret = false;
            }
            else
            {
                toport = toport1;
                ret = true;
            }

        } break;


        case eomc_enc_mais:
        {
            uint8_t toport1 = eobrd_port_unknown;
            bool result = parse_port_mais(fromstring, toport1, formaterror);

            if(false == result)
            {
                yWarning() << "ServiceParser::parse_encoder_port():" << t << "is not a legal string for an encoder connector port";
                formaterror = true;
                ret = false;
            }
            else
            {
                toport = toport1;
                ret = true;
            }

        } break;
        case eomc_enc_psc:
        {
            uint8_t toport1 = eobrd_port_unknown;
            bool result = parse_port_psc(fromstring, toport1, formaterror);

            if(false == result)
            {
                yWarning() << "ServiceParser::parse_encoder_port():" << t << "is not a legal string for an encoder connector port";
                formaterror = true;
                ret = false;
            }
            else
            {
                toport = toport1;
                ret = true;
            }

        } break;
        default:
        {
            int len = strlen(t);

            if(len > 15)
            {
                yWarning() << "SServiceParser::parse_encoder_port():" << t << "is not a legal string for a encoder port because it is too long with size =" << len;
                formaterror = true;
                return false;
            }
            char prefix[16] = {0};
            sscanf(t, "%3c", prefix);
            if(0 == strcmp(prefix, "CAN"))
            {
                toport = eobrd_port_nolocal;
                ret = true;
            }
            else
            {
                toport = eobrd_port_none;
                yWarning() << "ServiceParser::parse_encoder_port():" << t << "is not a legal string for a encoder port!";
                formaterror = true;
                return false;
            }
        }break;
    }
    return ret;
}


bool ServiceParser::parse_port_conn(std::string const &fromstring, eObrd_ethtype_t const ethboard, uint8_t &toport, bool &formaterror)
{
    const char *t = fromstring.c_str();
    bool ret = false;

    // format of string is CONN:P4 or CONN:eobrd_conn_P4 ... the parse function verifies presence of CONN:
    eObrd_connector_t conn = eobrd_conn_unknown;
    bool result = parse_connector(fromstring, conn, formaterror);

    if(false == result)
    {
        yWarning() << "ServiceParser::parse_port_conn():" << t << "is not a legal string for a eObrd_connector_t";
        formaterror = true;
        ret = false;
    }
    else
    {
        eObrd_port_t port = eoboards_connector2port(conn, eoboards_ethtype2type(ethboard));
        if(eobrd_port_unknown == port)
        {
            yWarning() << "ServiceParser::parse_port_conn():" << t << "does not convert to a legal port for connector" << eoboards_connector2string(conn, eobool_false) << "and parsed board";
            formaterror = true;
            ret = false;
        }
        else
        {
            toport = port;
            ret = true;
        }
    }

    return ret;
}



bool ServiceParser::parse_port_mais(std::string const &fromstring, uint8_t &toport, bool &formaterror)
{
    const char *t = fromstring.c_str();
    bool ret = false;

    // format of string is MAIS:eobrd_portmais_thumbproximal or MAIS:thumbproximal
    eObrd_portmais_t pmais = eobrd_portmais_unknown;
    bool result = parse_mais(fromstring, pmais, formaterror);

    if(false == result)
    {
        yWarning() << "ServiceParser::parse_port_mais():" << t << "is not a legal string for a port mais";
        formaterror = true;
        ret = false;
    }
    else
    {
        toport = pmais;
        ret = true;
    }

    return ret;
}


bool ServiceParser::parse_port_psc(std::string const &fromstring, uint8_t &toport, bool &formaterror)
{
    const char *t = fromstring.c_str();
    bool ret = false;

    // format of string is PSC:finger0 or finger0
    eObrd_portpsc_t ppsc = eobrd_portpsc_unknown;
    bool result = parse_psc(fromstring, ppsc, formaterror);

    if(false == result)
    {
        yWarning() << "ServiceParser::parse_port_psc():" << t << "is not a legal string for a port psc";
        formaterror = true;
        ret = false;
    }
    else
    {
        toport = ppsc;
        ret = true;
    }

    return ret;
}

//bool ServiceParser::parse_mais(std::string const &fromstring, eObrd_portmais_t &pmais, bool &formaterror)
//{
//    // parses MAIS:eobrd_portmais_thumbproximal or MAIS:thumbproximal

//    const char *tt = fromstring.c_str();
//    char prefix[16] = {0};
//    sscanf(tt, "%5c", prefix);

//    if(0 != strcmp(prefix, "MAIS:"))
//    {
//        yWarning() << "ServiceParser::parse_mais():" << tt << "is not a legal string because it must begin with MAIS:";
//        formaterror = true;
//        return false;
//    }

//    // ok, now i remove the first 5 characters "MAIS:" and parse the second section .... it can be extended or compact
//    const char *t = &tt[5];

//    eObool_t usecompactstring = eobool_false;
//    pmais = eoboards_string2portmais(t, usecompactstring);

//    if(eobrd_portmais_unknown == pmais)
//    {   // attempting to retrieve the compact form
//        usecompactstring = eobool_true;
//        pmais = eoboards_string2portmais(t, usecompactstring);
//    }

//    if(eobrd_portmais_unknown == pmais)
//    {
//        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eObrd_portmais_t";
//        formaterror = true;
//        return false;
//    }

//    return true;

//}


// we want to fill the des with relevant info:
// we may have CAN1:1:0 if we have a act_foc or an act_mc4, or a CONN:P4 if we have a pwm.
// hence, we need ... the string, the type of actuator, the ethboard (for transforming P4 into teh proper port value.
bool ServiceParser::parse_actuator_port(std::string const &fromstring, eObrd_ethtype_t const ethboard, eOmc_actuator_t const type, eOmc_actuator_descriptor_t &todes, bool &formaterror)
{
    const char *t = fromstring.c_str();

    bool ret = false;
    switch(type)
    {
        default:
        case eomc_act_unknown:
        {
            yWarning() << "ServiceParser::parse_actuator_port():" << t << "cannot be converted into a eObrd_location_t because argument type is" << eomc_actuator2string(type, eobool_false);
            todes.none.port = eobrd_port_none;
            ret = false;
        } break;


        case eomc_act_none:
        {
            //yWarning() << "ServiceParser::parse_actuator_port():" << t << "cannot be converted into a eObrd_location_t because argument type is" << eomc_actuator2string(type);
            todes.none.port = eobrd_port_none;
            ret = true;
        } break;

        case eomc_act_pwm:
        {

            uint8_t toport = eobrd_port_unknown;
            bool result = parse_port_conn(fromstring, ethboard, toport, formaterror);

            if(false == result)
            {
                yWarning() << "ServiceParser::parse_actuator_port():" << t << "is not a legal string for a pwm connector port";
                formaterror = true;
                ret = false;
            }
            else
            {
                todes.pwm.port = toport;
                ret = true;
            }

        } break;

        case eomc_act_foc:
        case eomc_act_mc4:
        {
            // read it as a CAN address
            eObrd_location_t loc;
            bool result = convert(fromstring, loc, formaterror);
            ret = true;
            if(false == result)
            {
                yWarning() << "ServiceParser::parse_actuator_port():" << t << "is not a legal string for a eObrd_location_t";
                formaterror = true;
                ret = false;
            }
            else if((eomc_act_mc4 == type) && (eobrd_place_extcan != loc.any.place))
            {
                yWarning() << "ServiceParser::parse_actuator_port():" << t << "is not a legal string for a eomc_act_mc4 location because it is not a eobrd_place_extcan";
                formaterror = true;
                ret = false;
            }
            else if(eomc_act_foc == type)
            {
                if((eobrd_place_can != loc.any.place) && (eobrd_place_extcan != loc.any.place))
                {
                    yWarning() << "ServiceParser::parse_actuator_port():" << t << "is not a legal string for a eomc_act_foc location because it is not a eobrd_place_extcan or eobrd_place_can";
                    formaterror = true;
                    ret = false;
                }
            }
            
            if(false == ret)
            {
                return ret;
            }
            
            if(eomc_act_foc == type)
            {
                // copy into todes.foc
                if(eobrd_place_can == loc.any.place)
                {
                    todes.foc.canloc.port = loc.can.port;
                    todes.foc.canloc.addr = loc.can.addr;
                    todes.foc.canloc.insideindex = eobrd_caninsideindex_first;

                    ret = true;
                }
                else
                {
                    todes.foc.canloc.port = loc.extcan.port;
                    todes.foc.canloc.addr = loc.extcan.addr;
                    todes.foc.canloc.insideindex = loc.extcan.index;
                    if(eobrd_caninsideindex_first != todes.foc.canloc.insideindex)
                    {
                        yWarning() << "ServiceParser::parse_actuator_port():" << "in eomc_act_foc the location has an index different from eobrd_caninsideindex_first. For now we force to it, but correct xml file.";
                        todes.foc.canloc.insideindex = eobrd_caninsideindex_first;
                        ret = true;
                    }
                    else
                    {
                        ret = true;
                    }
                }

            }
            else if(eomc_act_mc4 == type)
            {
                // copy into todes.mc4
                todes.mc4.canloc.port = loc.extcan.port;
                todes.mc4.canloc.addr = loc.extcan.addr;
                todes.mc4.canloc.insideindex = loc.extcan.index;

                if(eobrd_caninsideindex_none == todes.foc.canloc.insideindex)
                {
                    yWarning() << "ServiceParser::parse_actuator_port():" << "in eomc_act_mc4 the location has an eobrd_caninsideindex_none. Correct xml file.";
                    formaterror = true;
                    ret = false;
                }
                else
                {
                    ret = true;
                }
            }



        } break;

    }

    return ret;
}


bool ServiceParser::convert(std::string const &fromstring, eOmc_actuator_t &toactuatortype, bool &formaterror)
{
    const char *t = fromstring.c_str();
    eObool_t usecompactstring = eobool_false;
    toactuatortype = eomc_string2actuator(t, usecompactstring);

    if(eomc_act_unknown == toactuatortype)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        toactuatortype = eomc_string2actuator(t, usecompactstring);
    }

    if(eomc_act_unknown == toactuatortype)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmc_actuator_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(std::string const &fromstring, eOmc_position_t &toposition, bool &formaterror)
{
    const char *t = fromstring.c_str();
    eObool_t usecompactstring = eobool_false;
    toposition = eomc_string2position(t, usecompactstring);

    if(eomc_pos_unknown == toposition)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        toposition = eomc_string2position(t, usecompactstring);
    }

    if(eomc_pos_unknown == toposition)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmc_position_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::parse_connector(const std::string &fromstring, eObrd_connector_t &toconnector, bool &formaterror)
{
    // parses CONN:P4 or CONN:eobrd_conn_P4

    const char *tt = fromstring.c_str();
    char prefix[16] = {0};
    sscanf(tt, "%5c", prefix);

    if(0 != strcmp(prefix, "CONN:"))
    {
        yWarning() << "ServiceParser::convert():" << tt << "is not a legal string for eObrd_connector_t because it must begin with CONN:";
        formaterror = true;
        return false;
    }

    // ok, now i remove the first 5 characters "CONN:" and parse the second section .... it can be extended or compact
    const char *t = &tt[5];

    eObool_t usecompactstring = eobool_false;
    toconnector = eoboards_string2connector(t, usecompactstring);

    if(eobrd_conn_unknown == toconnector)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        toconnector = eoboards_string2connector(t, usecompactstring);
    }

    if(eobrd_conn_unknown == toconnector)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eObrd_connector_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::parse_mais(const std::string &fromstring, eObrd_portmais_t &toportmais, bool &formaterror)
{
    // parses MAIS:eobrd_portmais_thumbproximal or MAIS:thumbproximal

    const char *tt = fromstring.c_str();
    char prefix[16] = {0};
    sscanf(tt, "%5c", prefix);

    if(0 != strcmp(prefix, "MAIS:"))
    {
        yWarning() << "ServiceParser::parse_mais():" << tt << "is not a legal string for eObrd_portmais_t because it must begin with MAIS:";
        formaterror = true;
        return false;
    }

    // ok, now i remove the first 5 characters "MAIS:" and parse the second section .... it can be extended or compact
    const char *t = &tt[5];

    eObool_t usecompactstring = eobool_false;
    toportmais = eoboards_string2portmais(t, usecompactstring);

    if(eobrd_portmais_unknown == toportmais)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        toportmais = eoboards_string2portmais(t, usecompactstring);
    }

    if(eobrd_portmais_unknown == toportmais)
    {
        yWarning() << "ServiceParser::parse_mais():" << t << "is not a legal string for eObrd_portmais_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::parse_psc(const std::string &fromstring, eObrd_portpsc_t &toportpsc, bool &formaterror)
{
    // parses PSC:finger0 or finger0

    const char *tt = fromstring.c_str();
    char prefix[16] = {0};
    sscanf(tt, "%4c", prefix);

    const char *t=nullptr;

    if(0 != strcmp(prefix, "PSC:"))
        t = &tt[0]; //port of type finger0
    else
        t = &tt[4]; //port of type PSC:finger0

    eObool_t usecompactstring = eobool_false;
    toportpsc = eoboards_string2portpsc(t, usecompactstring);

    if(eobrd_portpsc_unknown == toportpsc)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        toportpsc = eoboards_string2portpsc(t, usecompactstring);
    }

    if(eobrd_portpsc_unknown == toportpsc)
    {
        yWarning() << "ServiceParser::parse_psc():" << t << "is not a legal string for eObrd_portpsc_t";
        formaterror = true;
        return false;
    }

    return true;
}

bool ServiceParser::convert(std::string const &fromstring, eOmc_encoder_t &toencodertype, bool &formaterror)
{
    const char *t = fromstring.c_str();
    eObool_t usecompactstring = eobool_false;
    toencodertype = eomc_string2encoder(t, usecompactstring);

    if(eomc_enc_unknown == toencodertype)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        toencodertype = eomc_string2encoder(t, usecompactstring);
    }

    if(eomc_enc_unknown == toencodertype)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmc_encoder_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::check_motion(Searchable &config)
{
    bool formaterror = false;
    
    // complete format is SERVICE{ type, PROPERTIES{ ETHBOARD, CANBOARDS, MC4, MAIS, CONTROLLER, JOINTMAPPING, JOINTSETS } }
    // so far, there is no SETTINGS{}.
    // then, some groups are not present for some kind of SERVICE.type
    // mc4:         ETHBOARD, CANBOARDS, MC4, MAIS
    // foc:         ETHBOARD, CANBOARDS, CONTROLLER, JOINTMAPPING, JOINTSETS
    // mc4plus:     ETHBOARD, CONTROLLER, JOINTMAPPING, JOINTSETS
    // mc4plusmais: ETHBOARD, CANBOARDS, MAIS, CONTROLLER, JOINTMAPPING, JOINTSETS



    const bool itisOKifwedontfindtheXMLgroup = true;

    Bottle b_SERVICE(config.findGroup("SERVICE"));
    if(b_SERVICE.isNull())
    {
        if(false == itisOKifwedontfindtheXMLgroup)
        {
            yError() << "ServiceParser::check_motion() cannot find SERVICE group";
            return false;
        }
        else
        {
            yWarning() << "ServiceParser::check_motion() cannot find SERVICE group, but we are in permissive mode, hence we ask the ETH board to config itself according to its IP address";
            mc_service.type = eomn_serv_MC_generic;
            return true;
        }
    }

    // check whether we have the proper type

    if(false == b_SERVICE.check("type"))
    {
        if(false == itisOKifwedontfindtheXMLgroup)
        {
            yError() << "ServiceParser::check_motion() cannot find SERVICE.type";
            return false;
        }
        else
        {
            yWarning() << "ServiceParser::check_motion() cannot find SERVICE.type, but we are in permissive mode, hence we ask the ETH board to config itself according to its IP address";
            mc_service.type = eomn_serv_MC_generic;
            return true;
        }
    }

    Bottle b_type(b_SERVICE.find("type").asString());
    if(false == convert(b_type.toString(), mc_service.type, formaterror))
    {
        yError() << "ServiceParser::check_motion() has found unknown SERVICE.type = " << b_type.toString();
        return false;
    }

    if(eomn_serv_MC_generic == mc_service.type)
    {
        yWarning() << "ServiceParser::check_motion() detects SERVICE.type = eomn_serv_MC_generic.. hence we ask the ETH board to config itself according to its IP address";
        return true;
    }

    // check whether we have the proper groups at first level.

    Bottle b_PROPERTIES = Bottle(b_SERVICE.findGroup("PROPERTIES"));
    if(b_PROPERTIES.isNull())
    {
        yError() << "ServiceParser::check_motion() cannot find PROPERTIES";
        return false;
    }


    // now, inside PROPERTIES there are groups which depend on mc_service.type.
    // i prefer to check them all in here rather to go on and check them one after another.
    Bottle b_PROPERTIES_ETHBOARD = Bottle(b_PROPERTIES.findGroup("ETHBOARD"));
    bool has_PROPERTIES_ETHBOARD = !b_PROPERTIES_ETHBOARD.isNull();

    Bottle b_PROPERTIES_MAIS = Bottle(b_PROPERTIES.findGroup("MAIS"));
    bool has_PROPERTIES_MAIS = !b_PROPERTIES_MAIS.isNull();

    Bottle b_PROPERTIES_PSC = Bottle(b_PROPERTIES.findGroup("PSC"));
    bool has_PROPERTIES_PSC = !b_PROPERTIES_PSC.isNull();

    Bottle b_PROPERTIES_MC4 = Bottle(b_PROPERTIES.findGroup("MC4"));
    bool has_PROPERTIES_MC4 = !b_PROPERTIES_MC4.isNull();

    Bottle b_PROPERTIES_CANBOARDS = Bottle(b_PROPERTIES.findGroup("CANBOARDS"));
    bool has_PROPERTIES_CANBOARDS = !b_PROPERTIES_CANBOARDS.isNull();

    Bottle b_PROPERTIES_JOINTMAPPING = Bottle(b_PROPERTIES.findGroup("JOINTMAPPING"));
    bool has_PROPERTIES_JOINTMAPPING = !b_PROPERTIES_JOINTMAPPING.isNull();



    bool itisoksofar = false;
    switch(mc_service.type)
    {
        case eomn_serv_MC_foc:
        {
            // must have: ETHBOARD, CANBOARDS, JOINTMAPPING
            itisoksofar = true;

            if(false == has_PROPERTIES_ETHBOARD)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.ETHBOARD for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_CANBOARDS)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_JOINTMAPPING)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

        } break;


        case eomn_serv_MC_mc4plus:
        {
            // must have: ETHBOARD, JOINTMAPPING

            itisoksofar = true;

            if(false == has_PROPERTIES_ETHBOARD)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.ETHBOARD for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_JOINTMAPPING)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

        } break;


        case eomn_serv_MC_mc4plusmais:
        {
            // must have: ETHBOARD, CANBOARDS, MAIS, JOINTMAPPING

            itisoksofar = true;

            if(false == has_PROPERTIES_ETHBOARD)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.ETHBOARD for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_CANBOARDS)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_MAIS)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MAIS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_JOINTMAPPING)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }


        } break;

        case eomn_serv_MC_mc4:
        {
            // must have: ETHBOARD, CANBOARDS, MC4, MAIS

            itisoksofar = true;

            if(false == has_PROPERTIES_ETHBOARD)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.ETHBOARD for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_CANBOARDS)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_MC4)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4 for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_MAIS)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MAIS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

        } break;

        case eomn_serv_MC_mc2pluspsc:
        {
            // must have: ETHBOARD, CANBOARDS, PSC, JOINTMAPPING

            itisoksofar = true;

            if(false == has_PROPERTIES_ETHBOARD)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.ETHBOARD for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_CANBOARDS)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_PSC)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MAIS for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }

            if(false == has_PROPERTIES_JOINTMAPPING)
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING for type" << eomn_servicetype2string(mc_service.type);
                itisoksofar = false;
            }


        } break;

        default:
        {
            yError() << "ServiceParser::check_motion() has found an unknown type" << eomn_servicetype2string(mc_service.type);
            itisoksofar = false;
        } break;
    }

    if(false == itisoksofar)
    {
        yError() << "ServiceParser::check_motion() detected missing groups in PROPERTIES";
        return false;
    }

    // now i parse the groups

    if(true == has_PROPERTIES_ETHBOARD)
    {
        // i get type

        Bottle b_PROPERTIES_ETHBOARD_type = b_PROPERTIES_ETHBOARD.findGroup("type");
        if(b_PROPERTIES_ETHBOARD_type.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.ETHBOARD.type";
            return false;
        }

        // we must have only one board
        int numofethboards = b_PROPERTIES_ETHBOARD_type.size() - 1;
        if(1 != numofethboards)
        {
            yError() << "ServiceParser::check_motion() PROPERTIES.ETHBOARD.type must have one board only";
            return false;
        }
        if(false == convert(b_PROPERTIES_ETHBOARD_type.get(1).asString(), mc_service.properties.ethboardtype, formaterror))
        {
            yError() << "ServiceParser::check_motion() has found unknown SERVICE.PROPERTIES.ETHBOARD.type = " << b_PROPERTIES_ETHBOARD_type.get(1).asString();
            return false;
        }

    } // has_PROPERTIES_ETHBOARD


    if(true == has_PROPERTIES_CANBOARDS)
    {
        // i get type, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. they must be all equal. i can have more than one board

        Bottle b_PROPERTIES_CANBOARDS_type = b_PROPERTIES_CANBOARDS.findGroup("type");
        if(b_PROPERTIES_CANBOARDS_type.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.type";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_PROTOCOL = Bottle(b_PROPERTIES_CANBOARDS.findGroup("PROTOCOL"));
        if(b_PROPERTIES_CANBOARDS_PROTOCOL.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.PROTOCOL";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_major = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("major"));
        if(b_PROPERTIES_CANBOARDS_PROTOCOL_major.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.PROTOCOL.major";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_PROTOCOL_minor = Bottle(b_PROPERTIES_CANBOARDS_PROTOCOL.findGroup("minor"));
        if(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.PROTOCOL.minor";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_FIRMWARE = Bottle(b_PROPERTIES_CANBOARDS.findGroup("FIRMWARE"));
        if(b_PROPERTIES_CANBOARDS_FIRMWARE.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.FIRMWARE";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_major = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("major"));
        if(b_PROPERTIES_CANBOARDS_FIRMWARE_major.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.FIRMWARE.major";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_minor = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("minor"));
        if(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.FIRMWARE.minor";
            return false;
        }
        Bottle b_PROPERTIES_CANBOARDS_FIRMWARE_build = Bottle(b_PROPERTIES_CANBOARDS_FIRMWARE.findGroup("build"));
        if(b_PROPERTIES_CANBOARDS_FIRMWARE_build.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CANBOARDS.FIRMWARE.build";
            return false;
        }

        int tmp = b_PROPERTIES_CANBOARDS_type.size();
        int numboards = tmp - 1;    // first position of bottle contains the tag "type"

        // check if all other fields have the same size.
        if( (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_major.size()) ||
            (tmp != b_PROPERTIES_CANBOARDS_PROTOCOL_minor.size()) ||
            (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_major.size()) ||
            (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_minor.size()) ||
            (tmp != b_PROPERTIES_CANBOARDS_FIRMWARE_build.size())
            )
        {
            yError() << "ServiceParser::check_motion() in PROPERTIES.CANBOARDS some params have inconsistent lengths";
            return false;
        }


        mc_service.properties.canboards.resize(0);

        formaterror = false;
        for(int i=0; i<numboards; i++)
        {
            servCanBoard_t item;

            convert(b_PROPERTIES_CANBOARDS_type.get(i+1).asString(), item.type, formaterror);
            convert(b_PROPERTIES_CANBOARDS_PROTOCOL_major.get(i+1).asInt(), item.protocol.major, formaterror);
            convert(b_PROPERTIES_CANBOARDS_PROTOCOL_minor.get(i+1).asInt(), item.protocol.minor, formaterror);

            convert(b_PROPERTIES_CANBOARDS_FIRMWARE_major.get(i+1).asInt(), item.firmware.major, formaterror);
            convert(b_PROPERTIES_CANBOARDS_FIRMWARE_minor.get(i+1).asInt(), item.firmware.minor, formaterror);
            convert(b_PROPERTIES_CANBOARDS_FIRMWARE_build.get(i+1).asInt(), item.firmware.build, formaterror);

            mc_service.properties.canboards.push_back(item);
        }

        // in here we could decide to return false if any previous conversion function has returned error
        // bool fromStringToBoolean(string str, bool &anyerror); // inside: if error then .... be sure to set error = true. dont set it to false.

        if(true == formaterror)
        {
            yError() << "ServiceParser::check_motion() has detected an illegal format for some of the params of PROPERTIES.CANBOARDS some param has inconsistent length";
            return false;
        }

    } // has_PROPERTIES_CANBOARDS


    if(true == has_PROPERTIES_MAIS)
    {
        // i get .location and nothing else

        Bottle b_PROPERTIES_MAIS_location = b_PROPERTIES_MAIS.findGroup("location");
        if(b_PROPERTIES_MAIS_location.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MAIS.location";
            return false;
        }

        int tmp = b_PROPERTIES_MAIS_location.size();
        int numboards = tmp - 1;    // first position of bottle contains the tag "location"

        // check if numboards is 1.
        if(1 != numboards)
        {
            yError() << "ServiceParser::check_motion() in PROPERTIES.MAIS.location must contain one item only and it has:" << numboards;
            return false;
        }

        formaterror = false;
        eObrd_location_t loc;
        if(false == convert(b_PROPERTIES_MAIS_location.get(1).asString(), loc, formaterror))
        {
            yError() << "ServiceParser::check_motion() has detected an illegal format for SERVICE.PROPERTIES.MAIS.location";
            return false;
        }

        if(eobrd_place_can == loc.any.place)
        {
            mc_service.properties.maislocation.port = loc.can.port;
            mc_service.properties.maislocation.addr = loc.can.addr;
            mc_service.properties.maislocation.insideindex = eobrd_caninsideindex_none;
        }
        else if(eobrd_place_extcan == loc.any.place)
        {
            mc_service.properties.maislocation.port = loc.extcan.port;
            mc_service.properties.maislocation.addr = loc.extcan.addr;
            mc_service.properties.maislocation.insideindex = eobrd_caninsideindex_none;
        }
        else
        {
            yError() << "ServiceParser::check_motion() has detected an illegal format for SERVICE.PROPERTIES.MAIS.location. it must be either can or extcan";
            return false;
        }

    } // has_PROPERTIES_MAIS


    if(true == has_PROPERTIES_MC4)
    {
        // i get:
        // SHIFTS.velocity/estimJointVelocity/estimJointAcceleration/estimMotorVelocity/estimMotorAcceleration,
        // BROADCASTPOLICY, JOINT2BOARD)

        Bottle b_PROPERTIES_MC4_SHIFTS = Bottle(b_PROPERTIES_MC4.findGroup("SHIFTS"));
        if(b_PROPERTIES_MC4_SHIFTS.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.SHIFTS";
            return false;
        }

        Bottle b_PROPERTIES_MC4_SHIFTS_velocity = Bottle(b_PROPERTIES_MC4_SHIFTS.findGroup("velocity"));
        if(b_PROPERTIES_MC4_SHIFTS_velocity.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.SHIFTS.velocity";
            return false;
        }
        Bottle b_PROPERTIES_MC4_SHIFTS_estimJointVelocity = Bottle(b_PROPERTIES_MC4_SHIFTS.findGroup("estimJointVelocity"));
        if(b_PROPERTIES_MC4_SHIFTS_estimJointVelocity.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.SHIFTS.estimJointVelocity";
            return false;
        }
        Bottle b_PROPERTIES_MC4_SHIFTS_estimJointAcceleration = Bottle(b_PROPERTIES_MC4_SHIFTS.findGroup("estimJointAcceleration"));
        if(b_PROPERTIES_MC4_SHIFTS_estimJointAcceleration.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.SHIFTS.estimJointAcceleration";
            return false;
        }
        Bottle b_PROPERTIES_MC4_SHIFTS_estimMotorVelocity = Bottle(b_PROPERTIES_MC4_SHIFTS.findGroup("estimMotorVelocity"));
        if(b_PROPERTIES_MC4_SHIFTS_estimMotorVelocity.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.SHIFTS.estimMotorVelocity";
            return false;
        }
        Bottle b_PROPERTIES_MC4_SHIFTS_estimMotorAcceleration = Bottle(b_PROPERTIES_MC4_SHIFTS.findGroup("estimMotorAcceleration"));
        if(b_PROPERTIES_MC4_SHIFTS_estimMotorAcceleration.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.SHIFTS.estimMotorAcceleration";
            return false;
        }

        Bottle b_PROPERTIES_MC4_BROADCASTPOLICY = Bottle(b_PROPERTIES_MC4.findGroup("BROADCASTPOLICY"));
        if(b_PROPERTIES_MC4_BROADCASTPOLICY.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.BROADCASTPOLICY";
            return false;
        }

        Bottle b_PROPERTIES_MC4_BROADCASTPOLICY_enable = Bottle(b_PROPERTIES_MC4_BROADCASTPOLICY.findGroup("enable"));
        if(b_PROPERTIES_MC4_BROADCASTPOLICY_enable.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.BROADCASTPOLICY.enable";
            return false;
        }

/*        Bottle b_PROPERTIES_MC4_JOINT2BOARD = Bottle(b_PROPERTIES_MC4.findGroup("JOINT2BOARD"));
        if(b_PROPERTIES_MC4_JOINT2BOARD.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.JOINT2BOARD";
            return false;
        }

        Bottle b_PROPERTIES_MC4_JOINT2BOARD_location = Bottle(b_PROPERTIES_MC4_JOINT2BOARD.findGroup("location"));
        if(b_PROPERTIES_MC4_JOINT2BOARD_location.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.MC4.JOINT2BOARD.location";
            return false;
        }
*/
        // 1. get values for SHIFTS

        uint8_t value = 0;
        if(true == convert(b_PROPERTIES_MC4_SHIFTS_velocity.get(1).asInt(), value, formaterror))
        {
            mc_service.properties.mc4shifts.velocity = value;
        }
        if(true == convert(b_PROPERTIES_MC4_SHIFTS_estimJointVelocity.get(1).asInt(), value, formaterror))
        {
            if(value > 15)
            {
                yError() << "ServiceParser::check_motion() manages values of PROPERTIES.MC4.SHIFTS.estim* only in range [0, 15]. read value =" << value;
                return false;
            }
            mc_service.properties.mc4shifts.estimJointVelocity = value;
        }
        if(true == convert(b_PROPERTIES_MC4_SHIFTS_estimJointAcceleration.get(1).asInt(), value, formaterror))
        {
            if(value > 15)
            {
                yError() << "ServiceParser::check_motion() manages values of PROPERTIES.MC4.SHIFTS.estim* only in range [0, 15]. read value =" << value;
                return false;
            }
            mc_service.properties.mc4shifts.estimJointAcceleration = value;
        }
        if(true == convert(b_PROPERTIES_MC4_SHIFTS_estimMotorVelocity.get(1).asInt(), value, formaterror))
        {
            if(value > 15)
            {
                yError() << "ServiceParser::check_motion() manages values of PROPERTIES.MC4.SHIFTS.estim* only in range [0, 15]. read value =" << value;
                return false;
            }
            mc_service.properties.mc4shifts.estimMotorVelocity = value;
        }
        if(true == convert(b_PROPERTIES_MC4_SHIFTS_estimMotorAcceleration.get(1).asInt(), value, formaterror))
        {
            if(value > 15)
            {
                yError() << "ServiceParser::check_motion() manages values of PROPERTIES.MC4.SHIFTS.estim* only in range [0, 15]. read value =" << value;
                return false;
            }
            mc_service.properties.mc4shifts.estimMotorAcceleration = value;
        }

        // 2. i get the values for b_PROPERTIES_MC4_BROADCASTPOLICY_enable

        int tmp = b_PROPERTIES_MC4_BROADCASTPOLICY_enable.size();
        int numofenables = tmp - 1;    // first position of bottle contains the tag "enable"

        mc_service.properties.mc4broadcasts.resize(0);
        for(int i=0; i<numofenables; i++)
        {
            // transform the string into a value.
            eOmc_mc4broadcast_t item = eomc_mc4broadcast_unknown;
            const char *str = b_PROPERTIES_MC4_BROADCASTPOLICY_enable.get(i+1).asString().c_str();

            // check compact and then non-compact form
            if(eomc_mc4broadcast_unknown == (item = eomc_string2mc4broadcast(str, eobool_true)))
            {
                item = eomc_string2mc4broadcast(str, eobool_false);
            }

            if((eomc_mc4broadcast_unknown != item) && (eomc_mc4broadcast_none != item))
            {   // if meaningful ...
                mc_service.properties.mc4broadcasts.push_back(item);
            }
        }



        // 3. i get the values for b_PROPERTIES_MC4_JOINT2BOARD_location.... there must be 12 values

/*        int tmp1 = b_PROPERTIES_MC4_JOINT2BOARD_location.size();
        int numoflocations = tmp1 - 1;    // first position of bottle contains the tag "location"

        if(12 != numoflocations)
        {
            yError() << "ServiceParser::check_motion() detects that PROPERTIES.MC4.JOINT2BOARD.location has not 12 items but" << numoflocations;
            return false;
        }

        mc_service.properties.mc4joints.resize(0);
        for(int i=0; i<numoflocations; i++)
        {
            // transform the string into a location

            formaterror = false;
            eObrd_location_t loc;
            if(false == convert(b_PROPERTIES_MC4_JOINT2BOARD_location.get(i+1).asString(), loc, formaterror))
            {
                yError() << "ServiceParser::check_motion() has detected an illegal format for PROPERTIES.MC4.JOINT2BOARD.location";
                return false;
            }

            eObrd_canlocation_t item;
            if(eobrd_place_extcan == loc.any.place)
            {
                item.port = loc.extcan.port;
                item.addr = loc.extcan.addr;
                item.insideindex = loc.extcan.index;
            }
            else
            {
                yError() << "ServiceParser::check_motion() has detected an illegal format for PROPERTIES.MC4.JOINT2BOARD.location. it must be extcan";
                return false;
            }

            mc_service.properties.mc4joints.push_back(item);
        }
*/

    } // has_PROPERTIES_MC4

    if(true == has_PROPERTIES_PSC)
    {
        // i get .location and nothing else

        Bottle b_PROPERTIES_PSC_location = b_PROPERTIES_PSC.findGroup("location");
        if(b_PROPERTIES_PSC_location.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.PSC.location";
            return false;
        }

        int tmp = b_PROPERTIES_PSC_location.size();
        int numboards = tmp - 1;    // first position of bottle contains the tag "location"

        // check if numboards is 3.
        if(eOas_psc_boards_maxnumber != numboards)
        {
            yError() << "ServiceParser::check_motion() in PROPERTIES.PSC.location must contain three items and it has:" << numboards;
            return false;
        }

        mc_service.properties.psclocations.resize(3);
        for(int i=0; i<3; i++)
        {
            eObrd_location_t loc;
            formaterror = false;
            if(false == convert(b_PROPERTIES_PSC_location.get(1+i).asString(), loc, formaterror))
            {
                yError() << "ServiceParser::check_motion() has detected an illegal format for SERVICE.PROPERTIES.PSC.location";
                return false;
            }

            if(eobrd_place_can == loc.any.place)
            {
                mc_service.properties.psclocations[i].port = loc.can.port;
                mc_service.properties.psclocations[i].addr = loc.can.addr;
                mc_service.properties.psclocations[i].insideindex = eobrd_caninsideindex_none;

            }
            else if(eobrd_place_extcan == loc.any.place)
            {
                mc_service.properties.psclocations[i].port = loc.extcan.port;
                mc_service.properties.psclocations[i].addr = loc.extcan.addr;
                mc_service.properties.psclocations[i].insideindex = eobrd_caninsideindex_none;
            }
            else
            {
                yError() << "ServiceParser::check_motion() has detected an illegal format for SERVICE.PROPERTIES.PSC.location. it must be either can or extcan";
                return false;
            }
        }

    } // has_PROPERTIES_PSC


    if(true == has_PROPERTIES_JOINTMAPPING)
    {

        // actuator, encoder1, encoder2 must all be present. the params contained inside must be all of equal length and equal to numberofjoints

        Bottle b_PROPERTIES_JOINTMAPPING_ACTUATOR = Bottle(b_PROPERTIES_JOINTMAPPING.findGroup("ACTUATOR"));
        if(b_PROPERTIES_JOINTMAPPING_ACTUATOR.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ACTUATOR";
            return false;
        }

        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1 = Bottle(b_PROPERTIES_JOINTMAPPING.findGroup("ENCODER1"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER1.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1";
            return false;
        }

        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2 = Bottle(b_PROPERTIES_JOINTMAPPING.findGroup("ENCODER2"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER2.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2";
            return false;
        }

        // now the vectors the above three contain ...

        Bottle b_PROPERTIES_JOINTMAPPING_ACTUATOR_type = Bottle(b_PROPERTIES_JOINTMAPPING_ACTUATOR.findGroup("type"));
        if(b_PROPERTIES_JOINTMAPPING_ACTUATOR_type.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ACTUATOR.type";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ACTUATOR_port = Bottle(b_PROPERTIES_JOINTMAPPING_ACTUATOR.findGroup("port"));
        if(b_PROPERTIES_JOINTMAPPING_ACTUATOR_port.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ACTUATOR.port";
            return false;
        }

        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_type = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER1.findGroup("type"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER1_type.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.type";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_port = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER1.findGroup("port"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER1_port.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.port";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_position = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER1.findGroup("position"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER1_position.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.position";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_resolution = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER1.findGroup("resolution"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER1_resolution.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.resolution";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_tolerance = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER1.findGroup("tolerance"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER1_tolerance.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.tolerance";
            return false;
        }



        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_type = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER2.findGroup("type"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER2_type.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.type";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_port = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER2.findGroup("port"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER2_port.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.port";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_position = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER2.findGroup("position"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER2_position.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.position";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_resolution = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER2.findGroup("resolution"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER2_resolution.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.resolution";
            return false;
        }
        Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_tolerance = Bottle(b_PROPERTIES_JOINTMAPPING_ENCODER2.findGroup("tolerance"));
        if(b_PROPERTIES_JOINTMAPPING_ENCODER2_tolerance.isNull())
        {
            yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.tolerance";
            return false;
        }
        // now the size of the vectors must all be equal

        int tmp = b_PROPERTIES_JOINTMAPPING_ACTUATOR_type.size();
        if( (tmp != b_PROPERTIES_JOINTMAPPING_ACTUATOR_port.size())      ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_type.size())      ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_port.size())      ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_position.size())  ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_resolution.size())||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_tolerance.size()) ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_type.size())      ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_port.size())      ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_position.size())  ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_resolution.size()) ||
            (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_tolerance.size())
            )
        {
            yError() << "ServiceParser::check_motion() detected wrong number of columns somewhere inside PROPERTIES.JOINTMAPPING";
            return false;
        }

        // ok, we have the number of joints .........
        mc_service.properties.numofjoints = tmp-1;

        // i attempt to parse the vectors inside actuators (type, port), encoder1 (type, port, position) and encoder2 (type, port, position).

        mc_service.properties.actuators.resize(0);
        mc_service.properties.encoder1s.resize(0);
        mc_service.properties.encoder2s.resize(0);

        if(eomn_serv_MC_mc4 == mc_service.type)
        {
            mc_service.properties.mc4joints.resize(0);
        }


        for(int i=0; i<mc_service.properties.numofjoints; i++)
        {
            servMC_actuator_t act;
            servMC_encoder_t enc1;
            servMC_encoder_t enc2;
            bool formaterror = false;

            eOmc_encoder_t enctype = eomc_enc_unknown;
            uint8_t encport = eobrd_port_unknown;
            eOmc_position_t encposition = eomc_pos_unknown;

            // actuators ..
            act.type = eomc_act_unknown;
            act.desc.pwm.port = eobrd_port_unknown;

            if(false == convert(b_PROPERTIES_JOINTMAPPING_ACTUATOR_type.get(i+1).asString(), act.type, formaterror))
            {
                yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.actuator.type not valid for item" << i;
                return false;
            }

            if((eomn_serv_MC_mc4 == mc_service.type) && (eomc_act_mc4 != act.type))
            {
                yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.actuator.type should be mc4 with mc service of type eomn_serv_MC_mc4. Error in item" << i;
                return false;
            }

            if(false == parse_actuator_port(b_PROPERTIES_JOINTMAPPING_ACTUATOR_port.get(i+1).asString(), mc_service.properties.ethboardtype, act.type, act.desc, formaterror))
            {
                    yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.actuator.port not valid for item" << i;
                return false;
            }

            if(eomn_serv_MC_mc4 == mc_service.type)
            {
                eObrd_canlocation_t item;
                item.port = act.desc.mc4.canloc.port;
                item.addr = act.desc.mc4.canloc.addr;
                item.insideindex = act.desc.mc4.canloc.insideindex;

                mc_service.properties.mc4joints.push_back(item);
            }


            // encoder1s ...

            enc1.desc.type = eomc_enc_unknown;
            enc1.desc.port = eobrd_port_unknown;
            enc1.desc.pos = eomc_pos_unknown;

            enctype = eomc_enc_unknown;
            if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER1_type.get(i+1).asString(), enctype, formaterror))
            {
                yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.encoder1.type not valid for item" << i;
                return false;
            }
            enc1.desc.type = enctype;

            // depending on type, we ....



            // dobbiamo fare un parser che riconosca CONN:P6 se aea, qenc etc, oppure MAIS:thumbproximal se mais

            encport = eobrd_port_unknown;
            if(false == parse_encoder_port(b_PROPERTIES_JOINTMAPPING_ENCODER1_port.get(i+1).asString(), mc_service.properties.ethboardtype, enctype, encport, formaterror))
            {
                yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.encoder1.port not valid for item" << i;
                return false;
            }
            enc1.desc.port = encport;

            encposition = eomc_pos_unknown;
            if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER1_position.get(i+1).asString(), encposition, formaterror))
            {
                    yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.encoder1.position not valid for item" << i;
                return false;
            }
            enc1.desc.pos = encposition;

            enc1.resolution = b_PROPERTIES_JOINTMAPPING_ENCODER1_resolution.get(i+1).asInt();

            enc1.tolerance = b_PROPERTIES_JOINTMAPPING_ENCODER1_tolerance.get(i+1).asDouble();



            // encoder2s ...

            enc2.desc.type = eomc_enc_unknown;
            enc2.desc.port = eobrd_port_unknown;
            enc2.desc.pos = eomc_pos_unknown;

            enctype = eomc_enc_unknown;
            if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER2_type.get(i+1).asString(), enctype, formaterror))
            {
                yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.encoder2.type not valid for item" << i;
                return false;
            }
//             //VALE: workaround to solve problem of configuration of encoder connected to 2foc board.
//             if((eomn_serv_MC_foc == mc_service.type) &&  (eomc_enc_roie == enctype))
//             {
//                 enctype = eomc_enc_none;
//             }


            enc2.desc.type = enctype;


            encport = eobrd_port_unknown;
            if(false == parse_encoder_port(b_PROPERTIES_JOINTMAPPING_ENCODER2_port.get(i+1).asString(), mc_service.properties.ethboardtype, enctype, encport, formaterror))
            {
                yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.encoder2.port not valid for item" << i;
                return false;
            }
            enc2.desc.port = encport;

            encposition = eomc_pos_unknown;
            if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER2_position.get(i+1).asString(), encposition, formaterror))
            {
                    yError() << "ServiceParser::check_motion() PROPERTIES.JOINTMAPPING.encoder2.position not valid for item" << i;
                return false;
            }
//              //VALE: workaround to solve problem of configuration of encoder connected to 2foc board.
//             if((eomn_serv_MC_foc == mc_service.type) &&  (eomc_enc_roie == enctype))
//             {
//                  enc2.desc.pos = eomc_pos_none;
//             }
//             else
//             {
//                  enc2.desc.pos = encposition;
//             }

            enc2.desc.pos = encposition;

            enc2.resolution = b_PROPERTIES_JOINTMAPPING_ENCODER2_resolution.get(i+1).asInt();
            enc2.tolerance = b_PROPERTIES_JOINTMAPPING_ENCODER2_tolerance.get(i+1).asDouble();


            // ok, we push act, enc1, enc2

            mc_service.properties.actuators.push_back(act);
            mc_service.properties.encoder1s.push_back(enc1);
            mc_service.properties.encoder2s.push_back(enc2);

        }

    } // has_PROPERTIES_JOINTMAPPING

    return true;
}

// int ServiceParser::getnumofjointsets(void)
// {
//
//     int n=0;
//     int njointsinset[mc_service.properties.numofjoints]; //the max number of sets is equal to max number of joints.
//     for(int i=0; i<mc_service.properties.numofjoints;i++)
//     {
//         njointsinset[i] = 0;
//     }
//
//     for(int i=0; i<mc_service.properties.numofjoints;i++)
//     {
//         int set = mc_service.properties.joint2set[i];
//
//         if(set > mc_service.properties.numofjoints)
//         {
//             yError() << "ServiceParser::getnumofjointsets: error in joint2set param. The set number is too big. Remember: max number of set is equal to max num of joint.";
//             return false;
//         }
//
//        njointsinset[set]++;
//     }
//     int count =0;
//     for(int i=0; i<mc_service.properties.numofjoints;i++)
//     {
//         if(njointsinset[i] != 0)
//             count++;
//     }
//
//     return  count;
// }


//bool ServiceParser::parseMCEncoderItem(Bottle &b_ENCODER,  vector<servMC_encoder_t> &encoders, char *encString)
//{
//#warning -> correct warnings on
//
//    Bottle b_ENCODER_TYPE(b_ENCODER.find("type").asString());
//    if(b_ENCODER_TYPE.isNull())
//    {
//        yError() << "ServiceParser::parseMCEncoderItem() cannot find JOINTMAPPING."<< encString<< ".type";
//        return false;
//    }
//
//    Bottle b_ENCODER_PORT(b_ENCODER.find("port").asString());
//    if(b_ENCODER_PORT.isNull())
//    {
//        yError() << "ServiceParser::parseMCEncoderItem() cannot find JOINTMAPPING"<< encString<< ".type";
//        return false;
//    }
//
//    encoders.resize(0);
//    int numofenc = b_ENCODER_TYPE.size() -1;
//
//     //VALE: controlla che il numero di act sia uguale a quello di joint per embobjmotion control obj!!!
//    for(int i = 0; i< numofenc; i++)
//    {
//        servMC_encoder_t item;
//        bool formaterror = false;
//        if(false == convert(b_ENCODER_TYPE.get(i+1).asString(), item.type, formaterror))
//        {
//             yError() << "ServiceParser::parseMCEncoderItem() encoder type not valid in " << encString;
//            return false;
//        }
//        //VALE: commented in ordet to compile I have not enouth time to implement this function
////         if(false == convert(b_ENCODER_TYPE.get(i+1).asString(), item.location, formaterror))
////         {
////              yError() << "ServiceParser::parseMCEncoderItem() encoder location not valid in " << encString;
////             return false;
////         }
//
//        if(false == convert(b_ENCODER_TYPE.get(i+1).asString(), item.position, formaterror))
//        {
//             yError() << "ServiceParser::parseMCEncoderItem() encoder location not valid in " << encString;
//            return false;
//        }
//        encoders.push_back(item);
//
//    }
//    return true;
//}


// bool ServiceParser::copyjomocouplingInfo( eOmc_4jomo_coupling_t *jc_dest)
// {
//
//     memset(jc_dest, 0, sizeof(eOmc_4jomo_coupling_t));
//
//     // very important: so far, the fw in Controller.c must find eomc_jointSetNum_none in un-used entries, even if we have less than 4 joints.
//     memset(jc_dest->joint2set, eomc_jointSetNum_none, sizeof(jc_dest->joint2set));
//
//     for(int i=0; i<mc_service.properties.numofjoints; i++)
//     {
//         jc_dest->joint2set[i] = mc_service.properties.joint2set[i];
//     }
//
//     for(int i=0; i<4; i++)
//     {
//         for(int j=0; j<4; j++)
//         {
//             jc_dest->joint2motor[i][j] = eo_common_float_to_Q17_14(mc_service.properties.controller.matrixJ2M[4*i+j]);
//             jc_dest->motor2joint[i][j] = eo_common_float_to_Q17_14(mc_service.properties.controller.matrixM2J[4*i+j]);
//         }
//     }
//
//
//     for(int r=0; r<4; r++)
//     {
//         for(int c=0; c<6; c++)
//         {
//             jc_dest->encoder2joint[r][c] = eo_common_float_to_Q17_14(mc_service.properties.controller.matrixE2J[6*r+c]);
//         }
//     }
//
//     for(int s=0; s< mc_service.properties.numofjointsets; s++)
//     {
//         memcpy(&jc_dest->jsetcfg[s]  , &mc_service.properties.jointset_cfgs[s], sizeof(eOmc_jointset_configuration_t));
//     }
//
//     return true;
// }



bool ServiceParser::parseService(Searchable &config, servConfigMC_t &mcconfig)
{
    bool ret = false;

    if(false == check_motion(config))
    {
        yError() << "ServiceParser::parseService() gets same errors parsing SERVICE MC group";
        return ret;
    }

    // if we dont find the xml file ... we just return true but mc_service.type is eomn_serv_MC_generic.
    // in such a case we dont transmit the config and ask the eth board to config itself accoring to its ip address.
    // ...

    mcconfig.ethservice.configuration.type = mc_service.type;
    
    switch(mc_service.type)
    {
        case eomn_serv_MC_foc:
        {
            eOmn_serv_config_data_mc_foc_t *data_mc = &(mcconfig.ethservice.configuration.data.mc.foc_based);

            // 1. ->version (of foc board).
            data_mc->version.firmware.major = mc_service.properties.canboards.at(0).firmware.major;
            data_mc->version.firmware.minor = mc_service.properties.canboards.at(0).firmware.minor;
            data_mc->version.firmware.build = mc_service.properties.canboards.at(0).firmware.build;
            data_mc->version.protocol.major = mc_service.properties.canboards.at(0).protocol.major;
            data_mc->version.protocol.minor = mc_service.properties.canboards.at(0).protocol.minor;

            // 2. ->arrayofjomodescriptors
            EOarray *arrayofjomos = eo_array_New(4, sizeof(eOmc_jomo_descriptor_t), &data_mc->arrayofjomodescriptors);
            int numofjomos = mc_service.properties.numofjoints;

            for(int i=0; i<numofjomos; i++)
            {
                eOmc_jomo_descriptor_t jomodes = {0};

                // 1. actuator is on foc: we need the address
                jomodes.actuator.foc.canloc.port = mc_service.properties.actuators[i].desc.foc.canloc.port;
                jomodes.actuator.foc.canloc.addr = mc_service.properties.actuators[i].desc.foc.canloc.addr;
                jomodes.actuator.foc.canloc.insideindex = eobrd_caninsideindex_first;
            
                // 2. encoder1 is ...
                jomodes.encoder1.type = mc_service.properties.encoder1s[i].desc.type;
                jomodes.encoder1.port = mc_service.properties.encoder1s[i].desc.port;
                jomodes.encoder1.pos = mc_service.properties.encoder1s[i].desc.pos;

                // 3. encoder2 is ...
                jomodes.encoder2.type = mc_service.properties.encoder2s[i].desc.type;
                jomodes.encoder2.port = mc_service.properties.encoder2s[i].desc.port;
                jomodes.encoder2.pos = mc_service.properties.encoder2s[i].desc.pos;

                eo_array_PushBack(arrayofjomos, &jomodes);

            }


            //// 3. ->jomocoupling
            //eOmc_4jomo_coupling_t *jomocoupling = &data_mc->jomocoupling;

            //ret = copyjomocouplingInfo(jomocoupling);

            // ok, everything is done
            ret = true;

        } break;
        
        case eomn_serv_MC_mc4:
        {
            eOmn_serv_config_data_mc_mc4_t *data_mc = &(mcconfig.ethservice.configuration.data.mc.mc4_based);

            // 1. ->version (of mc4 board) + ->mais.version.
            bool mc4boardFound = false;
            bool maisboardFound = false;
            for(size_t i=0; i<mc_service.properties.canboards.size(); i++)
            {
                if(eobrd_cantype_mc4 == mc_service.properties.canboards[i].type)
                {
                    data_mc->mc4version.protocol = mc_service.properties.canboards[i].protocol;
                    data_mc->mc4version.firmware = mc_service.properties.canboards[i].firmware;
                    mc4boardFound = true;
                }
                else if(eobrd_cantype_mais == mc_service.properties.canboards[i].type)
                {
                    data_mc->mais.version.protocol = mc_service.properties.canboards[i].protocol;
                    data_mc->mais.version.firmware = mc_service.properties.canboards[i].firmware;
                    maisboardFound = true;
                }
            }

            if((false == maisboardFound) || (false == mc4boardFound))
            {
                yError() << "ServiceParser::parseService() gets same errors parsing SERVICE MC group for mc4: cannot find description of mc4 or mais board";
            }

//            data_mc->mc4version.firmware.major = mc_service.properties.canboards.at(0).firmware.major;
//            data_mc->mc4version.firmware.minor = mc_service.properties.canboards.at(0).firmware.minor;
//            data_mc->mc4version.firmware.build = mc_service.properties.canboards.at(0).firmware.build;
//            data_mc->mc4version.protocol.major = mc_service.properties.canboards.at(0).protocol.major;
//            data_mc->mc4version.protocol.minor = mc_service.properties.canboards.at(0).protocol.minor;

            // 2. ->mc4shifts
            data_mc->mc4shifts = mc_service.properties.mc4shifts;

            // 3. ->mc4joints
            for(int i=0; i<12; i++)
            {
                data_mc->mc4joints[i] = mc_service.properties.mc4joints[i];
            }


            // 4. ->mais.canloc
            data_mc->mais.canloc = mc_service.properties.maislocation;

            // 5. ->broadcastflags
            data_mc->broadcastflags = 0;
            for(size_t i=0; i<mc_service.properties.mc4broadcasts.size(); i++)
            {
                eOmc_mc4broadcast_t item = mc_service.properties.mc4broadcasts[i];
                if((eomc_mc4broadcast_none != item) && (eomc_mc4broadcast_unknown != item))
                {
                    eo_common_hlfword_bitset(&data_mc->broadcastflags, item);
                }
            }

            // ok, everything is done
            ret = true;

        } break;
        
        case eomn_serv_MC_mc4plus:
        {
            eOmn_serv_config_data_mc_mc4plus_t *data_mc = &(mcconfig.ethservice.configuration.data.mc.mc4plus_based);

            // 3. ->arrayofjomodescriptors
            EOarray *arrayofjomos = eo_array_New(4, sizeof(eOmc_jomo_descriptor_t), &data_mc->arrayofjomodescriptors);
            int numofjomos = mc_service.properties.numofjoints;

            for(int i=0; i<numofjomos; i++)
            {
                eOmc_jomo_descriptor_t jomodes = {0};

                // 1. actuator is on pwm: we need the port
                jomodes.actuator.pwm.port = mc_service.properties.actuators[i].desc.pwm.port;

                // 2. encoder1 is ...
                jomodes.encoder1.type = mc_service.properties.encoder1s[i].desc.type;
                jomodes.encoder1.port = mc_service.properties.encoder1s[i].desc.port;
                jomodes.encoder1.pos = mc_service.properties.encoder1s[i].desc.pos;

                // 3. encoder2 is ...
                jomodes.encoder2.type = mc_service.properties.encoder2s[i].desc.type;
                jomodes.encoder2.port = mc_service.properties.encoder2s[i].desc.port;
                jomodes.encoder2.pos = mc_service.properties.encoder2s[i].desc.pos;

                eo_array_PushBack(arrayofjomos, &jomodes);

            }

            // // 3. ->jomocoupling
            //eOmc_4jomo_coupling_t *jomocoupling = &data_mc->jomocoupling;

            //ret = copyjomocouplingInfo(jomocoupling);

            // ok, everything is done
            ret = true;

        } break;
        
        case eomn_serv_MC_mc4plusmais:
        {
            eOmn_serv_config_data_mc_mc4plusmais_t *data_mc = &(mcconfig.ethservice.configuration.data.mc.mc4plusmais_based);

            // 1. ->mais
            eOmn_serv_config_data_as_mais_t *mais = &data_mc->mais;

            mais->canloc.port = mc_service.properties.maislocation.port;
            mais->canloc.addr = mc_service.properties.maislocation.addr;
            mais->canloc.insideindex = mc_service.properties.maislocation.insideindex;

            mais->version.firmware.major = mc_service.properties.canboards.at(0).firmware.major;
            mais->version.firmware.minor = mc_service.properties.canboards.at(0).firmware.minor;
            mais->version.firmware.build = mc_service.properties.canboards.at(0).firmware.build;
            mais->version.protocol.major = mc_service.properties.canboards.at(0).protocol.major;
            mais->version.protocol.minor = mc_service.properties.canboards.at(0).protocol.minor;

            // 2. ->arrayofjomodescriptors
            EOarray *arrayofjomos = eo_array_New(4, sizeof(eOmc_jomo_descriptor_t), &data_mc->arrayofjomodescriptors);
            int numofjomos = mc_service.properties.numofjoints;

            for(int i=0; i<numofjomos; i++)
            {
                eOmc_jomo_descriptor_t jomodes = {0};

                // 1. actuator is on pwm: we need the port
                jomodes.actuator.pwm.port = mc_service.properties.actuators[i].desc.pwm.port;

                // 2. encoder1 is ...
                jomodes.encoder1.type = mc_service.properties.encoder1s[i].desc.type;
                jomodes.encoder1.port = mc_service.properties.encoder1s[i].desc.port;
                jomodes.encoder1.pos = mc_service.properties.encoder1s[i].desc.pos;

                // 3. encoder2 is ...
                jomodes.encoder2.type = mc_service.properties.encoder2s[i].desc.type;
                jomodes.encoder2.port = mc_service.properties.encoder2s[i].desc.port;
                jomodes.encoder2.pos = mc_service.properties.encoder2s[i].desc.pos;

                eo_array_PushBack(arrayofjomos, &jomodes);

            }


            //// 3. ->jomocoupling
            //eOmc_4jomo_coupling_t *jomocoupling = &data_mc->jomocoupling;

            //ret = copyjomocouplingInfo(jomocoupling);

            // ok, everything is done
            ret = true;

        } break;

        case eomn_serv_MC_mc2pluspsc:
        {
            eOmn_serv_config_data_mc_mc2pluspsc_t *data_mc = &(mcconfig.ethservice.configuration.data.mc.mc2pluspsc);

            // 1. ->psc
            eOmn_serv_config_data_as_psc_t *psc = &data_mc->psc;


            for(int i=0; i<mc_service.properties.psclocations.size(); i++)
            {
                psc->boardInfo.canloc[i].port=mc_service.properties.psclocations[i].port;
                psc->boardInfo.canloc[i].addr=mc_service.properties.psclocations[i].addr;
                psc->boardInfo.canloc[i].insideindex=mc_service.properties.psclocations[i].insideindex;
            }


            psc->version.firmware.major = mc_service.properties.canboards.at(0).firmware.major;
            psc->version.firmware.minor = mc_service.properties.canboards.at(0).firmware.minor;
            psc->version.firmware.build = mc_service.properties.canboards.at(0).firmware.build;
            psc->version.protocol.major = mc_service.properties.canboards.at(0).protocol.major;
            psc->version.protocol.minor = mc_service.properties.canboards.at(0).protocol.minor;

            // 2. ->arrayofjomodescriptors
            EOarray *arrayofjomos = eo_array_New(4, sizeof(eOmc_jomo_descriptor_t), &data_mc->arrayofjomodescriptors);
            int numofjomos = mc_service.properties.numofjoints;

            for(int i=0; i<numofjomos; i++)
            {
                eOmc_jomo_descriptor_t jomodes = {0};

                // 1. actuator is on pwm: we need the port
                jomodes.actuator.pwm.port = mc_service.properties.actuators[i].desc.pwm.port;

                // 2. encoder1 is ...
                jomodes.encoder1.type = mc_service.properties.encoder1s[i].desc.type;
                jomodes.encoder1.port = mc_service.properties.encoder1s[i].desc.port;
                jomodes.encoder1.pos = mc_service.properties.encoder1s[i].desc.pos;

                // 3. encoder2 is ...
                jomodes.encoder2.type = mc_service.properties.encoder2s[i].desc.type;
                jomodes.encoder2.port = mc_service.properties.encoder2s[i].desc.port;
                jomodes.encoder2.pos = mc_service.properties.encoder2s[i].desc.pos;

                eo_array_PushBack(arrayofjomos, &jomodes);

            }

            // ok, everything is done
            ret = true;

        } break;

        case eomn_serv_MC_generic:
        {
            yWarning() << "ServiceParser::parseService() eomn_serv_MC_generic detected. set type = eomn_serv_MC_generic to let the remote board configure itself according to its IP address";
            mcconfig.ethservice.configuration.type = eomn_serv_MC_generic;
            ret =  true;
        } break;

        default:
        {
            yError() << "ServiceParser::parseService() unknown value in eOmn_serv_type_t field";
            ret = false;
        } break;
    }
    

    return ret;
}


servMC_encoder_t * ServiceParser::getEncoderAtJoint(int index)
{
    if(mc_service.properties.encoder1s[index].desc.pos ==  eomc_pos_atjoint)
    {
        return(&mc_service.properties.encoder1s[index]);
    }
    else if(mc_service.properties.encoder2s[index].desc.pos ==  eomc_pos_atjoint)
    {
        return(&mc_service.properties.encoder2s[index]);
    }
    else
        return NULL;
}

servMC_encoder_t * ServiceParser::getEncoderAtMotor(int index)
{
    if(mc_service.properties.encoder1s[index].desc.pos ==  eomc_pos_atmotor)
    {
        return(&mc_service.properties.encoder1s[index]);
    }
    else if(mc_service.properties.encoder2s[index].desc.pos ==  eomc_pos_atmotor)
    {
        return(&mc_service.properties.encoder2s[index]);
    }
    else
        return NULL;

}

static  eOmn_serv_configuration_t s_serv_config_mc_v3_0B0;
/*static const eOmn_serv_configuration_t s_serv_config_mc_v3_0B0 =
{   // eb12 or 0B0
    .type       = eomn_serv_MC_mc4plus,
    .filler     = {0},
    .data.mc.mc4plus_based =
    {
        .boardtype4mccontroller = emscontroller_board_HEAD_neckpitch_neckroll,
        .filler                 = {0},
        .arrayofjomodescriptors =
        {
            .head   =
            {
                .capacity       = 4,
                .itemsize       = sizeof(eOmn_serv_jomo_descriptor_t),
                .size           = 2,
                .internalmem    = 0
            },
            .data   =
            {
                { // joint 0: neck-pitch
                    .actuator.pwm   =
                    {
                        .port   = 0  // eomn_serv_mc_port_mc4plus_pwmP3 is hal_motor1=0 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_aea,
                        .port   = 1, // eomn_serv_mc_port_mc4plus_spiP11 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 0,  // eomn_serv_mc_port_mc4plus_qencP3 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 1: neck-roll
                    .actuator.pwm   =
                    {
                        .port   = 2  // eomn_serv_mc_port_mc4plus_pwmP4 is hal_motor3=2 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_aea,
                        .port   = 0, // eomn_serv_mc_port_mc4plus_spiP10 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 2,  // eomn_serv_mc_port_mc4plus_qencP4 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 2
                    .actuator.pwm   =
                    {
                        .port   = eomn_serv_mc_port_none
                    },
                    .sensor         =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    },
                    .encoder2    =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    }
                },
                { // joint 3
                    .actuator.pwm   =
                    {
                        .port   = eomn_serv_mc_port_none
                    },
                    .sensor         =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    },
                    .encoder2    =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    }
                }
            }
        },
        .jomocoupling       =
        {
            .joint2set      =
            {
                eomc_jointSetNum_zero, eomc_jointSetNum_zero, eomc_jointSetNum_none, eomc_jointSetNum_none
            },
            .joint2motor    =
            {   // zero matrix: use matrix embedded in controller and seecetd by boardtype4mccontroller
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(-1.0f),     EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            },
            .encoder2joint  =
            {   // identical matrix
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            }
        }
    }
};
*/
static  eOmn_serv_configuration_t s_serv_config_mc_v3_0B9;
/*static const eOmn_serv_configuration_t s_serv_config_mc_v3_0B9=
{   // eb15 or 0B9
    .type       = eomn_serv_MC_mc4plus,
    .filler     = {0},
    .data.mc.mc4plus_based =
    {
        .boardtype4mccontroller = emscontroller_board_4jointsNotCoupled,
        .filler                 = {0},
        .arrayofjomodescriptors =
        {
            .head   =
            {
                .capacity       = 4,
                .itemsize       = sizeof(eOmn_serv_jomo_descriptor_t),
                .size           = 4,
                .internalmem    = 0
            },
            .data   =
            {
                { // joint 0: lip-left
                    .actuator.pwm   =
                    {
                        .port   = 2  // eomn_serv_mc_port_mc4plus_pwmP4 is hal_motor2=1 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 2, // eomn_serv_mc_port_mc4plus_qencP4 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 2,  // eomn_serv_mc_port_mc4plus_qencP4 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 1: lip-high
                    .actuator.pwm   =
                    {
                        .port   = 1  // eomn_serv_mc_port_mc4plus_pwmP2 is hal_motor2=1 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 1, // eomn_serv_mc_port_mc4plus_qencP2 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 1,  // eomn_serv_mc_port_mc4plus_qencP2 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 2: lip-right
                    .actuator.pwm   =
                    {
                        .port   = 0  // eomn_serv_mc_port_mc4plus_pwmP3 is hal_motor4=3 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 0, // eomn_serv_mc_port_mc4plus_qencP3 is hal_quad_enc1 = 0 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 0, // eomn_serv_mc_port_mc4plus_qencP3 is hal_quad_enc1 = 0 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 3: lip-bottom
                    .actuator.pwm   =
                    {
                        .port   = 3  // eomn_serv_mc_port_mc4plus_pwmP5 is hal_motor3=2 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 3, // eomn_serv_mc_port_mc4plus_qencP5 is hal_quad_enc4 = 3 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 3, // eomn_serv_mc_port_mc4plus_qencP5 is hal_quad_enc4 = 3 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                }
            }
        },
        .jomocoupling       =
        {
            .joint2set      =
            {   // each joint is on a different set
                0, 1, 2, 3
            },
            .joint2motor    =
            {   // zero matrix: use matrix embedded in controller and seecetd by boardtype4mccontroller
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) }
            },
            .encoder2joint  =
            {   // identical matrix
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            }
        }
    }
};*/




static  eOmn_serv_configuration_t s_serv_config_mc_v3_0B1;
/*static const eOmn_serv_configuration_t s_serv_config_mc_v3_0B1 =
{   // eb13 or 0B1
    .type       = eomn_serv_MC_mc4plus,
    .filler     = {0},
    .data.mc.mc4plus_based =
    {
        .boardtype4mccontroller = emscontroller_board_HEAD_neckyaw_eyes,
        .filler                 = {0},
        .arrayofjomodescriptors =
        {
            .head   =
            {
                .capacity       = 4,
                .itemsize       = sizeof(eOmn_serv_jomo_descriptor_t),
                .size           = 4,
                .internalmem    = 0
            },
            .data   =
            {
                { // joint 0: neck-yaw
                    .actuator.pwm   =
                    {
                        .port   = 0  // eomn_serv_mc_port_mc4plus_pwmP3 is hal_motor1=0 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_aea,
                        .port   = 1, // eomn_serv_mc_port_mc4plus_spiP11 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 0,  // eomn_serv_mc_port_mc4plus_qencP3 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 1: eyes-tilt
                    .actuator.pwm   =
                    {
                        .port   = 1  // eomn_serv_mc_port_mc4plus_pwmP2 is hal_motor2=1 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_aea,
                        .port   = 0, // eomn_serv_mc_port_mc4plus_spiP10 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 1,  // eomn_serv_mc_port_mc4plus_qencP2 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 2: right-eye
                    .actuator.pwm   =
                    {
                        .port   = 3  // eomn_serv_mc_port_mc4plus_pwmP5 is hal_motor4=3 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 3, // eomn_serv_mc_port_mc4plus_qencP5 is hal_quad_enc4 = 3 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 3, // eomn_serv_mc_port_mc4plus_qencP5 is hal_quad_enc4 = 3 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 3: left-eye
                    .actuator.pwm   =
                    {
                        .port   = 2  // eomn_serv_mc_port_mc4plus_pwmP4 is hal_motor3=2 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 2, // eomn_serv_mc_port_mc4plus_qencP4 is hal_quad_enc3 = 2 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 2, // eomn_serv_mc_port_mc4plus_qencP4 is hal_quad_enc3 = 2 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                }
            }
        },
        .jomocoupling       =
        {
            .joint2set      =
            { //joint 2 and 3 are coupled
                eomc_jointSetNum_zero, eomc_jointSetNum_one, eomc_jointSetNum_two, eomc_jointSetNum_two
            },

                    //inverted matrix: joint to motor
                    // |m0|    | 1     0       0      0 |    |j0|
                    // |m1|  = | 0     1       0      0 |  * |j1|
                    // |m2|    | 0     0       1     -1 |    |j2|
                    // |m3|    | 0     0       1      1 |    |j2|


            .joint2motor    =
            {   // zero matrix: use matrix embedded in controller and seecetd by boardtype4mccontroller
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(-1.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            },
            .encoder2joint  =
            {   // identical matrix
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            }
        }
    }
};
*/
static  eOmn_serv_configuration_t s_serv_config_mc_v3_0B7;
/*static const eOmn_serv_configuration_t s_serv_config_mc_v3_0B7 =
{   // eb14 or 0B7f
    .type       = eomn_serv_MC_mc4plus,
    .filler     = {0},
    .data.mc.mc4plus_based =
    {
        .boardtype4mccontroller = emscontroller_board_FACE_eyelids_jaw,
        .filler                 = {0},
        .arrayofjomodescriptors =
        {
            .head   =
            {
                .capacity       = 4,
                .itemsize       = sizeof(eOmn_serv_jomo_descriptor_t),
                .size           = 2,
                .internalmem    = 0
            },
            .data   =
            {
                { // joint 0: jaw
                    .actuator.pwm   =
                    {
                        .port   = 0  // eomn_serv_mc_port_mc4plus_pwmP3 is hal_motor1=0 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_aea,
                        .port   = 1, // eomn_serv_mc_port_mc4plus_spiP11 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    }
                },
                { // joint 1: eyelids
                    .actuator.pwm   =
                    {
                        .port   = 1  // eomn_serv_mc_port_mc4plus_pwmP2 is hal_motor2=1 ?? verify!
                    },
                    .sensor         =
                    {
                        .type   = eomc_enc_aea,
                        .port   = 0, // eomn_serv_mc_port_mc4plus_spiP10 ?? verify!
                        .pos    = eomc_pos_atjoint
                    },
                    .encoder2    =
                    {
                        .type   = eomc_enc_qenc,
                        .port   = 1,  // eomn_serv_mc_port_mc4plus_qencP2 ?? verify!
                        .pos    = eomc_pos_atmotor
                    }
                },
                { // joint 2
                    .actuator.pwm   =
                    {
                        .port   = eomn_serv_mc_port_none
                    },
                    .sensor         =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    },
                    .encoder2    =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    }
                },
                { // joint 3
                    .actuator.pwm   =
                    {
                        .port   = eomn_serv_mc_port_none
                    },
                    .sensor         =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    },
                    .encoder2    =
                    {
                        .type   = eomn_serv_mc_sensor_none,
                        .port   = eomn_serv_mc_port_none,
                        .pos    = eomn_serv_mc_sensor_pos_none
                    }
                }
            }
        },
        .jomocoupling       =
        {
            .joint2set      =
            {   // each joint is on a different set
                eomc_jointSetNum_zero, eomc_jointSetNum_one, eomc_jointSetNum_none, eomc_jointSetNum_none
            },
            .joint2motor    =
            {   // zero matrix: use matrix embedded in controller and seecetd by boardtype4mccontroller
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            },
            .encoder2joint  =
            {   // identical matrix
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            }
        }
    }
};
*/

/*eOmn_serv_configuration_t*  arrayConf[4] =
{
    &s_serv_config_mc_v3_0B9,       // board ip.1, 0b9, face 4 lips
    &s_serv_config_mc_v3_0B7,       // board ip.2, 0b7, face eyelids + jaw
    &s_serv_config_mc_v3_0B1,       // board ip.3, 0b1, head neck yaw + 3 eyes
    &s_serv_config_mc_v3_0B0
}*/
bool ServiceParser::parseService2(Searchable &config, servConfigMC_t &mcconfig)
{
    eOmc_arrayof_4jomodescriptors_t *ja = &s_serv_config_mc_v3_0B1.data.mc.mc4plus_based.arrayofjomodescriptors;
    eOmc_4jomo_coupling_t *jc = &s_serv_config_mc_v3_0B1.data.mc.mc4plus_based.jomocoupling;


eOq17_14_t m1[][4]    =
    {   // zero matrix: use matrix embedded in controller and seecetd by boardtype4mccontroller
        { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
        { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
        { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(-1.0f) },
        { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
    };

eOq17_14_t  m2[][4]=
            {   // identical matrix
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            };










   // s_serv_config_mc_v3_0B1.type       = eomn_serv_MC_mc4plus;
    //s_serv_config_mc_v3_0B1.filler     = {0, 0, 0};

    //s_serv_config_mc_v3_0B1.data.mc.mc4plus_based.boardtype4mccontroller = 6; //emscontroller_board_HEAD_neckyaw_eyes;
    //s_serv_config_mc_v3_0B1.data.mc.mc4plus_based.filler = {0};

    ja->head.capacity       = 4;
    ja->head.itemsize       = sizeof(eOmc_jomo_descriptor_t);
    ja->head.size           = 4;
    ja->head.internalmem    = 0;

    ja->data[0].actuator.pwm.port   = 0 ; // eomn_serv_mc_port_mc4plus_pwmP3 is hal_motor1=0 ?? verify!
    ja->data[0].encoder1.type   = eomc_enc_aea;
    ja->data[0].encoder1.port   = 1; // eomn_serv_mc_port_mc4plus_spiP11 ?? verify!
    ja->data[0].encoder1.pos    = eomc_pos_atjoint;

    ja->data[0].encoder2.type   = eomc_enc_qenc;
    ja->data[0].encoder2.port   = 0;  // eomn_serv_mc_port_mc4plus_qencP3 ?? verify!
    ja->data[0].encoder2.pos    = eomc_pos_atmotor;

    ja->data[1].actuator.pwm.port   = 1;  // eomn_serv_mc_port_mc4plus_pwmP2 is hal_motor2=1 ?? verify!
    ja->data[1].encoder1.type   = eomc_enc_aea;
    ja->data[1].encoder1.port   = 0; // eomn_serv_mc_port_mc4plus_spiP10 ?? verify!
    ja->data[1].encoder1.pos    = eomc_pos_atjoint;
    ja->data[1].encoder2.type   = eomc_enc_qenc;
    ja->data[1].encoder2.port   = 1;  // eomn_serv_mc_port_mc4plus_qencP2 ?? verify!
    ja->data[1].encoder2.pos    = eomc_pos_atmotor;

    ja->data[2].actuator.pwm.port   = 3;
    ja->data[2].encoder1.type   = eomc_enc_qenc;
    ja->data[2].encoder1.port   = 3; // eomn_serv_mc_port_mc4plus_qencP5 is hal_quad_enc4 = 3 ?? verify!
    ja->data[2].encoder1.pos    = eomc_pos_atjoint;
    ja->data[2].encoder2.type   = eomc_enc_qenc,
    ja->data[2].encoder2.port   = 3, // eomn_serv_mc_port_mc4plus_qencP5 is hal_quad_enc4 = 3 ?? verify!
    ja->data[2].encoder2.pos    = eomc_pos_atmotor;

    ja->data[3].actuator.pwm.port   = 2;

    ja->data[3].encoder1.type   = eomc_enc_qenc;
    ja->data[3].encoder1.port   = 2; // eomn_serv_mc_port_mc4plus_qencP4 is hal_quad_enc3 = 2 ?? verify!
    ja->data[3].encoder1.pos    = eomc_pos_atjoint;


    ja->data[3].encoder2.type   = eomc_enc_qenc;
    ja->data[3].encoder2.port   = 2; // eomn_serv_mc_port_mc4plus_qencP4 is hal_quad_enc3 = 2 ?? verify!
    ja->data[3].encoder2.pos    = eomc_pos_atmotor;


    jc->joint2set[0] =  eomc_jointSetNum_zero;
    jc->joint2set[1] =  eomc_jointSetNum_one;
    jc->joint2set[2] = eomc_jointSetNum_two;
    jc->joint2set[3] = eomc_jointSetNum_two;


/*
    jc->joint2motor    =
    {   // zero matrix: use matrix embedded in controller and seecetd by boardtype4mccontroller
        { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
        { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
        { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(-1.0f) },
        { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
    };
    jc->encoder2joint  =
            {   // identical matrix
                { EO_COMMON_FLOAT_TO_Q17_14(1.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f) },
                { EO_COMMON_FLOAT_TO_Q17_14(0.0f),      EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(0.0f),    EO_COMMON_FLOAT_TO_Q17_14(1.0f) }
            };



*/






memcpy(&jc->joint2motor, &m1, sizeof(m1));
memcpy(&jc->encoder2joint, &m2, sizeof(m2));

yError() << "print first m ";
for(int r=0; r<4; r++)
{
    for(int c=0; c<4; c++)
    {
        yError() << "r=" <<r << "c=" << c << "val=" << m1[r][c];
    }
}

yError() << "print second  m ";
for(int r=0; r<4; r++)
{
    for(int c=0; c<4; c++)
    {
        yError() << "r=" <<r << "c=" << c << "val=" << m2[r][c];
    }
}



    if(mcconfig.id > 3)
    {
        yError() << " id scheda sbagliato " << mcconfig.id;
        return false;
    }


    memcpy(&mcconfig.ethservice, &s_serv_config_mc_v3_0B1, sizeof(eOmn_serv_configuration_t));
    return true;
}
#endif

// eof
