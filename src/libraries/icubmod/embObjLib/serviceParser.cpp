
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


    as_service.type = eomn_serv_NONE;

    as_service.properties.canboards.resize(0);
    as_service.properties.sensors.resize(0);

    as_service.settings.acquisitionrate = 0;
    as_service.settings.enabledsensors.resize(0);
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


bool convert(ConstString const &fromstring, eOmc_ctrlboard_t &controllerboard, bool &formaterror)
{
    const char *t = fromstring.c_str();
    eObool_t usecompactstring = eobool_false;
    controllerboard = eomc_string2controllerboard(t, usecompactstring);

    if(eomc_ctrlboard_unknown == controllerboard)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        controllerboard = eomc_string2controllerboard(t, usecompactstring);
    }

    if(eomc_ctrlboard_unknown == controllerboard)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmc_ctrlboard_t";
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

bool ServiceParser::convert(ConstString const &fromstring, eObrd_ethtype_t& tobrdethtype, bool& formaterror)
{
    const char *t = fromstring.c_str();

    eObrd_type_t type = eoboards_string2type(t);
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
    if((eomn_serv_AS_mais != type) && (eomn_serv_AS_strain != type) && (eomn_serv_AS_inertials != type))
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



#if defined(SERVICE_PARSER_USE_MC)

bool ServiceParser::convert(Bottle &bottle, vector<double> &matrix, bool &formaterror, int targetsize)
{
    matrix.resize(0);
    
    int tmp = bottle.size();
    int sizeofmatrix = tmp - 1;    // first position of bottle contains the tag "matrix"

    // check if there are really the target number of elements in matrix.
    if(targetsize != sizeofmatrix)
    {
        yError() << "ServiceParser::check() in a SERVICE.PROPERTIES.CONTROLLER.matrix there are not" << targetsize << "elements";
        return false;
    }

    formaterror = false;
    for(int i=0; i<sizeofmatrix; i++)
    {
        double item = 0;

        // ok, i use the standard converter ... but what if it is not a double format? so far we dont check.
        item = bottle.get(i+1).asDouble();
        matrix.push_back(item);
    }

    // in here we could decide to return false if any previous conversion function has returned error
    
    return true;        
}



bool ServiceParser::convert(ConstString const &fromstring, servMC_actuator_location_t &location, bool &formaterror)
{
    // it is actually a micro-parser: PRE-num
    // at

    const char *t = fromstring.c_str();
    int len = strlen(t);

    if(len > 15)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because it is too long with size =" << len;
        formaterror = true;
        return false;
    }

    char prefix[16] = {0};
    sscanf(t, "%3c", prefix);
    if(0 == strcmp(prefix, "CAN")) //VALE: questa parte e'particamente uguale a quella nella funzione convert con parametro eObrd_location_t. Fai una funzioncina per evitare di replicare il codice.
    {
        int bus = 0;
        int adr = 0;
        char cc = 'x';
        int sub = 9;
        int numberofreaditems = sscanf(t, "%3c%d:%d%c%d", prefix, &bus, &adr, &cc, &sub);

        if((3 != numberofreaditems) && (5 != numberofreaditems))
        {
            yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because we dont have correct number of sections separated by :";
            formaterror = true;
            return false;
        }

        // verify bus being eitehr 1 or 2, and adr being 1 ----- 14
        if((1 != bus) && (2 != bus))
        {
            yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because we can have either CAN1 or CAN2";
            formaterror = true;
            return false;
        }
        if((0 == adr) || (adr > 14))
        {
            yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because CAN address is in range [1, 14]";
            formaterror = true;
            return false;
        }

        location.oncan.place = servMC_actuator_place_can;
        location.oncan.port = (1 == bus) ? (eOcanport1) : (eOcanport2);
        location.oncan.addr = adr;
        location.oncan.index = eobrd_caninsideindex_first;
        if(5 == numberofreaditems)//if user specified index (mc4 can case)
        {
            if((0 != sub) && (1 != sub))
            {
                yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because in CANx:adr:SUB, SUB address must be in range [0, 1]";
                formaterror = true;
                return false;
            }
            location.oncan.index = (0 == sub) ? (eobrd_caninsideindex_first) : (eobrd_caninsideindex_second);
        }
        

    }
    else if(0 == strcmp(prefix, "LOC"))
    {
        //Add here parsing for local port (both PWM port (P7, P8, etc)
        //in xml file we have: LOC:P7
        int bus = 0;
        int adr = 0;
        char cc = 'x';
        int port = 9;
        int numberofreaditems = sscanf(t, "%3c:%c%d", prefix, &cc, &port);
        if(0 != strcmp(&cc, "P"))
        {
             yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because in LOC:Px, because there is not P char";
             formaterror = true;
             return false;
        }
        if(port > servMC_board_connector_PMAX-1)
        {
             yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because in LOC:Px, because x is bigger then " << (servMC_board_connector_PMAX-1);
             formaterror = true;
             return false;
        }
        location.local.place = servMC_actuator_place_local;
        location.local.boardConnector = (servMC_board_connector_t)port;    
        
    }
    else
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for a servMC_actuator_location_t because it does not begin with ETH or CAN";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(ConstString const &fromstring, eOmc_actuator_t &toactuatortype, bool &formaterror)
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


bool ServiceParser::convert(ConstString const &fromstring, eOmc_position_t &tosensorposition, bool &formaterror)
{
    const char *t = fromstring.c_str();
    eObool_t usecompactstring = eobool_false;
    tosensorposition = eomc_string2position(t, usecompactstring);

    if(eomc_pos_unknown == tosensorposition)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        tosensorposition = eomc_string2position(t, usecompactstring);
    }

    if(eomc_pos_unknown == tosensorposition)
    {
        yWarning() << "ServiceParser::convert():" << t << "is not a legal string for eOmc_position_t";
        formaterror = true;
        return false;
    }

    return true;
}


bool ServiceParser::convert(ConstString const &fromstring, eOmc_encoder_t &tosensortype, bool &formaterror)
{
    const char *t = fromstring.c_str();
    eObool_t usecompactstring = eobool_false;
    tosensortype = eomc_string2encoder(t, usecompactstring);

    if(eomc_enc_unknown == tosensortype)
    {   // attempting to retrieve the compact form
        usecompactstring = eobool_true;
        tosensortype = eomc_string2encoder(t, usecompactstring);
    }

    if(eomc_enc_unknown == tosensortype)
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
    
    // format is SERVICE{ type, PROPERTIES{ ETHBOARD, CANBOARDS, CONTROLLER, JOINTMAPPING, JOINTSETS } }
    // so far, there is no SETTINGS{}.

    Bottle b_SERVICE(config.findGroup("SERVICE"));
    if(b_SERVICE.isNull())
    {
        yError() << "ServiceParser::check_motion() cannot find SERVICE group";
        return false;
    }

    // check whether we have the proper type

    if(false == b_SERVICE.check("type"))
    {
        yError() << "ServiceParser::check_motion() cannot find SERVICE.type";
        return false;
    }

    Bottle b_type(b_SERVICE.find("type").asString());
    if(false == convert(b_type.toString(), mc_service.type, formaterror))
    {
        yError() << "ServiceParser::check_motion() has found unknown SERVICE.type = " << b_type.toString();
        return false;
    }



    // check whether we have the proper groups at first level.

    Bottle b_PROPERTIES = Bottle(b_SERVICE.findGroup("PROPERTIES"));
    if(b_PROPERTIES.isNull())
    {
        yError() << "ServiceParser::check_motion() cannot find PROPERTIES";
        return false;
    }
    else
    {
        //bool has_PROPERTIES = true;

        // now, inside PROPERTIES there are groups which depend on mc_service.type.
        // i prefer to check them all in here rather to go on and check them one after another.
        Bottle b_PROPERTIES_ETHBOARD = Bottle(b_PROPERTIES.findGroup("ETHBOARD"));
        bool has_PROPERTIES_ETHBOARD = !b_PROPERTIES_ETHBOARD.isNull();

        Bottle b_PROPERTIES_CANBOARDS = Bottle(b_PROPERTIES.findGroup("CANBOARDS"));
        bool has_PROPERTIES_CANBOARDS = !b_PROPERTIES_CANBOARDS.isNull();

        Bottle b_PROPERTIES_CONTROLLER = Bottle(b_PROPERTIES.findGroup("CONTROLLER"));
        bool has_PROPERTIES_CONTROLLER = !b_PROPERTIES_CONTROLLER.isNull();

        Bottle b_PROPERTIES_JOINTMAPPING = Bottle(b_PROPERTIES.findGroup("JOINTMAPPING"));
        bool has_PROPERTIES_JOINTMAPPING = !b_PROPERTIES_JOINTMAPPING.isNull();

        Bottle b_PROPERTIES_JOINTSETS = Bottle(b_PROPERTIES.findGroup("JOINTSETS"));
        bool has_PROPERTIES_JOINTSETS = !b_PROPERTIES_JOINTSETS.isNull();


        bool itisoksofar = false;
        switch(mc_service.type)
        {
            case eomn_serv_MC_foc:
            {
                // must have: ETHBOARD, CANBOARDS, CONTROLLER, JOINTMAPPING, JOINTSETS
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

                if(false == has_PROPERTIES_CONTROLLER)
                {
                    yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CONTROLLER for type" << eomn_servicetype2string(mc_service.type);
                    itisoksofar = false;
                }

                if(false == has_PROPERTIES_JOINTMAPPING)
                {
                    yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING for type" << eomn_servicetype2string(mc_service.type);
                    itisoksofar = false;
                }

                if(false == has_PROPERTIES_JOINTSETS)
                {
                    yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTSETS for type" << eomn_servicetype2string(mc_service.type);
                    itisoksofar = false;
                }

            } break;

            case eomn_serv_MC_mc4:
            {
                yError() << "ServiceParser::check_motion() does not support eomn_serv_MC_mc4 yet";
                itisoksofar = false;
            } break;

            case eomn_serv_MC_mc4plus:
            {
                // must have: ETHBOARD, CONTROLLER, JOINTMAPPING, JOINTSETS
                yError() << "ServiceParser::check_motion() does not support eomn_serv_MC_mc4plus yet";
                itisoksofar = false;
            } break;

            case eomn_serv_MC_mc4plusmais:
            {
                // must have: ETHBOARD, CANBOARDS, CONTROLLER, JOINTMAPPING, JOINTSETS
                yError() << "ServiceParser::check_motion() does not support eomn_serv_MC_mc4plusmais yet";
                itisoksofar = false;
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

            if(false == convert(b_PROPERTIES_ETHBOARD_type.toString(), mc_service.properties.ethboardtype, formaterror))
            {
                yError() << "ServiceParser::check_motion() has found unknown SERVICE.PROPERTIES.ETHBOARD.type = " << b_PROPERTIES_ETHBOARD_type.toString();
                return false;
            }

        } // has_PROPERTIES_ETHBOARD


        if(true == has_PROPERTIES_CANBOARDS)
        {
            // i get type, PROTOCOL.major/minor, FIRMWARE.major/minor/build and see their sizes. they must be all equal.

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


        if(true == has_PROPERTIES_CONTROLLER)
        {
            formaterror = false;

            Bottle b_PROPERTIES_CONTROLLER_type(b_PROPERTIES_CONTROLLER.find("type").asString());
            if(b_PROPERTIES_CONTROLLER_type.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.CONTROLLER.type";
                return false;
            }

            if(false == convert(b_PROPERTIES_CONTROLLER_type.toString(), mc_service.properties.controller.type, formaterror))
            {
                yError() << "ServiceParser::check_motion() has found unknown SERVICE.PROPERTIES.CONTROLLER.type = " << b_PROPERTIES_CONTROLLER_type.toString();
                return false;
            }

            // matrix J2M

            Bottle b_PROPERTIES_CONTROLLER_matrixJ2M = Bottle(b_PROPERTIES_CONTROLLER.findGroup("matrixJ2M"));
            if(b_PROPERTIES_CONTROLLER_matrixJ2M.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find SERVICE.PROPERTIES.CONTROLLER.matrixJ2M";
                return false;
            }

            formaterror = false;
            if(false == convert(b_PROPERTIES_CONTROLLER_matrixJ2M, mc_service.properties.controller.matrixJ2M, formaterror, 16))
            {
                yError() << "ServiceParser::check_motion() has detected an illegal format for some of the values of SERVICE.PROPERTIES.CONTROLLER.matrixJ2M";
                return false;
            }


            // matrix E2J

            Bottle b_PROPERTIES_CONTROLLER_matrixE2J = Bottle(b_PROPERTIES_CONTROLLER.findGroup("matrixE2J"));
            if(b_PROPERTIES_CONTROLLER_matrixE2J.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find SERVICE.PROPERTIES.CONTROLLER.matrixE2J";
                return false;
            }

#warning TODO: if we need 6x6 matrixE2J, change 16 into 36 and then change mn data structure ....
            formaterror = false;
            if(false == convert(b_PROPERTIES_CONTROLLER_matrixE2J, mc_service.properties.controller.matrixE2J, formaterror, 16))
            {
                yError() << "ServiceParser::check_motion() has detected an illegal format for some of the values of SERVICE.PROPERTIES.CONTROLLER.matrixE2J";
                return false;
            }


            // matrix M2J
            mc_service.properties.controller.matrixM2J.resize(0);
//            Bottle b_PROPERTIES_CONTROLLER_matrixM2J = Bottle(b_PROPERTIES_CONTROLLER.findGroup("matrixM2J"));
//            if(b_PROPERTIES_CONTROLLER_matrixM2J.isNull())
//            {
//                yError() << "ServiceParser::check_motion() cannot find SERVICE.PROPERTIES.CONTROLLER.matrixM2J";
//                return false;
//            }
//
//            formaterror = false;
//            if( false == convert(b_PROPERTIES_CONTROLLER_matrixM2J, mc_service.properties.controller.matrixM2J, formaterror, 16))
//            {
//                yError() << "ServiceParser::check_motion() has detected an illegal format for some of the values of SERVICE.PROPERTIES.CONTROLLER.matrixM2J";
//                return false;
//            }

        } // has_PROPERTIES_CONTROLLER


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

            Bottle b_PROPERTIES_JOINTMAPPING_ACTUATOR_type(b_PROPERTIES_JOINTMAPPING_ACTUATOR.find("type").asString());
            if(b_PROPERTIES_JOINTMAPPING_ACTUATOR_type.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ACTUATOR.type";
                return false;
            }
            Bottle b_PROPERTIES_JOINTMAPPING_ACTUATOR_port(b_PROPERTIES_JOINTMAPPING_ACTUATOR.find("port").asString());
            if(b_PROPERTIES_JOINTMAPPING_ACTUATOR_port.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ACTUATOR.port";
                return false;
            }

            Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_type(b_PROPERTIES_JOINTMAPPING_ENCODER1.find("type").asString());
            if(b_PROPERTIES_JOINTMAPPING_ENCODER1_type.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.type";
                return false;
            }
            Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_port(b_PROPERTIES_JOINTMAPPING_ENCODER1.find("port").asString());
            if(b_PROPERTIES_JOINTMAPPING_ENCODER1_port.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.port";
                return false;
            }
            Bottle b_PROPERTIES_JOINTMAPPING_ENCODER1_position(b_PROPERTIES_JOINTMAPPING_ENCODER1.find("position").asString());
            if(b_PROPERTIES_JOINTMAPPING_ENCODER1_position.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER1.position";
                return false;
            }

            Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_type(b_PROPERTIES_JOINTMAPPING_ENCODER2.find("type").asString());
            if(b_PROPERTIES_JOINTMAPPING_ENCODER2_type.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.type";
                return false;
            }
            Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_port(b_PROPERTIES_JOINTMAPPING_ENCODER2.find("port").asString());
            if(b_PROPERTIES_JOINTMAPPING_ENCODER2_port.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.port";
                return false;
            }
            Bottle b_PROPERTIES_JOINTMAPPING_ENCODER2_position(b_PROPERTIES_JOINTMAPPING_ENCODER2.find("position").asString());
            if(b_PROPERTIES_JOINTMAPPING_ENCODER2_position.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTMAPPING.ENCODER2.position";
                return false;
            }

            // now the size of the vectors must all be equal

            int tmp = b_PROPERTIES_JOINTMAPPING_ACTUATOR_type.size();
            if( (tmp != b_PROPERTIES_JOINTMAPPING_ACTUATOR_port.size()) ||
                (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_type.size()) ||
                (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_port.size()) ||
                (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER1_position.size()) ||
                (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_type.size()) ||
                (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_port.size()) ||
                (tmp != b_PROPERTIES_JOINTMAPPING_ENCODER2_position.size())
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

            for(int i=0; i<mc_service.properties.numofjoints; i++)
            {
                servMC_actuator_t act;
                servMC_encoder_t enc1;
                servMC_encoder_t enc2;
                bool formaterror = false;

                // actuators ...

                act.type = eomc_act_unknown;
                act.location.local.place = servMC_actuator_place_none;
                act.location.local.boardConnector = servMC_board_connector_NONE;

                if(false == convert(b_PROPERTIES_JOINTMAPPING_ACTUATOR_type.get(i+1).asString(), act.type, formaterror))
                {
                    yError() << "ServiceParser::check_motion() actuator type not valid";
                    return false;
                }

                if(false == convert(b_PROPERTIES_JOINTMAPPING_ACTUATOR_port.get(i+1).asString(), act.location, formaterror))
                {
                     yError() << "ServiceParser::check_motion() actuator port not valid";
                    return false;
                }


                // encoder1s ...

                enc1.type = eomc_enc_none;
                enc1.location = {0}; // change it ...
                enc1.position = eomc_pos_none;

                if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER1_type.get(i+1).asString(), enc1.type, formaterror))
                {
                     yError() << "ServiceParser::check_motion() encoder type not valid";
                    return false;
                }
#warning TODO: parse encoder b_PROPERTIES_JOINTMAPPING_ENCODER1_port
                //VALE: commented in ordet to compile I have not enouth time to implement this function
        //         if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER1_port.get(i+1).asString(), enc1.location, formaterror))
        //         {
        //              yError() << "ServiceParser::check_motion() encoder port not valid";
        //             return false;
        //         }

                if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER1_position.get(i+1).asString(), enc1.position, formaterror))
                {
                     yError() << "ServiceParser::check_motion() encoder position not valid";
                    return false;
                }


                // encoder2s ...

                enc2.type = eomc_enc_none;
                enc2.location = {0}; // change it ...
                enc2.position = eomc_pos_none;

                if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER2_type.get(i+1).asString(), enc2.type, formaterror))
                {
                     yError() << "ServiceParser::check_motion() encoder type not valid";
                    return false;
                }
#warning TODO: parse encoder b_PROPERTIES_JOINTMAPPING_ENCODER2_port
                //VALE: commented in ordet to compile I have not enouth time to implement this function
        //         if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER2_port.get(i+1).asString(), enc2.location, formaterror))
        //         {
        //              yError() << "ServiceParser::check_motion() encoder port not valid";
        //             return false;
        //         }

                if(false == convert(b_PROPERTIES_JOINTMAPPING_ENCODER2_position.get(i+1).asString(), enc2.position, formaterror))
                {
                     yError() << "ServiceParser::check_motion() encoder position not valid";
                    return false;
                }

                mc_service.properties.actuators.push_back(act);
                mc_service.properties.encoder1s.push_back(enc1);
                mc_service.properties.encoder2s.push_back(enc2);

            }


        } // has_PROPERTIES_JOINTMAPPING




        if(true == has_PROPERTIES_JOINTSETS)
        {

            Bottle b_PROPERTIES_JOINTSETS_list = Bottle(b_PROPERTIES_JOINTSETS.find("list").asString());
            if(b_PROPERTIES_JOINTSETS_list.isNull())
            {
                yError() << "ServiceParser::check_motion() cannot find PROPERTIES.JOINTSETS.list";
                return false;
            }

            if((b_PROPERTIES_JOINTSETS_list.size()-1) != mc_service.properties.numofjoints)
            {
                yError() << "ServiceParser::check_motion() detected a wrong size of PROPERTIES.JOINTSETS.list: it must be equal to the number of joints" << mc_service.properties.numofjoints;
                return false;
            }

            mc_service.properties.joint2set.resize(0);

            for(int i=0; i<mc_service.properties.numofjoints; i++)
            {
                mc_service.properties.joint2set.push_back(b_PROPERTIES_JOINTSETS_list.get(i+1).asInt());
            }

        } // has_PROPERTIES_JOINTSETS

    }
    
    return true;
}


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


bool ServiceParser::parseService(Searchable &config, servConfigMC_t &mcconfig)
{
    bool ret = false;

    if(false == check_motion(config))
    {
        yError() << "ServiceParser::parseService() gets same errors parsing SERVICE MC group";
        return ret;
    }

    mcconfig.ethservice.configuration.type = mc_service.type;
    
    switch(mc_service.type)
    {
        case eomn_serv_MC_foc:
        {
            eOmn_serv_config_data_mc_foc_t *data_mc_foc = &(mcconfig.ethservice.configuration.data.mc.foc_based);

            // 1. ->boardtype4mccontroller
            data_mc_foc->boardtype4mccontroller = mc_service.properties.controller.type;

            // 2. ->version (of foc board).
            data_mc_foc->version.firmware.major = mc_service.properties.canboards.at(0).firmware.major;
            data_mc_foc->version.firmware.minor = mc_service.properties.canboards.at(0).firmware.minor;
            data_mc_foc->version.firmware.build = mc_service.properties.canboards.at(0).firmware.build;
            data_mc_foc->version.protocol.major = mc_service.properties.canboards.at(0).protocol.major;
            data_mc_foc->version.protocol.minor = mc_service.properties.canboards.at(0).protocol.minor;

            // 3. ->arrayofjomodescriptors
            EOarray *arrayofjomos = eo_array_New(4, sizeof(eOmc_jomo_descriptor_t), &data_mc_foc->arrayofjomodescriptors);
            int numofjomos = mc_service.properties.numofjoints;

            for(int i=0; i<numofjomos; i++)
            {
                if(eomc_act_foc != mc_service.properties.actuators[i].type)
                {
                    //VALE: remove fallowing two rows if you want to test the parse on example xml, where actuators are of different type
                    yError() << "ServiceParser::parseService() Actuator type  missmach with service type";
                    return false;
                }

                eOmc_jomo_descriptor_t jomodes = {0};

                // 1. actuator is on foc: we need the address
                jomodes.actuator.foc.canloc.port = mc_service.properties.actuators[i].location.oncan.port;
                jomodes.actuator.foc.canloc.addr = mc_service.properties.actuators[i].location.oncan.addr;
                //jomodes.actuator.foc.canloc.insideindex = mc_service.properties.actuators[i].location.oncan.index;
                jomodes.actuator.foc.canloc.insideindex = eobrd_caninsideindex_none;
            
                // 2. encoder1 is ...
                jomodes.encoder1.type = mc_service.properties.encoder1s[i].type;
                jomodes.encoder1.port = eobrd_port_none; // must copy in here the port. attention: different boards have different port values
                jomodes.encoder1.pos = eomc_pos_none; // must copy in here the pos.

                // 3. encoder2 is ...
                jomodes.encoder2.type = mc_service.properties.encoder2s[i].type;
                jomodes.encoder2.port = eobrd_port_none; // must copy in here the port. attention: different boards have different port values
                jomodes.encoder2.pos = eomc_pos_none; // must copy in here the pos.

                eo_array_PushBack(arrayofjomos, &jomodes);

            }


            // 4. ->jomocoupling
            eOmc_4jomo_coupling_t *jomocoupling = &data_mc_foc->jomocoupling;
            memset(jomocoupling, 0, sizeof(eOmc_4jomo_coupling_t));

            for(int i=0; i<numofjomos; i++)
            {
                jomocoupling->joint2set[i] = mc_service.properties.joint2set[i];
            }

            for(int i=0; i<4; i++)
            {
                for(int j=0; j<4; j++)
                {
                    jomocoupling->joint2motor[i][j] = eo_common_float_to_Q17_14(mc_service.properties.controller.matrixJ2M[4*i+j]);
                    jomocoupling->encoder2joint[i][j] = eo_common_float_to_Q17_14(mc_service.properties.controller.matrixE2J[4*i+j]);
                }
            }


            // ok, everything is done
            ret = true;

        } break;
        
        case eomn_serv_MC_mc4:
        {

        } break;
        
        case eomn_serv_MC_mc4plus:
        {

        } break;
        
        case eomn_serv_MC_mc4plusmais:
        {

        } break;

        case eomn_serv_MC_generic:
        {
            yError() << "ServiceParser::parseService() eomn_serv_MC_generic unused";
        } break;

        default:
        {
            yError() << "ServiceParser::parseService() unknown value in eOmn_serv_type_t field";
        } break;
    }
    

    return ret;
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










    s_serv_config_mc_v3_0B1.type       = eomn_serv_MC_mc4plus;
    //s_serv_config_mc_v3_0B1.filler     = {0, 0, 0};

    s_serv_config_mc_v3_0B1.data.mc.mc4plus_based.boardtype4mccontroller = 6; //emscontroller_board_HEAD_neckyaw_eyes;
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
