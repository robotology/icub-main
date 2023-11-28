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

#include "ethBoards.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


//#include <yarp/os/SystemClock.h>
//#include <yarp/os/Log.h>
//#include <yarp/os/LogStream.h>
//using yarp::os::Log;


// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------


// - class eth::EthBoards

eth::EthBoards::EthBoards()
{
    memset(LUT, 0, sizeof(LUT));
    sizeofLUT = 0;
}


eth::EthBoards::~EthBoards()
{
    memset(LUT, 0, sizeof(LUT));
    sizeofLUT = 0;
}


size_t eth::EthBoards::number_of_resources(void)
{
    return(sizeofLUT);
}

size_t eth::EthBoards::number_of_interfaces(eth::AbstractEthResource * res)
{
    if(NULL == res)
    {
        return(0);
    }

    eOipv4addr_t ipv4 = res->getProperties().ipv4addr;
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;


    if(index>=maxEthBoards)
    {
        return 0;
    }

    if(NULL == LUT[index].resource)
    {
        return 0;
    }

    return(LUT[index].numberofinterfaces);
}


bool eth::EthBoards::add(eth::AbstractEthResource* res)
{
    if(NULL == res)
    {
        return false;
    }

    eOipv4addr_t ipv4 = res->getProperties().ipv4addr;
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;

    if(index>=maxEthBoards)
    {
        //yError() << "eth::EthBoards::add() fails because index =" << index << "is >= maxEthBoards" << maxEthBoards;
        return false;
    }

    if(NULL != LUT[index].resource)
    {
        return false;
    }

    LUT[index].resource = res;
    LUT[index].ipv4 = ipv4;
    LUT[index].boardnumber = index;
    LUT[index].name = res->getProperties().boardnameString;
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<iethresType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT++;

    return true;
}


bool eth::EthBoards::add(eth::AbstractEthResource* res, eth::IethResource* interface)
{
    if((NULL == res) || (NULL == interface))
    {
        return false;
    }

    iethresType_t type = interface->type();

    if(iethres_none == type)
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getProperties().ipv4addr;
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    if(NULL != LUT[index].interfaces[type])
    {
        return false;
    }

    // ok, i add it
    LUT[index].interfaces[type] = interface;
    LUT[index].numberofinterfaces ++;


    return true;
}


bool eth::EthBoards::rem(eth::AbstractEthResource* res)
{
    if(NULL == res)
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getProperties().ipv4addr;
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    LUT[index].resource = NULL;
    LUT[index].ipv4 = 0;
    LUT[index].boardnumber = 0;
    LUT[index].name = "";
    LUT[index].numberofinterfaces = 0;
    for(int i=0; i<iethresType_numberof; i++)
    {
        LUT[index].interfaces[i] = NULL;
    }

    sizeofLUT--;

    return true;
}


bool eth::EthBoards::rem(eth::AbstractEthResource* res, iethresType_t type)
{
    if((NULL == res) || (iethres_none == type))
    {
        return false;
    }

    // now i compute the index
    eOipv4addr_t ipv4 = res->getProperties().ipv4addr;
    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return false;
    }

    if(res != LUT[index].resource)
    {
        return false;
    }

    if(NULL != LUT[index].interfaces[type])
    {
        LUT[index].interfaces[type] = NULL;
        LUT[index].numberofinterfaces --;
    }

    return true;
}


eth::AbstractEthResource* eth::EthBoards::get_resource(eOipv4addr_t ipv4)
{
    eth::AbstractEthResource * ret = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        ret = LUT[index].resource;
    }

    return(ret);
}

bool eth::EthBoards::get_LUTindex(eOipv4addr_t ipv4, uint8_t &index)
{
    index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;

    if(index>=maxEthBoards)
    {
        return false;
    }

    if(NULL == LUT[index].resource)
    {
        return false;
    }
    return true;
}

eth::IethResource* eth::EthBoards::get_interface(eOipv4addr_t ipv4, iethresType_t type)
{
    eth::IethResource *dev = NULL;
    uint8_t index;

    if(!get_LUTindex(ipv4, index))
    {
        return NULL;
    }

    if(iethres_none == type)
    {
        return NULL;
    }

    dev = LUT[index].interfaces[type];

    return dev;

}


eth::IethResource* eth::EthBoards::get_interface(eOipv4addr_t ipv4, eOprotID32_t id32)
{
    eth::IethResource *dev = NULL;

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index>=maxEthBoards)
    {
        return NULL;
    }

    if(NULL == LUT[index].resource)
    {
        return dev;
    }

    iethresType_t type = iethres_none;
    eOprotEndpoint_t ep = eoprot_ID2endpoint(id32);
    switch(ep)
    {
        case eoprot_endpoint_management:
        {
            type = iethres_management;
        } break;

        case eoprot_endpoint_motioncontrol:
        {
            type = iethres_motioncontrol;
        } break;

        case eoprot_endpoint_skin:
        {
            type = iethres_skin;
        } break;

        case eoprot_endpoint_analogsensors:
        {
            eOprotEntity_t en = eoprot_ID2entity(id32);
            if(eoprot_entity_as_strain == en)
                type = iethres_analogstrain;
            else if(eoprot_entity_as_mais == en)
                type = iethres_analogmais;
            else if(eoprot_entity_as_inertial3 == en)
                type = iethres_analoginertial3;
            else if(eoprot_entity_as_temperature == en)
                type = iethres_analogstrain; 
            else if(eoprot_entity_as_psc == en)
                type = iethres_analogpsc;
            else if(eoprot_entity_as_pos == en)
                type = iethres_analogpos;
            else if(eoprot_entity_as_ft == en)
                type = iethres_analogft;   
            else if(eoprot_entity_as_battery == en)
                type = iethres_analogbattery;                
            else
                type = iethres_none;
        } break;

        default:
        {
            type = iethres_none;
        } break;
    }

    if(iethres_none != type)
    {
        dev = LUT[index].interfaces[type];
    }



    return dev;
}


const string & eth::EthBoards::name(eOipv4addr_t ipv4)
{
    const string & ret = "none";

    uint8_t index = 0;
    eo_common_ipv4addr_to_decimal(ipv4, NULL, NULL, NULL, &index);
    index --;
    if(index<maxEthBoards)
    {
        return LUT[index].name;
    }
    else
    {
        return errorname[0]; // the last one contains an error string
    }

    return(ret);
}



bool eth::EthBoards::execute(void (*action)(eth::AbstractEthResource* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    for(int i=0; i<maxEthBoards; i++)
    {
        eth::AbstractEthResource* res = LUT[i].resource;
        if(NULL != res)
        {
            action(res, par);
        }

    }

    return(true);
}


bool eth::EthBoards::execute(eOipv4addr_t ipv4, void (*action)(eth::AbstractEthResource* res, void* p), void* par)
{
    if(NULL == action)
    {
        return(false);
    }

    eth::AbstractEthResource* res = get_resource(ipv4);

    if(NULL == res)
    {
        return(false);
    }

    action(res, par);

    return(true);
}



// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





