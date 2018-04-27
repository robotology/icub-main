/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __embObjDevPrivData_h__
#define __embObjDevPrivData_h__


#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>
#include <ethManager.h>
#include <abstractEthResource.h>


namespace yarp {
    namespace dev {
        class embObjDevPrivData;
    }
}

class yarp::dev::embObjDevPrivData
{
public:
    eth::TheEthManager* ethManager;
    eth::AbstractEthResource* res;
    struct behFlags
    {
        bool opened;
        bool verbosewhenok;
    }behFlags;
    
    embObjDevPrivData();
    ~embObjDevPrivData();
    
    inline  eth::AbstractEthResource* getEthRes()
    {return res;}
    
    inline  eth::TheEthManager* getEthManager()
    {return ethManager;}
    
    inline bool isOpen() {return behFlags.opened;}
    inline void setOpen(bool flag) {behFlags.opened=flag;}
    inline bool isVerbose() {return behFlags.verbosewhenok;}
    
    std::string getBoardInfo(void) const; //This function need to be const
    
    inline bool NOT_YET_IMPLEMENTED(const char *txt, const char *classname)
    {
        yWarning() << std::string(txt) << " not yet implemented for "<< std::string(classname) << "\n";
        return false;
    }
};

yarp::dev::embObjDevPrivData::embObjDevPrivData()
{
    ethManager = nullptr;
    res = nullptr;
    behFlags.opened = false;
    ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        behFlags.verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        behFlags.verbosewhenok = false;
    }
    
}
yarp::dev::embObjDevPrivData::~embObjDevPrivData()
{;}

std::string yarp::dev::embObjDevPrivData::getBoardInfo(void) const
{
    if(nullptr == res)
    {
        return " BOARD name_unknown (IP unknown) ";
    }
    else
    {
        return ("BOARD " + res->getProperties().boardnameString +  " (IP "  + res->getProperties().ipv4addrString + ") ");
    }
}

#endif //__embObjDevPrivData_h__
