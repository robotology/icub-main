/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __embObjDevPrivData_h__
#define __embObjDevPrivData_h__

#include <string>
#include <yarp/os/LogStream.h>
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
    std::string deviceNameType;
    
    embObjDevPrivData(std::string name);
    ~embObjDevPrivData();
    
    inline  eth::AbstractEthResource* getEthRes()
    {return res;}
    
    inline  eth::TheEthManager* getEthManager()
    {return ethManager;}
    
    inline bool isOpen() {return behFlags.opened;}
    inline void setOpen(bool flag) {behFlags.opened=flag;}
    inline bool isVerbose() {return behFlags.verbosewhenok;}
    
    inline bool NOT_YET_IMPLEMENTED(const char *txt, const char *classname)
    {
        yWarning() << std::string(txt) << " not yet implemented for "<< std::string(classname) << "\n";
        return false;
    };
    
    std::string getBoardInfo(void) const; //This function need to be const
    
    bool prerareEthService(yarp::os::Searchable &config, eth::IethResource *interface);
    void cleanup(eth::IethResource *interface);

    bool serviceSetRegulars(eOmn_serv_category_t category, vector<eOprotID32_t> &id32vector, double timeout = 0.500);


};







#endif //__embObjDevPrivData_h__
