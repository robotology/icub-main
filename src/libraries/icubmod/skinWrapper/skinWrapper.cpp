/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <yarp/os/Thread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Os.h>
#include <iCub/FactoryInterface.h>

#include "skinWrapper.h"

using namespace yarp::sig;
using namespace yarp::os;

skinWrapper::skinWrapper()
{
    yTrace(); 
    multipleWrapper=NULL;
    analog=NULL;
		setId("undefinedPartName");
}

skinWrapper::~skinWrapper() { }

void skinWrapper::calibrate()
{
    if (analog)
    {
        analog->calibrateSensor();
    }
}

bool skinWrapper::open(yarp::os::Searchable &params)
{
    bool correct=true;
		
    if(params.check("ports"))
    {
        Bottle *ports=params.find("ports").asList();
        setId(ports->get(0).asString().c_str());
        numPorts=ports->size();
    }
    // Verify minimum set of parameters required
    if(!params.check("robotName") )
    {
        correct=false;
        yError() << "skinWrapper: missing robot Name, check your configuration file!! Quitting\n";
        return false;
    }

    if (params.check("period"))
    {
        period=params.find("period").asInt();
    }
    else
    {
        period=20;
        yDebug() <<"skinWrapper Warning: part "<<id<<" using default period ("<<period<<")\n";
    }

    // Read the list of ports
    int total_taxels=params.find("total_taxels").asInt();
    std::string robotName=params.find("robotName").asString().c_str();
    std::string root_name;
    root_name+="/";
    root_name+=robotName;
    root_name+="/skin";


    Property option(params.toString().c_str());
    option.put("name",root_name);
    option.unput("device");
    option.put("device","analogServer");
    option.put("channels",total_taxels);
    option.unput("total_taxels");
    if(!driver.open(option))
    {
        yError()<<"skinWrapper: unable to open the device";
        return false;
    }
    if(!driver.isValid())
    {
        yError()<<"skinWrapper: invalid device";
        return false;
    }
    return true;
}

bool skinWrapper::close()
{
    if (NULL != analog)
        analog=0;

    if(driver.isValid())
        driver.close();
    return true;
}

// implementare i metodi attach e detach come un vero wrapper che si rispetti!!!! 
bool skinWrapper::attachAll(const yarp::dev::PolyDriverList &skinDev)
{
    yTrace() ;
    if (skinDev.size() != 1)
    {
        std::cerr<<"skinWrapper: cannot attach more than one device\n";
        return false;
    }

    yarp::dev::PolyDriver * subdevice=skinDev[0]->poly;

    if (subdevice->isValid())
    {
        subdevice->view(analog);
    }
    else
    {
        yError() << "skinWrapper: subdevice passed to attach method is invalid!!!";
        return false;
    }
    if(NULL == analog)
    {
        yError() << "skinWrapper: The analog sensor is not correctly instantiated, cannot attach !!!";
        return false;
    }
    if(driver.isValid())
    {
        driver.view(multipleWrapper);
    }
    else
    {
        yError()<<"skinWrapper: cannot open analog server for skin";
        return false;
    }
    if(multipleWrapper == YARP_NULLPTR)
    {
        yError()<<"skinWrapper: cannot call attach function";
        return false;
    }
    multipleWrapper->attachAll(skinDev);
    return true;
}

bool skinWrapper::detachAll()
{
    yTrace();
    multipleWrapper->detachAll();
//    analogServer->stop();
    return true;
}


