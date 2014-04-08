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

skinWrapper::skinWrapper()
{
    yTrace(); 
    analogServer=NULL;
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

bool skinWrapper::open(yarp::os::Searchable &inputParams)
{
    yTrace() << "skinWrapper param = " << inputParams.toString().c_str();

    Property params;
    params.fromString(inputParams.toString().c_str());
    bool correct=true;
		
		if(params.check("ports"))
		{
			Bottle *ports=params.find("ports").asList();
			setId(ports->get(0).asString().c_str());
		}
    // Verify minimum set of parameters required
    if(!params.check("robotName") )
    {
        correct=false;
        yError() << "SkinWrapper missing robot Name, check your configuration file!! Quitting\n";
        return false;
    }

    if (params.check("period"))
    {
        period=params.find("period").asInt();
    }
    else
    {
        period=20;
        yDebug() <<"SkinWrapper Warning: part "<<id<<" using default period ("<<period<<")\n";
    }

/*  // Open the device -- no necessary, the factory will do the job, add an attach method for getting the IAnalogInterface from the sensor
    
		std::string devicename=params.find("device").asString().c_str();
    params.put("device", devicename.c_str());

    std::string canbusdevice=params.find("canbusdevice").asString().c_str();
    params.put("canbusdevice", canbusdevice.c_str());

    driver.open(params);
    if (!driver.isValid())
        return false;

    driver.view(analog);

    RateThread *thread;
    driver.view(thread);

    if (!analog)
    {
        std::cerr<<"Error: part "<<id<<" device " << devicename << " does not implement analog interface"<<endl;
        driver.close();
        return false;
    }
*/
    // Read the list of ports
    std::string robotName=params.find("robotName").asString().c_str();
    std::string root_name;
    root_name+="/";
    root_name+=robotName;
    root_name+="/skin/";

    std::vector<AnalogPortEntry> skinPorts;  // temporary, actual ports are owned by the analogserver
		// port names are optional, do not check for correctness.
    if(!params.check("ports"))
		{
        // if there is no "ports" section take the name of the "skin" group as the only port name
        skinPorts.resize( (size_t) 1);
        skinPorts[0].offset = 0;
        skinPorts[0].length = -1;
        skinPorts[0].port_name = root_name + this->id;
    }
    else
		{
        Bottle *ports=params.find("ports").asList();

        if (!params.check("total_taxels", "number of taxels of the part"))
            return false;
        int total_taxels=params.find("total_taxels").asInt();
        int nports=ports->size();
        int totalT = 0;
        skinPorts.resize(nports);

        for(int k=0;k<ports->size();k++)
        {
            Bottle parameters=params.findGroup(ports->get(k).asString().c_str());

            if (parameters.size()!=5)
            {
                yError () <<"check skin port parameters in part description";
                yError() << "--> I was expecting "<<ports->get(k).asString().c_str() << " followed by four integers";
                return false;
            }

            int wBase=parameters.get(1).asInt();
            int wTop=parameters.get(2).asInt();
            int base=parameters.get(3).asInt();
            int top=parameters.get(4).asInt();

            yDebug() << "SkinWrapper part "<< id << " mapping port/taxels--> "<< ports->get(k).asString().c_str() << ": "<< wBase<<" "<<wTop<<" "<<base<<" "<<top;

            //check consistenty
            if(wTop-wBase != top-base){
                yError() <<"Error: check skin port parameters in part description";
                yError() <<"Numbers of mapped taxels do not match.";
                return false;
            }
            int taxels=top-base+1;

            skinPorts[k].length = taxels;
            skinPorts[k].offset = wBase;
            skinPorts[k].port_name = root_name+string(ports->get(k).asString().c_str());

            totalT+=taxels;
        }

        if (totalT!=total_taxels)
        {
            yError() <<"Error total number of mapped taxels does not correspond to total taxels";
            return false;
        }
    }

    // If everything is ok create analog server, for now with period 0 (disabled I guess)
    analogServer = new yarp::dev::AnalogServer(skinPorts);
    analogServer->setRate(0);
    return true;
}

bool skinWrapper::close()
{
    if (NULL != analogServer)
    {
        delete analogServer;
    }
    if (NULL != analog)
        analog=0;

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

    // Check if both analogServer and analogSensor are ok before doing attach.
    if( NULL == analogServer)
    {
        yError() << "skinWrapper: analogServer already attached!!!";
        return false;
    }

    if(NULL == analog)
    {
        yError() << "skinWrapper: The analog sensor is not correctly instantiated, cannot attach !!!";
        return false;
    }

    // if we got this far every conditions are ok
//     analogServer = new yarp::dev::AnalogServer(skinPorts);
    analogServer->setRate(period);
    analogServer->attach(analog);
    analogServer->start();
    return true;
}

bool skinWrapper::detachAll()
{
    yTrace();
    analogServer->stop();
    return true;
}


