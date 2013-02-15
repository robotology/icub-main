// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Alberto Cardellino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "virtualAnalogServer.h"

#include <iostream>

using namespace std;

AnalogSubDevice::AnalogSubDevice()
{
    detach();
}

AnalogSubDevice::~AnalogSubDevice()
{
    detach();
}

bool AnalogSubDevice::configure(int map0, int map1, const std::string &key)
{
    mIsConfigured=false;

    if (map1<map0)
    {
        cerr<<"check configuration file top<base."<<endl;
        return false;
    }

    mMap0=map0;
    mMap1=map1;

    mKey=key;

    mTorques.resize(mMap1-mMap0+1);

    mIsConfigured=true;

    return true;
}

bool AnalogSubDevice::attach(yarp::dev::PolyDriver *device, const std::string &key)
{
    if (key!=mKey)
    {
        cerr << "Wrong device sorry." << endl;
        return false;
    }

    //configure first
    if (!mIsConfigured)
    {
        cerr << "You need to call configure before you can attach any device" << endl;
        return false;
    }

    if (!device)
    {
        cerr << "Invalid device (null pointer)" << endl;
        return false;
    }

    mpDevice=device;

    if (mpDevice->isValid())
    {
        mpDevice->view(mpSensor);
    }
    else
    {
        cerr << "Invalid device (isValid() returned false" << endl;
        return false;
    }

    if (mpSensor)
    {
        mIsAttached=true;
        return true;
    }

    return false;
}

void AnalogSubDevice::detach()
{
    mMap0=mMap1=-1;

    mpDevice=NULL;
    mpSensor=NULL;

    mIsConfigured=false;
    mIsAttached=false;
}

bool VirtualAnalogServer::open(Searchable& config)
{
    cout << config.toString().c_str() << endl << endl;

    mIsVerbose = (config.check("verbose","if present, give detailed output"));
 
    if (mIsVerbose) cout << "running with verbose output\n";

    //thus thread period is useful for output port... this input port has callback so maybe can skip it (?)
    //thread_period = prop.check("threadrate", 20, "thread rate in ms. for streaming encoder data").asInt();

    std::cout << "Using VirtualAnalogServer\n";

    if (!config.check("networks", "list of networks merged by this wrapper"))
    {
        return false;
    }

    if (!config.check("channels", "number of channels "))
    {
        return false;
    }

    mNChannels=config.find("channels").asInt();

    Bottle *networks=config.find("networks").asList();
    mNSubdevs=networks->size();
    
    mChan2Board.resize(mNChannels);
    mChan2BAddr.resize(mNChannels);
    
    mSubdevices.resize(mNSubdevs);

    int totalJ=0;

    for (unsigned int k=0; k<networks->size(); ++k)
    {
        Bottle parameters=config.findGroup(networks->get(k).asString().c_str());

        if (parameters.size()!=5)    // mapping joints using the paradigm: part from - to / network from - to
        {
            cerr << "Error: check network parameters in part description" << endl;
            cerr << "--> I was expecting " << networks->get(k).asString().c_str() << " followed by four integers" << endl;
            return false;
        }

        int map0=parameters.get(1).asInt();
        int map1=parameters.get(2).asInt();
        
        int map2=parameters.get(3).asInt();
        int map3=parameters.get(4).asInt();

        for (int j=map0; j<=map1; ++j)
        {
            mChan2Board[j]=k;
            mChan2BAddr[j]=j-map0+map2;
        }

        if (!mSubdevices[k].configure(map2,map3,networks->get(k).asString().c_str()))
        {
            cerr << "configure of subdevice ret false" << endl;
            return false;
        }

        totalJ+=map1-map0+1;
    }

    if (totalJ!=mNChannels)
    {
        cerr << "Error total number of mapped channels does not correspond to part channels" << endl;
        return false;
    }

    std::string portName="/";
    portName+=config.check("name",Value("controlboard"),"prefix for port names").asString().c_str();
    portName+="/torque:i";

    if (!mPortInputTorques.open(portName.c_str()))
    {
        cerr << "can't open port " << portName.c_str() << endl;
        return false;
    }

    return true;
}

bool VirtualAnalogServer::close()
{
    Thread::stop();

    mPortInputTorques.interrupt();
    mPortInputTorques.close();
}

bool VirtualAnalogServer::attachAll(const PolyDriverList &polylist)
{
    mMutex.wait();

    for (int p=0; p<polylist.size(); ++p)
    {
        std::string key=polylist[p]->key.c_str();

        // find appropriate entry in list of subdevices and attach
        for (unsigned int k=0; k<mSubdevices.size(); ++k)
        {    
            if (mSubdevices[k].getKey()==key)
            {
                if (!mSubdevices[k].attach(polylist[p]->poly,key))
                {
                    mMutex.post();
                    return false;
                }
            }
        }
    }

    //check if all devices are attached to the driver
    for (unsigned int k=0; k<mSubdevices.size(); ++k)
    {
        if (!mSubdevices[k].isAttached())
        {
            mMutex.post();
            return false;
        }
   }

    //Thread::start();

    mMutex.post();
    return true;
}

void VirtualAnalogServer::run()
{
    yarp::sig::Vector *pTorques;
    
    while (Thread::isRunning())
    {
        pTorques=mPortInputTorques.read();

        if (pTorques)
        {
            mMutex.wait();

            if (pTorques->length()==mNChannels)
            {
                for (int i=0; i<mNChannels; ++i)
                {
                    mSubdevices[mChan2Board[i]].setTorque(mChan2BAddr[i],(*pTorques)[i]);
                }
            }

            for (int d=0; d<mNSubdevs; ++d)
            {
                mSubdevices[d].flushTorques();
            }

            mMutex.post();
        }
        else
        {
            yarp::os::Time::delay(0.5);
        }
    }
}
