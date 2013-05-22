// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef __VIRTUALANALOGSERVER__
#define __VIRTUALANALOGSERVER__

/*
* Copyright (C) 2013 RobotCub Consortium
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// VirtualAnalogServer
// A server that opens an input port getting externally measured analog values
// and it is able to attach to a one or more virtual analog sensor through
// IVirtualAnalogSensor interface.

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/Wrapper.h>

#include <yarp/dev/IVirtualAnalogSensor.h>

#include <string>
#include <vector>

#include <Debug.h>

#include <stdarg.h>

#ifdef MSVC
    #pragma warning(disable:4355)
#endif

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class VirtualAnalogServer;

class AnalogSubDevice
{
public:
    AnalogSubDevice();
   ~AnalogSubDevice();

    bool attach(PolyDriver *driver, const std::string &key);
    void detach();

    bool configure(int map0, int map1, const std::string &key);
   
    bool isAttached(){ return mIsAttached; }

    void setTorque(int joint,double torque)
    {
        if (joint<mMap0 || mMap1<joint) return;

        mTorques[joint-mMap0]=torque;
    }

    void flushTorques()
    {
        if (mpSensor) mpSensor->updateMeasure(mTorques);
    }

    const std::string& getKey(){ return mKey; }

protected:
    std::string mKey;
    
    int mMap0,mMap1; 

    yarp::sig::Vector mTorques;

    bool mIsConfigured;
    bool mIsAttached;
    
    PolyDriver            *mpDevice;
    IVirtualAnalogSensor  *mpSensor;
};



///////////////////////////////////////////////////

class VirtualAnalogServer : public DeviceDriver, public Thread, public IMultipleWrapper
{
public:
    VirtualAnalogServer() : mMutex(1)
    {
        mIsVerbose=false;
        mNChannels=0;
        mNSubdevs=0;
    }

    ~VirtualAnalogServer()
    {
        close();
    }

    // DeviceDriver //////////////////////////////////////////////////////////
    virtual bool open(Searchable& config);
    virtual bool close();
    //////////////////////////////////////////////////////////////////////////

    // Thread ////////////////////////////////////////////////////////////////
    virtual void run();    
    //////////////////////////////////////////////////////////////////////////

    // IMultipleWrapper //////////////////////////////////////////////////////
    virtual bool attachAll(const yarp::dev::PolyDriverList &p);
    virtual bool detachAll();
    //////////////////////////////////////////////////////////////////////////

protected:

    yarp::os::Semaphore mMutex;

    bool mIsVerbose;

    int mNChannels;
    int mNSubdevs;

    std::vector<int> mChan2Board;
    std::vector<int> mChan2BAddr;

    std::vector<AnalogSubDevice> mSubdevices;
    yarp::os::BufferedPort<yarp::os::Bottle> mPortInputTorques;
};

#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif
