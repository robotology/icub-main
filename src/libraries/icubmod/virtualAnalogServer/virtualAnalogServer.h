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
        if (mpSensor) mpSensor->setTorque(mTorques);
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
    virtual bool detachAll()
    {
        mMutex.wait();

        for (int k=0; k<mNSubdevs; ++k)
        {
            mSubdevices[k].detach();
        }

        mMutex.post();
        return true;
    }
    //////////////////////////////////////////////////////////////////////////

protected:

    yarp::os::Semaphore mMutex;

    bool mIsVerbose;

    int mNChannels;
    int mNSubdevs;

    std::vector<int> mChan2Board;
    std::vector<int> mChan2BAddr;

    std::vector<AnalogSubDevice> mSubdevices;
    yarp::os::BufferedPort<yarp::sig::Vector> mPortInputTorques;
};






#endif // DOXYGEN_SHOULD_SKIP_THIS

#if 0
class VirtualAnalogServer : public DeviceDriver,
                            public Thread,
                            public IMultipleWrapper
{
private:
    bool spoke;
    bool verb;

    // Remapping support
    int               base;
    int               top;
    int               channels;

    std::vector<DevicesLutEntry> lut;                   // size is the number of joint, it says which subdevice that joint is handled by
    std::vector<AnalogSubDevice> subdevices;            // size is the number of devices, each handling one or more joints

    yarp::os::BufferedPort<yarp::sig::Vector> portInputTorques;


public:
    /**
    * Constructor.
    */
    VirtualAnalogServer();
    /**
    * Destructor.
    */
    virtual ~VirtualAnalogServer(){ close(); }

    /**
    * Return the value of the verbose flag.
    * @return the verbose flag.
    */
    bool verbose() const { return verb; }



    //////////////////////////////////////////////////////////////////////////
    // DeviceDriver
    /**
    * Open the device driver.
    * @param prop is a Searchable object which contains the parameters.
    * Allowed parameters are:
    * - verbose or v to print diagnostic information while running.
    * - subdevice to specify the name of the wrapped device.
    * - name to specify the predix of the port names.
    * - calibrator to specify the name of the calibrator object (created through a PolyDriver).
    * and all parameters required by the wrapped device driver.
    */
    virtual bool open(Searchable& config);
    /**
    * Close the device driver by deallocating all resources and closing ports.
    * @return true if successful or false otherwise.
    */
    virtual bool close();
    //
    //////////////////////////////////////////////////////////////////////////



    //////////////////////////////////////////////////////////////////////////
    // Thread
    virtual void run();    
    //
    //////////////////////////////////////////////////////////////////////////


    
    //////////////////////////////////////////////////////////////////////////
    // IMultipleWrapper
    virtual bool attachAll(const yarp::dev::PolyDriverList &p);

    virtual bool detachAll()
    {
        RateThread::stop();

        for(unsigned int k=0; k<subdevices.size(); ++k)
        {
        }

        return true;
    }
    //
    //////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////
    // IVirtualAnalogSensor
    virtual int configure(yarp::os::Searchable &config);
    virtual int setTorque(yarp::sig::Vector &torques);
    virtual int getChannels();
    //
    //////////////////////////////////////////////////////////////////////////
};


#endif
#endif
