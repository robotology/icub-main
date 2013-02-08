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
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
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

/* the control command message type
* head is a Bottle which contains the specification of the message type
* body is a Vector which move the robot accordingly
*/
typedef PortablePair<Bottle, Vector> CommandMessage;

class VirtualAnalogServer;

class AnalogSubDevice
{
public:
    std::string id;
    int base;
    int top;
    int myChannels;

    bool configuredF;
    bool attachedF;

    PolyDriver            *device;
    IPreciselyTimed       *iTimed;
    IVirtualAnalogSensor  *sensor;
    
    yarp::sig::Vector encoders;
    yarp::sig::Vector encodersTimes;

    AnalogSubDevice();

    bool attach(PolyDriver *d, const std::string &id);
    void detach();

    bool configure(int base, int top, int channels, const std::string &id);

    bool isAttached()
    { return attachedF; }
};

struct DevicesLutEntry
{
    int offset; //an offset, the device is mapped starting from this joint
    int deviceEntry; //index to the joint corresponding subdevice in the list
};


/**
* Callback implementation after buffered input.
*/
class ImplementCallbackHelper2 : public TypedReaderCallback<CommandMessage> {
protected:
    IVirtualAnalogSensor  *IVAnalog;
    int controlledAxes;

public:
    /**
    * Constructor.
    * @param x is the instance of the container class using the callback.
    */
    ImplementCallbackHelper2(VirtualAnalogServer *x);

    /**
    * Callback function.
    * @param v is the Vector being received.
    */
    virtual void onRead(CommandMessage& v);

    bool initialize();
};


#endif // DOXYGEN_SHOULD_SKIP_THIS

/*
* A modified version of the network wrapper. Similar
* to the the network wrapper in YARP, but it
* maps only a subpart of the underlying device.
* Allows also deferred attach/detach of a subdevice.
*/
class VirtualAnalogServer : public DeviceDriver,
                            public RateThread,
                            public IMultipleWrapper,
                            public IPreciselyTimed,
                            public IVirtualAnalogSensor
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

    inline AnalogSubDevice *getSubdevice(unsigned int i);

    Stamp time;                           // envelope to attach to the state port
    Semaphore timeMutex;

//     rpc port stuff   // Useful??
//     Port rpc_p;     // RPC to configure the robot
//     CommandsHelper2 command_reader;
//     PortReaderBuffer<CommandMessage> control_buffer;


    Port inputStreamingData_p; // in port to command the robot
    PortReaderBuffer<Bottle> inputStreamingData_buffer;
    ImplementCallbackHelper2 inputStreamingData_callback;

    // Useful?

//     Vector            encoders;

//     //utility
//     int               thread_period;
//     std::string       partName;
// 



public:
    /**
    * Constructor.
    */
    VirtualAnalogServer();
    virtual ~VirtualAnalogServer()  { close();  }

    /**
    * Return the value of the verbose flag.
    * @return the verbose flag.
    */
    bool verbose() const { return verb; }

    /**
    * Default open() method.
    * @return always false since initialization requires parameters.
    */
    virtual bool open() { return false; }

    /**
    * Close the device driver by deallocating all resources and closing ports.
    * @return true if successful or false otherwise.
    */
    virtual bool close();


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
    virtual bool open(Searchable& prop);

    virtual bool detachAll()
    {
        RateThread::stop();

        int devices=device.subdevices.size();
        for(int k=0;k<devices;k++)
            device.getSubdevice(k)->detach();

        return true;
    }

    virtual bool attachAll(const yarp::dev::PolyDriverList &l);

};

#endif
