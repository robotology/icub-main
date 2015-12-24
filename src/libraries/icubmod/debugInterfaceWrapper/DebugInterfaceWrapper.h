// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef __DEBUG_INTERFACE_WRAPPER__
#define __DEBUG_INTERFACE_WRAPPER__

/*
* Copyright (C) 2014 RobotCub Consortium
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// DebugInterfaceWrapper
// A small wrapper for devices that implements the
// iCub debugInterface
//

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/DebugInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <vector>

#include <RpcMsgHandler.h>
#include <WrappedDevice.h>

#ifdef MSVC
#pragma warning(disable:4355)
#endif



#ifndef DOXYGEN_SHOULD_SKIP_THIS


/* the control command message type
* head is a Bottle which contains the specification of the message type
* body is a Vector which move the robot accordingly
*/
typedef yarp::os::PortablePair<yarp::os::Bottle, yarp::sig::Vector> CommandMessage;

#endif // DOXYGEN_SHOULD_SKIP_THIS


class DebugInterfaceWrapper:    public yarp::dev::DeviceDriver,
                                public yarp::dev::IDebugInterface,
                                public yarp::dev::IMultipleWrapper
{
private:
    bool spoke;
    bool _verbose;

    iCub::Wrapper::iDebug::WrappedDevice device;

    yarp::os::Port                                  rpc_p;              // RPC port receiving the commands
    yarp::os::PortReaderBuffer<yarp::os::Bottle>    rpc_buffer;         // Buffer storing new commands
    iCub::Wrapper::iDebug::RpcMsgHandler            rpcMsgHandler;      // Attach a callback function to the rpc port


    yarp::os::Stamp time;     // envelope to attach to the state port
    yarp::os::Semaphore timeMutex;

    // Config
    std::string       partName;
    int               controlledJoints;


    // Default usage
    // Open the wrapper only, the attach method needs to be called before using it
    bool openDeferredAttach(yarp::os::Property& prop);

    // For the simulator, if a subdevice parameter is given to the wrapper, it will
    // open it and and attach to it immediatly.
    yarp::dev::PolyDriver *subDeviceOwned;
    bool openAndAttachSubDevice(yarp::os::Property& prop);

public:
    bool getAxes(int &controlledJoints);


    /**
    * Constructor.
    */
    DebugInterfaceWrapper();

    virtual ~DebugInterfaceWrapper() {
        close();
    }

    /**
    * Return the value of the verbose flag.
    * @return the verbose flag.
    */
    bool verbose() const { return _verbose; }

    /**
    * Default open() method.
    * @return always false since initialization requires parameters.
    */
    virtual bool open() {  return false; }

    /**
    * Close the device driver by deallocating all resources and closing ports.
    * @return true if successful or false otherwise.
    */
    virtual bool close();


    /**
    * Open the device driver.
    * @param prop is a Searchable object which contains the parameters.
    * Allowed parameters are:
    * - verbose or v to print diagnostic information while running..
    * - name to specify the prefix of the port names.
    * - subdevice [optional] if specified, the openAndAttachSubDevice will be
    *             called, otherwise openDeferredAttach is called.
    * and all parameters required by the wrapper.
    */
    virtual bool open(yarp::os::Searchable& prop);

    virtual bool detachAll();

    virtual bool attachAll(const yarp::dev::PolyDriverList &l);

    //------------------------------\\
    //       Debug Interface        \\
    //------------------------------\\

    /* Set a generic parameter (for debug)
     * @param type is the CAN code representing the command message
     * @return true/false on success/failure
     */
    virtual bool setParameter(int j, unsigned int type, double value);

    /* Get a generic parameter (for debug)
     * @param type is the CAN code representing the command message
     * @return true/false on success/failure
     */
    virtual bool getParameter(int j, unsigned int type, double* value);

    /* Set a generic parameter (for debug)
     * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    virtual bool setDebugParameter(int j, unsigned int index, double value);

    /* Get a generic parameter (for debug)
     * @param index is the number of the debug parameter
     * @return true/false on success/failure
     */
    virtual bool getDebugParameter(int j, unsigned int index, double* value);
};

#endif  // DEBUG_INTERFACE_WRAPPER
