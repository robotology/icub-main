#ifndef __DEBUG_INTERFACE_WRAPPED_DEVICE__
#define __DEBUG_INTERFACE_WRAPPED_DEVICE__

/*
 * An helper that stores configurations for mapping the axis
 * in case the wrappers does not match exactly with the wrapped
 * device.
 */

#include <iostream>

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

namespace iCub{
    namespace Wrapper{
        namespace iDebug{
            class SubDevice;
            class WrappedDevice;
            struct DevicesLutEntry;
        }
    }
}


class iCub::Wrapper::iDebug::SubDevice
{
public:
    std::string id_subDev;
    int base_subDev;
    int top_subDev;
    int axes_subDev;

    bool configured;
    bool attached;

    yarp::dev::PolyDriver               *subdevice;
    yarp::dev::IDebugInterface          *iDbg_subDev;

    // to get axis number for runtime sanity check
    yarp::dev::IPositionControl         *iPos_subDev;

    SubDevice();

    bool attach(yarp::dev::PolyDriver *d, const std::string &id_subDev);
    void detach();

    bool configure(int base_subDev, int top_subDev, int axes_subDev, const std::string &id_subDev);

    bool isAttached()
    { return attached; }

};

typedef std::vector<iCub::Wrapper::iDebug::SubDevice> SubDeviceVector;

struct iCub::Wrapper::iDebug::DevicesLutEntry
{
    int deviceJoint;    // the joint number relative to the corresponding subdevice
    int deviceEntry;    // index if subdevice owning the joint
};


class iCub::Wrapper::iDebug::WrappedDevice
{
public:
    SubDeviceVector subdevices;
    std::vector<DevicesLutEntry> lut;

    inline SubDevice *getSubdevice(unsigned int i)
    {
        if (i>=subdevices.size())
            return 0;

        return &subdevices[i];
    }
};





#endif // __DEBUG_INTERFACE_WRAPPED_DEVICE__

