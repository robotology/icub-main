#ifndef _ICUB_CLIENT_VEL_INTERFACE_
#define _ICUB_CLIENT_VEL_INTERFACE_

#include <stdio.h>
#include <string>
#include <string.h>
#include <list>
#include <iostream>
#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>

#include <ace/OS_NS_sys_stat.h>
#include <ace/OS_NS_netdb.h>
#include <ace/INET_Addr.h>
#include <ace/Sock_Connect.h>
#include <yarp/os/Time.h>

#define PI 3.14159265

#if 0
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardPid.h>

#include <yarp/os/impl/NameServer.h>
#include <yarp/os/impl/Logger.h>
#include <yarp/os/impl/PortCore.h>
#include <yarp/os/impl/SplitString.h>
#include <yarp/os/impl/NetType.h>
#include <yarp/os/impl/NameConfig.h>
#include <yarp/os/ManagedBytes.h>
#endif

// This class controls a single joint using the velocity controller.
// Ideally could be instatiated as many times as needed to control every joint of the robotpart
// indipendently and with different parameter... let see if this can be done easily

class VelCtrl
{
public:
    int joint;    // joint to be controlled
    int type;
    double amp;
    double time;
    double period;
    double home;

    int index;
    VelCtrl(int _joint);
    bool configure(int _type, double _amplitude, double _refresh, double _period, double _home);
    double compute();
    double sinusoidal(int index);
};

// This class is a collection of joints
class robotPart
{
public:
    yarp::os::ConstString robotName, partName;
    yarp::dev::PolyDriver robotPartDriver;

    // this port is used to echo the command sent to the robot through a yarp port for monitoring
    //yarp::os::BufferedPort<yarp::os::Bottle> out_port;
    yarp::os::Port out_port;
    yarp::os::Bottle msg;
    yarp::sig::Vector velocity;  //?

    yarp::dev::IPositionControl   *posControl;
    yarp::dev::IVelocityControl   *velControl;
    yarp::dev::IControlLimits     *limits;

    robotPart(void);
    void configurePart(const char *portName);

    void update(void);
    VelCtrl *control;
    bool configureJoint(int _j, int _type, double _amplitude, double _time, double _period);
    void homing();
};


class VCModule : public yarp::os::RFModule
{
public:
    explicit VCModule();
    virtual ~VCModule();

    double getPeriod();

    bool updateModule();
    bool open(yarp::os::Searchable& config);
    bool close();
    bool interruptModule();
    bool configure(yarp::os::ResourceFinder &rf);

private:
    double updatePeriod;
    robotPart part;
};
#endif
