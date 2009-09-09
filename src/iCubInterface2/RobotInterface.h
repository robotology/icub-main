// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "IRobotInterface.h"

#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
 
#include <yarp/String.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include "./canIdDiscoverer/canIdDiscoverer.h"

//
// only a container of the robot code on the main application
//
 
class RobotInterface: public IRobotInterface
{
protected:
    bool initialized;
    bool isParking;
    bool isCalibrating;
    bool abortF;

    yarp::dev::PolyDriver headCalib;
    yarp::dev::PolyDriver rarmCalib;
    yarp::dev::PolyDriver larmCalib;
    yarp::dev::PolyDriver legsCalib;

    yarp::dev::PolyDriver head;
    yarp::dev::IPositionControl *head_ipc;
    yarp::dev::IAmplifierControl *head_amp;
    yarp::dev::IControlCalibration2 *head_cal;

    yarp::dev::PolyDriver gyro;
	yarp::dev::IGenericSensor *gyro_i;

    yarp::dev::PolyDriver rarm;
    yarp::dev::IPositionControl *rarm_ipc;
    yarp::dev::IAmplifierControl *rarm_amp;
    yarp::dev::IControlCalibration2 *rarm_cal;
	
    yarp::dev::PolyDriver larm;
    yarp::dev::IPositionControl *larm_ipc;
    yarp::dev::IAmplifierControl *larm_amp;
    yarp::dev::IControlCalibration2 *larm_cal;

    yarp::dev::PolyDriver legs;
    yarp::dev::IPositionControl *legs_ipc;
    yarp::dev::IAmplifierControl *legs_amp;
    yarp::dev::IControlCalibration2 *legs_cal;

    bool hasRightArm;
    bool hasLeftArm;
    bool hasLegs;
    bool hasHead;

    //////////////////// DEBUG
    yarp::dev::IVelocityControl *head_ivc;
    yarp::dev::IPidControl *head_pid;
    ////////////////////////

    ICUB_CAN_IDS can_ids;
    CanIdDiscoverer idDisc;

public:
    // default constructor.
    RobotInterface();

    /**
    * Initializes all robot devices. Reads list of devices to be initialized 
    * in the file specfied in 'file'.
    * @param full path to config file
    * @return true/false on success failure
    */
    bool initialize(const std::string &file);
 
    /**
    * Closes all robot devices.
    */
    bool finalize();

    /**
    * Park the robot. This function can be blocking or not depending on 
    * the value of the parameter wait.
    * @param wait if true the function blocks and returns only when parking is finished
    */
    void park(bool wait=true);

    void abort();

protected:
    bool instantiateRightArm(yarp::os::Property& options);
    bool instantiateLeftArm(yarp::os::Property& options);
    bool instantiateHead(yarp::os::Property& options);
    bool instantiateInertial(yarp::os::Property& options);
    bool instantiateLegs(yarp::os::Property& options);

    bool forceNetworkId(yarp::os::Property& op, int n);
};
