// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
* \author Lorenzo Natale
*
* Copyright (C) 2008 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* This file can be edited at src/iCubInterface2/main.cpp.
*/ 

#include "IRobotInterface.h"

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include "canIdDiscoverer/canIdDiscoverer.h"

#include <list>
#include <iostream>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Stamp.h>

class ServerGenericSensor: public yarp::os::RateThread
{
    yarp::dev::IGenericSensor *is;
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    std::string name;
	yarp::os::Stamp lastStateStamp;

public:
    ServerGenericSensor(const char *n, int rate=20): RateThread(rate)
    {
        is=0;
        name=std::string(n);
    }

    void attach(yarp::dev::IGenericSensor *s)
    {
        is=s;
    }

    bool threadInit()
    {
        if (port.open(name.c_str()))
            return true;
        else
            return false;
    }

    void threadRelease()
    {
        port.close();
    }

    void run()
    {
        yarp::sig::Vector &v=port.prepare();
        if (is!=0)
        {
            if (is->read(v))
				{
					lastStateStamp.update();
				}
			port.setEnvelope (lastStateStamp);
            port.write();
        }
    }
};

struct CanNetworkIdEntry
{
    ICUB_CAN_IDS ids;
    std::string deviceName;
};

class CanNetworkIdList
{
    typedef std::list<CanNetworkIdEntry *> IdList;
    typedef IdList::iterator IdListIt;

    IdList idList;

public:
    CanNetworkIdList(){}
    ~CanNetworkIdList()
    {
        IdListIt it=idList.begin();
        while(it!=idList.end())
        {
            delete (*it);
            it++;
        }
     }

    ICUB_CAN_IDS *find(const std::string &devicename)
    {
        IdListIt it=idList.begin();
        while(it!=idList.end())
        {
            if ((*it)->deviceName==devicename)
                return &((*it)->ids);
            it++;
        }
        return 0;
    }

    void push(const std::string &device, ICUB_CAN_IDS &canIds)
    {
        CanNetworkIdEntry *tmp = new CanNetworkIdEntry;
        tmp->deviceName=device;
        tmp->ids=canIds;
        idList.push_back(tmp);
    }
};

class ParallelCalibrator: public yarp::os::Thread
{
public:
    yarp::dev::IControlCalibration2 *iCalib;

    ParallelCalibrator()
    {
        iCalib=0;
    }

    void run()
    {
        fprintf(stderr, "Starting calibrator...", iCalib);
        if (iCalib!=0)
        {
            iCalib->calibrate();
            fprintf(stderr, "done\n");
        }
        else
            fprintf(stderr, "skipping, calibrator was not set\n");
    }
};

class ParallelParker: public yarp::os::Thread
{
public:
    yarp::dev::IControlCalibration2 *iCalib;

    ParallelParker()
    {
        iCalib=0;
    }

    void run()
    {
        fprintf(stderr, "Parking...");
        if (iCalib!=0)
        {
            iCalib->park();
            fprintf(stderr, "done\n");
        }
        else
            fprintf(stderr, "skipping, calibrator was not set\n");
    }
};


//
// only a container of the robot code on the main application
//

/*
*   Manage one network on the robot
*/
class RobotNetworkEntry
{
public:
    RobotNetworkEntry()
    { iCalib=0; iAnalog=0; iAnalogServer=0;}
    
    ~RobotNetworkEntry()
    {
        RobotNetworkEntry::close();
    }

    yarp::dev::PolyDriver driver;
    yarp::dev::PolyDriver calibrator;
    ParallelCalibrator calibThread;
    ParallelParker     parkerThread;
    std::string             id;

    yarp::dev::IControlCalibration2  *iCalib;
    yarp::dev::IGenericSensor        *iAnalog;
    ServerGenericSensor              *iAnalogServer;

    bool isValid()
    {
        return driver.isValid();
    }

    void close()
    {
        if (iAnalogServer)
            {
                iAnalogServer->stop();
                delete iAnalogServer;
                iAnalogServer=0;
            }

        iAnalog=0;

        if (isValid())
            driver.close();
    }

    void startPark()
    {
        std::cerr<<"startPark"<<std::endl;
        if (isValid()) {
            parkerThread.iCalib=iCalib;
            parkerThread.start();
        }
        std::cerr<<"done"<<std::endl;
    }

    void joinPark()
    {
        //  cerr<<"joinPark"<<endl;
        if (isValid())
            parkerThread.stop();
        //   cerr<<"done"<<endl;
    }

    void startCalib()
    {
        if (isValid()) {
            calibThread.iCalib=iCalib;
            calibThread.start();
        }
    }

    void joinCalib()
    {
        if (isValid())
            calibThread.stop();
    }

    void abortPark()
    {
        if (isValid())
        {
            if (iCalib!=0)
                iCalib->abortPark();
        }
    }

    void abortCalibration()
    {
        if (isValid())
        {
            if (iCalib!=0)
                iCalib->abortCalibration();
        }
    }
};

typedef std::list<RobotNetworkEntry *>::iterator RobotNetworkIt;

class RobotNetwork:public std::list<RobotNetworkEntry *>
{
public:
    RobotNetworkEntry *find(const std::string &id);
};

/*
*   Contains stuff to convert the robot as seen from the point of 
*   view of a collection of networks to a collection of limbs.
*/
class RobotPartEntry
{
public:
    RobotPartEntry();
    ~RobotPartEntry();

    bool open(yarp::os::Property &p);
    void close();

    yarp::dev::PolyDriver driver;
    yarp::dev::IMultipleWrapper *iwrapper;
    std::string         id;
    RobotNetwork        networks;

    bool push(RobotNetworkEntry *entry)
    {
        //should first verify if not duplicated
        networks.push_back(entry);
        return true;
    }
};

typedef std::list<RobotPartEntry *>::iterator RobotPartIt;

class RobotParts: public std::list<RobotPartEntry *>
{
public:
    RobotPartEntry *find(const std::string &pName);
};

class CartesianController
{
public:
    yarp::dev::PolyDriver driver;
    yarp::dev::IMultipleWrapper *iwrapper;

    void close()
    {
        if (iwrapper)
            iwrapper->detachAll();

        driver.close();
    }
    
    CartesianController()
    {
        iwrapper=0;
    }
    ~CartesianController()
    {
        close();
    }
};

typedef std::list<CartesianController *>::iterator CartesianControllersIt;
typedef std::list<CartesianController *> CartesianControllers;

class RobotInterfaceRemap: public IRobotInterface
{
protected:
    RobotNetwork  networks;
    RobotParts    parts;
    CartesianControllers cartesianControllers;

    bool initialized;
    bool isParking;
    bool isCalibrating;
    bool abortF;

    yarp::dev::PolyDriver gyro;
    yarp::dev::IGenericSensor *gyro_i;

    CanNetworkIdList can_ids;
    CanIdDiscoverer idDisc;
    bool automaticIds;

    std::string robotName;

public:
    // default constructor.
    RobotInterfaceRemap();
    ~RobotInterfaceRemap();

    /**
    * Initializes all robot devices. Depending on the
    * format of the configuration file call initialize10()
    * or initialize20().
    * @param full path to config file
    * @return true/false on success failure
    */
    bool initialize(const std::string &file);

    /**
    * Initialize a cartesian controller.
    */
    bool initCart(const std::string &file);

    /**
    * Finalize cartesian controllers
    */
    bool fnitCart();

    /**
    * Initializes all robot devices. Reads list of devices to be initialized 
    * in the file specfied in 'options'.
    * @param options contains name of config file
    * @return true/false on success failure
    */
    bool initialize10(const std::string &inifile);

    /**
    * Initializes all robot devices. Reads list of devices to be initialized 
    * in the file specfied in 'inifile'. New version for icub 2.0.
    */
    bool initialize20(const std::string &inifile);

    /**
    * Instantiate a device which controls a can network. 
    * @param parameters: parameters to be passed to the device
    * @param net: data structure to manage the network
    */
    bool instantiateNetwork(std::string &filename, 
        yarp::os::Property &robotOptions,
        RobotNetworkEntry &net);

    bool instantiateInertial(const std::string &path,
        yarp::os::Property &options);

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

    /**
    * Calibrate the robot. This function can be blocking or not depending on 
    * the value of the parameter wait.
    * @param wait if true the function blocks and returns only when parking is finished
    */
    void calibrate(bool wait=true);

    void abort();

protected:
    bool instantiateRightArm(yarp::os::Property& options);
    bool instantiateLeftArm(yarp::os::Property& options);
    bool instantiateHead(yarp::os::Property& options);
    bool instantiateInertial(yarp::os::Property& options);
    bool instantiateLegs(yarp::os::Property& options);

    bool forceNetworkId(yarp::os::Property& op, int n);
};

