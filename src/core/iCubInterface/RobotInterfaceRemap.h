// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
* \author Lorenzo Natale
*
* Copyright (C) 2010 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* This file can be edited at src/iCubInterface/main.cpp.
*/ 

#include "IRobotInterface.h"

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include "canIdDiscoverer/canIdDiscoverer.h"

#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/BufferedPort.h>

#ifdef _USE_INTERFACEGUI
#include <iCub/LoggerInterfaces.h>
#include <iCubInterfaceGuiServer.h>
#endif

using namespace std;

/**
  * Handler of the rpc port related to an analog sensor.
  * Manage the calibration command received on the rpc port.
 **/
class AnalogServerHandler: public yarp::os::PortReader
{
	yarp::dev::IAnalogSensor* is;	// analog sensor to calibrate, when required
	yarp::os::Port rpcPort;			// rpc port related to the analog sensor

public:
	AnalogServerHandler(const char* n){
		rpcPort.open(n);
		rpcPort.setReader(*this);
	}

	~AnalogServerHandler(){
		rpcPort.close();
		is = 0;
	}

	void setInterface(yarp::dev::IAnalogSensor *is){
		this->is = is;
	}

	bool _handleIAnalog(yarp::os::Bottle &cmd, yarp::os::Bottle &reply)
    {
        if (is==0)
            return false;

        int msgsize=cmd.size();

        int code=cmd.get(1).asVocab();
        switch (code)
        {
        case VOCAB_CALIBRATE:
            if (msgsize==2)
                is->calibrateSensor();
            else
            {
                //read Vector of values and pass to is->calibrate();
            }
            return true;
            break;
        case VOCAB_CALIBRATE_CHANNEL:
            if (msgsize==3)
            {
                int ch=cmd.get(2).asInt();
                is->calibrateChannel(ch);
            }
            if (msgsize==4)
            {
                int ch=cmd.get(2).asInt();
                double v=cmd.get(3).asDouble();
                is->calibrateChannel(ch, v);
            }

            return true;
            break;
        default:
            return false;
        }
    }

    virtual bool read(yarp::os::ConnectionReader& connection) {
        yarp::os::Bottle in;
        yarp::os::Bottle out;
        bool ok=in.read(connection);
        if (!ok) return false;

        // parse in, prepare out
        int code = in.get(0).asVocab();
        bool ret=false;
        if (code==VOCAB_IANALOG)
        {
            ret=_handleIAnalog(in, out);
        }

        if (!ret)
        {
            out.clear();
            out.addVocab(VOCAB_FAILED);
        }

        yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
        if (returnToSender!=NULL) {
            out.write(*returnToSender);
        }
        return true;
    }

};


/**
  * A yarp port that output data read from an analog sensor.
  * It contains information about which data of the analog sensor are sent
  * on the port, i.e. an offset and a length.
  */
struct AnalogPortEntry
{
	yarp::os::BufferedPort<yarp::sig::Vector> port;
	std::string port_name;		// the complete name of the port
    int offset;					// an offset, the port is mapped starting from this taxel
    int length;					// length of the output vector of the port (-1 for max length)
	AnalogPortEntry(){}
	AnalogPortEntry(const AnalogPortEntry &alt){
		this->length = alt.length;
		this->offset = alt.offset;
		this->port_name = alt.port_name;
	}	
	AnalogPortEntry &operator =(const AnalogPortEntry &alt){ 
		this->length = alt.length;
		this->offset = alt.offset;
		this->port_name = alt.port_name;
		return *this;
	}
};

/**
  * It reads the data from an analog sensor and sends them on one or more ports.
  * It creates one rpc port and its related handler for every output port.
  */
class AnalogServer: public yarp::os::RateThread
{
    yarp::dev::IAnalogSensor *is;				// the analog sensor to read from
    std::vector<AnalogPortEntry> analogPorts;	// the list of output ports
	std::vector<AnalogServerHandler*> handlers;	// the list of rpc port handlers
	yarp::os::Stamp lastStateStamp;				// the last reading time stamp

	void setHandlers(){
		for(unsigned int i=0;i<analogPorts.size(); i++){
			std::string rpcPortName = analogPorts[i].port_name;
			rpcPortName += "/rpc:i";
			AnalogServerHandler* ash = new AnalogServerHandler(rpcPortName.c_str());
			handlers.push_back(ash);
		}
	}

public:
	// Constructor used when there is only one output port
	AnalogServer(const char* name, int rate=20): RateThread(rate)
    {
        is=0;
        analogPorts.resize(1);
		analogPorts[0].offset = 0;
		analogPorts[0].length = -1;	// max length
		analogPorts[0].port_name = std::string(name);
		setHandlers();
    }

	// Contructor used when one or more output ports are specified
	AnalogServer(const std::vector<AnalogPortEntry>& _analogPorts, int rate=20): RateThread(rate)
    {
        is=0;
        this->analogPorts=_analogPorts;
        setHandlers();
    }

    ~AnalogServer()
    {
        threadRelease();
        is=0;
    }

	/**
	  * Specify which analog sensor this thread has to read from.
	  */
    void attach(yarp::dev::IAnalogSensor *s)
    {
        is=s;
		for(unsigned int i=0;i<analogPorts.size(); i++){
			handlers[i]->setInterface(is);
		}
    }    


	bool threadInit()
    {        
		for(unsigned int i=0; i<analogPorts.size(); i++){
			// open data port
			if (!analogPorts[i].port.open(analogPorts[i].port_name.c_str()))
				return false;
		}
		return true;
    }

    void threadRelease()
    {
		for(unsigned int i=0; i<analogPorts.size(); i++){
			analogPorts[i].port.close();
		}
    }

    void run()
    {
		int first, last, ret;
        if (is!=0)
        {
			// read from the analog sensor
            yarp::sig::Vector v;
            ret=is->read(v);
            
            if (ret==yarp::dev::IAnalogSensor::AS_OK)
            {
                if (v.size()>0)
                {
                    lastStateStamp.update();
                    // send the data on the port(s), splitting them as specified in the config file
                    for(unsigned int i=0; i<analogPorts.size(); i++){
                        yarp::sig::Vector &pv = analogPorts[i].port.prepare();
                        first = analogPorts[i].offset;
                        if(analogPorts[i].length==-1)	// read the max length available
                            last = v.size()-1;
                        else
                            last = analogPorts[i].offset + analogPorts[i].length - 1;
                        // check vector limit
                        if(last>=(int)v.size()){
                            cerr<<"Error while sending analog sensor output on port "<< analogPorts[i].port_name<< endl;
                            cerr<<"Vector size expected to be at least "<<last<<" whereas it is "<< v.size()<< endl;
                            continue;
                        }
                        pv = v.subVector(first, last);
                        analogPorts[i].port.setEnvelope(lastStateStamp);
                        analogPorts[i].port.write();
                    }
				}
            }
            else
            {
                //todo release
            }

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
    { iCalib=0; }
    
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
    std::list<yarp::dev::IAnalogSensor *> analogSensors;
    std::list<AnalogServer  *> analogServers;

    bool isValid()
    {
        return driver.isValid();
    }

    void close()
    {
        std::list<AnalogServer *>::iterator it=analogServers.begin();

        while(it!=analogServers.end())
        {
            if ( (*it) )
            {
                (*it)->stop();
                delete (*it);
                (*it)=0;
            }
            it++;
        }

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

class SkinPartEntry
{
private:
    AnalogServer *analogServer;
    yarp::dev::IAnalogSensor *analog;

public:
    SkinPartEntry();
    ~SkinPartEntry();

    bool open(yarp::os::Property &deviceP, yarp::os::Property &partP);
    void close();
	void calibrate();

    void setId(const std::string &i)
    { id=i; }

    std::string id;
    yarp::dev::PolyDriver driver;
};

class SkinParts: public std::list<SkinPartEntry *>
{
public:
    SkinPartEntry *find(const std::string &pName);
    void close();
};

typedef SkinParts::iterator SkinPartsIt;

class RobotInterfaceRemap: public IRobotInterface
{
protected:
    RobotNetwork  networks;
    RobotParts    parts;
    CartesianControllers cartesianControllers;

    SkinParts     skinparts;

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

    #ifdef _USE_INTERFACEGUI
    iCubInterfaceGuiServer *mServerLogger;
    #endif

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
    bool finiCart();

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
    * Park the robot. This function can be blocking or not depending on 
    * the value of the parameter wait.
    * @param wait if true the function blocks and returns only when parking is finished
    */
    void park(bool wait=true);

    bool detachWrappers();
    bool closeNetworks();

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

