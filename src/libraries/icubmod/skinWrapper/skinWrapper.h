/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef SKINWRAPPER_H_
#define SKINWRAPPER_H_


//#include "RobotInterfaceRemap.h"
//#include "extractPath.h"

#include <string>
#include <sstream>

#include <yarp/os/Thread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Os.h>

//#include "ControlBoardWrapper.h"
//#include "ControlBoardWrapper2.h"

#include <iCub/FactoryInterface.h>

// Logger interface
#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

#include "Debug.h"

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

//#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <yarp/sig/Vector.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/BufferedPort.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

/**
  * Handler of the rpc port related to an analog sensor.
  * Manage the calibration command received on the rpc port.
 **/
class AnalogServerHandler: public yarp::os::PortReader
{
    yarp::dev::IAnalogSensor* is;   // analog sensor to calibrate, when required
    yarp::os::Port rpcPort;         // rpc port related to the analog sensor

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
    std::string port_name;      // the complete name of the port
    int offset;                 // an offset, the port is mapped starting from this taxel
    int length;                 // length of the output vector of the port (-1 for max length)
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
    yarp::dev::IAnalogSensor *is;               // the analog sensor to read from
    std::vector<AnalogPortEntry> analogPorts;   // the list of output ports
    std::vector<AnalogServerHandler*> handlers; // the list of rpc port handlers
    yarp::os::Stamp lastStateStamp;             // the last reading time stamp

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
        analogPorts[0].length = -1; // max length
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
                        if(analogPorts[i].length==-1)   // read the max length available
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


class skinWrapper : public yarp::dev::DeviceDriver
{
private:
// to be removed ?   -> if change the analog sensorto be a device and the analog server to be a wrapper??
    AnalogServer *analogServer;
    yarp::dev::IAnalogSensor *analog;

//    yarp::sig::Vector wholeData;      // may be useful if one the skin wrapper has to get data from more than one device...

public:
    std::string id;
    yarp::dev::PolyDriver driver;

    skinWrapper();
    ~skinWrapper();

//    bool open(yarp::os::Property &params, yarp::os::Property &params);
    bool open(yarp::os::Searchable &params);
    bool close();
    void calibrate();
    Vector * getData();

    void setId(const std::string &i)
    {
        id=i;
    }
};


//class SkinParts : public std::vector<skinWrapper*>
//{
//public:
//    skinWrapper *find(const std::string &pName);
//    void close();
//};
//
//typedef SkinParts::iterator SkinPartsIt;


#endif /* SKINWRAPPER_H_ */
