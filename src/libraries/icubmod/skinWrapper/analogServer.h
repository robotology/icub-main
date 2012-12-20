 
 #ifndef ANALOG_SERVER_H_
 #define ANALOG_SERVER_H_
 
 //#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
 
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>

#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <iCub/FactoryInterface.h>

#include "Debug.h"

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
    AnalogServerHandler(const char* n);
    ~AnalogServerHandler();

    void setInterface(yarp::dev::IAnalogSensor *is);

    bool _handleIAnalog(yarp::os::Bottle &cmd, yarp::os::Bottle &reply);

    virtual bool read(yarp::os::ConnectionReader& connection);
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
    AnalogPortEntry();
    AnalogPortEntry(const AnalogPortEntry &alt);
    AnalogPortEntry &operator =(const AnalogPortEntry &alt);
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

    void setHandlers();

public:
    // Constructor used when there is only one output port
    AnalogServer(const char* name, int rate=20);

    // Contructor used when one or more output ports are specified
    AnalogServer(const std::vector<AnalogPortEntry>& _analogPorts, int rate=20);

    ~AnalogServer();

    /**
      * Specify which analog sensor this thread has to read from.
      */
    void attach(yarp::dev::IAnalogSensor *s);
    bool threadInit();
    void threadRelease();
    void run();
};

#endif