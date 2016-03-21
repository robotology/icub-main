// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __embObjInertials_h__
#define __embObjInertials_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <iCub/DebugInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>
#include <hostTransceiver.hpp>
#include <ethResource.h>
#include <ethManager.h>


#include "FeatureInterface.h"  

#include "IethResource.h"

#include <yarp/os/LogStream.h>


namespace yarp {
    namespace dev {
        class embObjInertials;
        class TheEthManager;
    }
}



typedef int AnalogDataFormat;
/*! class yarp::dev::embObjInertials
 * 
 */
class yarp::dev::embObjInertials:       public yarp::dev::IAnalogSensor,
                                        public yarp::dev::DeviceDriver,
                                        public IethResource
{

public:

    enum { inertials_Channels = 6, inertials_FormatData = 16, inertials_maxNumber = eoas_inertial_pos_max_numberof+1 };

public:

    embObjInertials();
    ~embObjInertials();

    // An open function yarp factory compatible
    bool open(yarp::os::Searchable &config);
    bool close();

    // IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int ch);

    // IethResource interface
    virtual bool initialised();
    virtual iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

private:

    char boardIPstring[20];

    TheEthManager* ethManager;
    EthResource* res;

    bool opened;
    bool verbosewhenok;

    unsigned int    counterSat;
    unsigned int    counterError;
    unsigned int    counterTimeout;

    ////////////////////
    // parameters
    int _period;
    int _numofsensors;

    yarp::os::Semaphore mutex;

    vector<double> analogdata;

    uint8_t _fromInertialPos2DataIndexAccelerometers[eoas_inertial_pos_max_numberof];
    uint8_t _fromInertialPos2DataIndexGyroscopes[eoas_inertial_pos_max_numberof];

    short status;
    double timeStamp;

private:

    // for all
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool fromConfig(yarp::os::Searchable &config);
    bool initRegulars();
    void cleanup(void);
    // not used ...
    bool isEpManagedByBoard();


    // for inertials
    bool configServiceInertials(Searchable& globalConfig);
    bool sendConfig2MTBboards(yarp::os::Searchable &config);
    eOas_inertial_position_t getLocationOfInertialSensor(yarp::os::ConstString &strpos);


    // for ??
    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);
};


#endif

