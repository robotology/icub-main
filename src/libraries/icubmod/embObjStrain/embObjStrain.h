// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __embObjStrain_h__
#define __embObjStrain_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
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
#include "EoAnalogSensors.h"

#include "IethResource.h"

#include <yarp/os/LogStream.h>

#include "serviceParser.h"


namespace yarp {
    namespace dev {
        class embObjStrain;
        class TheEthManager;
    }
}


#define EMBOBJSTRAIN_USESERVICEPARSER

// -- class embObjStrain

class yarp::dev::embObjStrain:      public yarp::dev::IAnalogSensor,
                                    public yarp::dev::DeviceDriver,
                                    public IethResource
{

public:

    enum { strain_Channels = 6, strain_FormatData = 16 };

public:

    embObjStrain();
    ~embObjStrain();

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
    ServiceParser* parser;

    bool opened;
    bool verbosewhenok;

    unsigned int    counterSat;
    unsigned int    counterError;
    unsigned int    counterTimeout;

    ////////////////////
    // parameters
    servConfigStrain_t serviceConfig;


    yarp::os::Semaphore mutex;

    vector<double> analogdata;
    vector<double> scaleFactor;

    bool scaleFactorIsFilled;

    short status;
    double timeStamp;

private:

    // for all
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool fromConfig(yarp::os::Searchable &config);
    bool initRegulars();
    void cleanup(void);
    void printServiceConfig(void);
    // not used ...
    bool isEpManagedByBoard();

    // for strain
    bool fillScaleFactor();
    bool sendConfig2Strain(void);

    // for ??
    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);
};


#endif

