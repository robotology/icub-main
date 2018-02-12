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

#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>


#include <yarp/os/LogStream.h>

#include "serviceParser.h"


namespace yarp {
    namespace dev {
        class embObjStrain;
    }
}


#define EMBOBJSTRAIN_USESERVICEPARSER

// -- class embObjStrain

class yarp::dev::embObjStrain:      public yarp::dev::IAnalogSensor,
                                    public yarp::dev::DeviceDriver,
                                    public eth::IethResource
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
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

private:

    string boardIPstring;
    string boardName;
    eOipv4addr_t ipv4addr;

    eth::TheEthManager* ethManager;
    eth::AbstractEthResource* res;
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
    vector<double> offset;
    vector<double> scaleFactor;

    bool scaleFactorIsFilled;

    short status;
    double timeStamp;

private:

    // for all
    bool fromConfig(yarp::os::Searchable &config);
    bool initRegulars();
    void cleanup(void);
    void printServiceConfig(void);

    // for strain
    bool fillScaleFactor();
    bool sendConfig2Strain(void);

    // for ??
    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);
};


#endif

