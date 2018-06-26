// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __embObjInertials_h__
#define __embObjInertials_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/PeriodicThread.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>

#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>


#include "FeatureInterface.h"  
#include "EoAnalogSensors.h"


#include <yarp/os/LogStream.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

namespace yarp {
    namespace dev {
        class embObjInertials;
    }
}

#include "serviceParser.h"


#define EMBOBJINERTIALS_PUBLISH_OLDSTYLE


// -- class embObjInertials
class yarp::dev::embObjInertials:       public yarp::dev::IAnalogSensor,
                                        public yarp::dev::DeviceDriver,
                                        public yarp::dev::IThreeAxisGyroscopes,
                                        public yarp::dev::IThreeAxisLinearAccelerometers,
                                        public eth::IethResource
{

public:


#if defined(EMBOBJINERTIALS_PUBLISH_OLDSTYLE)
    // 6 channels because we have (pos, type, t, x, y, z)
    enum { inertials_Channels = 6, inertials_FormatData = 16, inertials_maxNumber = eOas_inertials_maxnumber };
#else
    // 4 channels because we have (t, x, y, z)
    enum { inertials_Channels = 4, inertials_FormatData = 16, inertials_maxNumber = eOas_inertials_maxnumber };
#endif

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
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);


    /* IThreeAxisGyroscopes methods */
    virtual size_t getNrOfThreeAxisGyroscopes() const override;
    virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    virtual bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    virtual size_t getNrOfThreeAxisLinearAccelerometers() const override;
    virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    virtual bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;


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
    servConfigInertials_t serviceConfig;

    mutable yarp::os::Semaphore mutex;

    vector<double> analogdata;
    vector<uint16_t> gyrSensors;
    vector<uint16_t> accSensors;


    short status;
    double timeStamp;

private:

    // for all
    bool fromConfig(yarp::os::Searchable &config);
    bool initRegulars();
    void cleanup(void);
    void printServiceConfig(void);

    // for inertials
    bool sendConfig2MTBboards(yarp::os::Searchable &config);


    // for ??
    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);

};


#endif

