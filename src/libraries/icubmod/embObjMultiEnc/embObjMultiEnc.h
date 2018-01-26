// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __embObjMultiEnc_h__
#define __embObjMultiEnc_h__

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
        class embObjMultiEnc;
    }
}


#define EMBOBJMULTIENC_USESERVICEPARSER


// -- class embObjMais

class yarp::dev::embObjMultiEnc:        public yarp::dev::IAnalogSensor,
                                        public yarp::dev::DeviceDriver,
                                        public eth::IethResource
{

public:

    enum{default_numofencperjoint = eOmc_joint_multienc_maxnum, default_numofjoints = 2};

public:

    embObjMultiEnc();
    ~embObjMultiEnc();

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
    
    uint8_t numofjoints;
    uint8_t numofencperjoint;

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
    servConfigMais_t serviceConfig;

    yarp::os::Semaphore mutex;

    vector<double> analogdata;
    vector<double> encoderConversionFactor;

    short status;
    double timeStamp;
    vector<int> listofjoints;

private:

    // for all
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool fromConfig(yarp::os::Searchable &config);
    bool initRegulars();
    void cleanup(void);
    void printServiceConfig(void);



    // for ??
    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);
};


#endif

