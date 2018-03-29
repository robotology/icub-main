/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include </usr/local/src/robot/yarp-silvio/src/libYARP_dev/include/yarp/dev/MultipleAnalogSensorsInterfaces.h>
//#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>

#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>
#include <serviceParser.h>

#include "FeatureInterface.h"  

namespace yarp{
    namespace dev{
        class embObjIMU;
    }
}

#define DEFAULT_PERIOD 10   //ms

class yarp::dev::embObjIMU :  public DeviceDriver,
                            public IGenericSensor,
                            public yarp::dev::IPreciselyTimed,
                            public yarp::dev::IThreeAxisGyroscopes,
                            public yarp::dev::IThreeAxisLinearAccelerometers,
                            public yarp::dev::IThreeAxisMagnetometers,
                            public yarp::dev::IOrientationSensors,
                            public eth::IethResource
{
public:
    embObjIMU();
    ~embObjIMU();

    // Device Driver interface
    virtual bool open(yarp::os::Searchable &config) override;
    virtual bool close() override;

    // IGenericSensor interface.
    virtual bool read(yarp::sig::Vector &out) override;
    virtual bool getChannels(int *nc) override;
    virtual bool calibrate(int ch, double v) override;

    // IPreciselyTimed interface
    virtual yarp::os::Stamp getLastInputStamp() override;

    /* IThreeAxisGyroscopes methods */
    virtual size_t getNrOfThreeAxisGyroscopes() const override;
    virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    virtual bool getThreeAxisGyroscopeName(size_t sens_index, yarp::os::ConstString &name) const override;
    virtual bool getThreeAxisGyroscopeFrameName(size_t sens_index, yarp::os::ConstString &frameName) const override;
    virtual bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    virtual size_t getNrOfThreeAxisLinearAccelerometers() const override;
    virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    virtual bool getThreeAxisLinearAccelerometerName(size_t sens_index, yarp::os::ConstString &name) const override;
    virtual bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const override;
    virtual bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    virtual size_t getNrOfThreeAxisMagnetometers() const override;
    virtual yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    virtual bool getThreeAxisMagnetometerName(size_t sens_index, yarp::os::ConstString &name) const override;
    virtual bool getThreeAxisMagnetometerFrameName(size_t sens_index, yarp::os::ConstString &frameName) const override;
    virtual bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    virtual size_t getNrOfOrientationSensors() const override;
    virtual yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    virtual bool getOrientationSensorName(size_t sens_index, yarp::os::ConstString &name) const override;
    virtual bool getOrientationSensorFrameName(size_t sens_index, yarp::os::ConstString &frameName) const override;
    virtual bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;
    
    /* Iethresource */
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

    yarp::sig::Vector rpy, gravity;
    yarp::sig::Matrix dcm;
    yarp::sig::Vector accels;

private:
    std::string getBoardInfo(void);
    bool fromConfig(yarp::os::Searchable &config);
    void cleanup(void);
    bool sendConfing2board(void);
    bool initRegulars(void);
    
    eth::TheEthManager* ethManager;
    eth::AbstractEthResource* res;
    ServiceParser* parser;
    servConfigInertials3_t servCfg;
    bool opened;
    bool verbosewhenok;
    
    //da lasciare???
    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, yarp::os::ConstString &name) const;
    bool genericGetFrameName(size_t sens_index, yarp::os::ConstString &frameName) const;
    
    
    unsigned int nchannels;
    double dummy_value;
    yarp::os::Stamp lastStamp;
    yarp::os::ConstString m_sensorName;
    yarp::os::ConstString m_frameName;
};
