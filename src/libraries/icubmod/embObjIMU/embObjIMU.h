/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __embObjIMU_h__
#define __embObjIMU_h__

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>

#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>
#include <serviceParser.h>

#include "FeatureInterface.h"  
#include <mutex>
#include <stdexcept>


namespace yarp {
    namespace dev {
        class embObjIMU;
    }
}

class yarp::dev::embObjIMU :            public DeviceDriver,
                                        //public IGenericSensor,
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


    /* Iethresource methods */
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);


private:
    std::string getBoardInfo(void) const;
    bool fromConfig(yarp::os::Searchable &config);
    void cleanup(void);
    bool sendConfing2board(void);
    bool initRegulars(void);
    
    eth::TheEthManager* ethManager;
    eth::AbstractEthResource* res;
    ServiceParser* parser;
    servConfigImu_t servCfg;
    bool opened;
    bool verbosewhenok;
    mutable std::mutex mutex;
    typedef struct
    {
        std::string name;
        std::string framename;
        std::vector<int> values;
        yarp::dev::MAS_status state; 
        double timestamp;
    } sensorInfo_t;
    std::vector<std::vector<sensorInfo_t>> sensorsData;


    // data used for handling the received messsages
    std::uint8_t positionmap[eoas_sensors_numberof][eOcanports_number][16];

    bool buildmaps(void);
    bool getIndex(const eOas_inertial3_data_t *data, uint8_t &index, eOas_sensor_t &type);
    yarp::dev::MAS_status sensorState_eo2yarp(uint8_t eo_state);
    
    
    bool outOfRangeErrorHandler(const std::out_of_range& oor) const;
    
    /*private methos ti ge info of a sensor fiven index and type */
    size_t getNumOfSensors(eOas_sensor_t type) const;
    yarp::dev::MAS_status getSensorStatus(size_t sens_index, eOas_sensor_t type) const;
    bool getSensorName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &name) const;
    bool getSensorFrameName(size_t sens_index, eOas_sensor_t type, yarp::os::ConstString &frameName) const;
    bool getSensorMeasure(size_t sens_index, eOas_sensor_t type, yarp::sig::Vector& out, double& timestamp) const;

    //debug
    void updateDebugPrints(eOprotID32_t id32, double timestamp, void* rxdata);
    
};

#endif //__embObjIMU_h__

