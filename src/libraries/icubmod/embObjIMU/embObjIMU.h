/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __embObjIMU_h__
#define __embObjIMU_h__

#include <string>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ReturnValue.h>

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include "IethResource.h"



/**
 *  @ingroup icub_hardware_modules
 *  \defgroup analogSensorEth analogSensorEth
 *
 *
 * Driver for Ethernet communication with inertial sensors (accelerometers, gyroscopes)
 *        mounted on Strain2 board.
 *
 * It is possible to read from multiple strain2 board.
 * The parameters accepted in the config argument of the open method are described in https://icub-tech-iit.github.io/documentation/icub_robot_configuration/icub_robot_configuration_index/.
 */





namespace yarp {
    namespace dev {
        class embObjIMU;
    }
}

class yarp::dev::embObjIMU :            public DeviceDriver,
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
    virtual yarp::dev::ReturnValue getNrOfThreeAxisGyroscopes(size_t &n) const override;
    virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    virtual yarp::dev::ReturnValue getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    virtual yarp::dev::ReturnValue getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    virtual yarp::dev::ReturnValue getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    virtual yarp::dev::ReturnValue getNrOfThreeAxisLinearAccelerometers(size_t &n) const override;
    virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    virtual yarp::dev::ReturnValue getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    virtual yarp::dev::ReturnValue getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    virtual yarp::dev::ReturnValue getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    virtual yarp::dev::ReturnValue getNrOfThreeAxisMagnetometers(size_t &n) const override;
    virtual yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    virtual yarp::dev::ReturnValue getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;
    virtual yarp::dev::ReturnValue getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;
    virtual yarp::dev::ReturnValue getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    virtual yarp::dev::ReturnValue getNrOfOrientationSensors(size_t &n) const override;
    virtual yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    virtual yarp::dev::ReturnValue getOrientationSensorName(size_t sens_index, std::string &name) const override;
    virtual yarp::dev::ReturnValue getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual yarp::dev::ReturnValue getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;


    /* Iethresource methods */
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);


private:
    
    void *mPriv;
    
    std::string getBoardInfo(void) const;
    std::pair <size_t, eOas_sensor_t> getGyroSubIndex(size_t sens_index) const;
    std::pair <size_t, eOas_sensor_t> getAccSubIndex(size_t sens_index) const;
    void cleanup(void);
    
    //debug
    void updateDebugPrints(eOprotID32_t id32, double timestamp, void* rxdata);
    
};

#endif //__embObjIMU_h__

