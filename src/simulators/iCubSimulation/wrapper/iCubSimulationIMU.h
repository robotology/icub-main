/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <mutex>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IGenericSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>



#ifndef __iCubSimulationIMUh__
#define __iCubSimulationIMUh__

class iCubSimulationIMU :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IThreeAxisGyroscopes,
        public yarp::dev::IThreeAxisLinearAccelerometers,
        public yarp::dev::IThreeAxisMagnetometers,
        public yarp::dev::IOrientationSensors
{
public:
    iCubSimulationIMU();
    virtual ~iCubSimulationIMU();

    // Device Driver interface
    bool open(yarp::os::Searchable &config) override;
    bool close() override;

    /* IThreeAxisGyroscopes methods */
    size_t getNrOfThreeAxisGyroscopes() const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    size_t getNrOfThreeAxisLinearAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    size_t getNrOfThreeAxisMagnetometers() const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    bool getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;
    bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;
    bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    size_t getNrOfOrientationSensors() const override;
    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    bool getOrientationSensorName(size_t sens_index, std::string &name) const override;
    bool getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;
    bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;

    void updateIMUData(const yarp::os::Bottle& imuData);

    yarp::sig::Vector gyro, rpy, magn, accels;

private:
    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, std::string &name) const;
    bool genericGetFrameName(size_t sens_index, std::string &frameName) const;

    yarp::os::Stamp lastStamp;
    std::string m_sensorName;
    std::string m_frameName;
    mutable std::mutex m_mutex;

};

#endif // __iCubSimulationIMUh__