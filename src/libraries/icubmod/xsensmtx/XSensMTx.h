// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Radu Bogdan Rusu, Alexis Maldonado
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __XSENSMTX__
#define __XSENSMTX__

#include "MTComm.h"
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IGenericSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <string>

namespace yarp{
    namespace dev{
        class XSensMTx;
    }
}

struct XSensMTxParameters
{
    std::string comPortString;
    short comPort;
};

/**
 * @ingroup icub_hardware_modules
 * @brief `xsensmtx` : driver for XSens's MTx IMU unit.
 * @author Radu Bogdan Rusu, Alexis Maldonado
 *
 * | YARP device name |
 * |:-----------------:|
 * | `xsensmtx` |
 */
class yarp::dev::XSensMTx : public yarp::dev::IGenericSensor,
                            public yarp::dev::IPreciselyTimed,
                            public DeviceDriver,
                            public yarp::dev::IThreeAxisGyroscopes,
                            public yarp::dev::IThreeAxisLinearAccelerometers,
                            public yarp::dev::IThreeAxisMagnetometers,
                            public yarp::dev::IOrientationSensors
{
public:
    XSensMTx();
    virtual ~XSensMTx();
    
    // IGenericSensor interface.
    bool read(yarp::sig::Vector &out) override;
    bool getChannels(int *nc) override;
    bool open(yarp::os::Searchable &config) override;
    bool calibrate(int ch, double v) override;
    bool close() override;

    yarp::os::Stamp getLastInputStamp() override;

    /* IThreeAxisGyroscopes methods */
    /**
     * Get the  number of three axis gyroscopes in the device
     * @return 1
     */
    size_t getNrOfThreeAxisGyroscopes() const override;

    /**
     * Get the status of three axis gyroscope
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;

    /**
     * Get the name of three axis gyroscope
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which three axis gyroscope measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get three axis gyroscope measurements
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] out 3D angular velocity measurement in deg/s
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    /**
     * Get the  number of three axis linear accelerometers in the device
     * @return 1
     */
    size_t getNrOfThreeAxisLinearAccelerometers() const override;

    /**
     * Get the status of three axis linear accelerometer
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;

    /**
     * Get the name of three axis linear accelerometer
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which three axis linear accelerometer measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get three axis linear accelerometer measurements
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] out 3D linear acceleration measurement in m/s^2
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    /**
     * Get the  number of three axis magnetometers in the device
     * @return 1
     */
    size_t getNrOfThreeAxisMagnetometers() const override;

    /**
     * Get the status of three axis magnetometer
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;

    /**
     * Get the name of three axis magnetometer
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    bool getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which three axis magnetometer measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get three axis magnetometer measurements
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] out 3D magnetometer measurement
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    /**
     * Get the  number of orientation sensors in the device
     * @return 1
     */
    size_t getNrOfOrientationSensors() const override;

    /**
     * Get the status of orientation sensor
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;

    /**
     * Get the name of orientation sensor
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    bool getOrientationSensorName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which orientation sensor measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    bool getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get orientation sensor measurements
     * @param[in] sens_index sensor index (must be 0 in the case BoschIMU)
     * @param[out] out RPY Euler angles in deg
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;

    // Open the device
    bool open(const XSensMTxParameters &par);

private:
    bool start();
    bool stop();

    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, std::string &name) const;
    bool genericGetFrameName(size_t sens_index, std::string &frameName) const;
    bool genericGetMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp, size_t startIdx) const;
    void *system_resources;
    int nchannels;
    std::string m_sensorName;
    std::string m_frameName;
    yarp::os::Stamp lastStamp;
};


#endif
