/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include <iCubSimulationIMU.h>

#include "OdeInit.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


iCubSimulationIMU::iCubSimulationIMU()
{
    gyro.resize(3,0.0);
    rpy.resize(3,0.0);
    accels.resize(3, 0.0);
    magn.resize(3,0.0);
    m_sensorName = "sensorName";
    m_frameName  = "frameName";
}

iCubSimulationIMU::~iCubSimulationIMU()
{
    OdeInit& odeinit = OdeInit::get();
    lock_guard<mutex> lck(odeinit.mtx);
    odeinit.removeSimulationIMU();
}

bool iCubSimulationIMU::open(yarp::os::Searchable &config)
{
    OdeInit& odeinit = OdeInit::get();
    lock_guard<mutex> lck(odeinit.mtx);
    odeinit.setSimulationIMU(this);
    return true;
}

bool iCubSimulationIMU::close()
{
    // TODO see what does simulation control
    return true;
}



yarp::dev::MAS_status iCubSimulationIMU::genericGetStatus(size_t sens_index) const
{
    if (sens_index!=0) {
        return yarp::dev::MAS_status::MAS_ERROR;
    }

    return yarp::dev::MAS_status::MAS_OK;
}

bool iCubSimulationIMU::genericGetSensorName(size_t sens_index, std::string &name) const
{
    if (sens_index!=0) {
        return false;
    }

    name = m_sensorName;
    return true;
}

bool iCubSimulationIMU::genericGetFrameName(size_t sens_index, std::string &frameName) const
{
    if (sens_index!=0) {
        return false;
    }

    frameName = m_frameName;
    return true;
}

size_t iCubSimulationIMU::getNrOfThreeAxisGyroscopes() const
{
    return 1;
}

yarp::dev::MAS_status iCubSimulationIMU::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool iCubSimulationIMU::getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool iCubSimulationIMU::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool iCubSimulationIMU::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

    out.resize(3);
    m_mutex.lock();
    out = gyro;
    m_mutex.unlock();

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

    return true;
}

size_t iCubSimulationIMU::getNrOfThreeAxisLinearAccelerometers() const
{
    return 1;
}

yarp::dev::MAS_status iCubSimulationIMU::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool iCubSimulationIMU::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool iCubSimulationIMU::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool iCubSimulationIMU::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

    out.resize(3);
    m_mutex.lock();
    out = accels;
    m_mutex.unlock();

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

    return true;
}

size_t iCubSimulationIMU::getNrOfThreeAxisMagnetometers() const
{
    return 1;
}

yarp::dev::MAS_status iCubSimulationIMU::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool iCubSimulationIMU::getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool iCubSimulationIMU::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool iCubSimulationIMU::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

    out.resize(3);
    m_mutex.lock();
    out = magn;
    m_mutex.unlock();

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

    return true;
}

size_t iCubSimulationIMU::getNrOfOrientationSensors() const
{
    return 1;
}

yarp::dev::MAS_status iCubSimulationIMU::getOrientationSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool iCubSimulationIMU::getOrientationSensorName(size_t sens_index, std::string &name) const
{
    return genericGetSensorName(sens_index, name);
}

bool iCubSimulationIMU::getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool iCubSimulationIMU::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy_out, double& timestamp) const
{
    if (sens_index!=0) {
        return false;
    }

    rpy_out.resize(3);
    m_mutex.lock();
    rpy_out = rpy;
    m_mutex.unlock();

    // Workaround for https://github.com/robotology/yarp/issues/1610
    yarp::os::Stamp copyStamp(lastStamp);
    timestamp = copyStamp.getTime();

    return true;
}


void iCubSimulationIMU::updateIMUData(const yarp::os::Bottle& imuData) {
    m_mutex.lock();
    if (imuData.size() < 12) {
        return;
    }
    rpy[0] = imuData.get(0).asFloat64();
    rpy[1] = imuData.get(1).asFloat64();
    rpy[2] = imuData.get(2).asFloat64();

    accels[0] = imuData.get(3).asFloat64();
    accels[1] = imuData.get(4).asFloat64();
    accels[2] = imuData.get(5).asFloat64();

    gyro[0] = imuData.get(6).asFloat64();
    gyro[1] = imuData.get(7).asFloat64();
    gyro[2] = imuData.get(8).asFloat64();

    magn[0] = imuData.get(9).asFloat64();
    magn[1] = imuData.get(10).asFloat64();
    magn[2] = imuData.get(11).asFloat64();

    m_mutex.unlock();
}
