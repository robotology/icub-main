/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include "IethResource.h"

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>


namespace yarp {
    namespace dev {
        class embObjMultipleFTsensor;
    }
}


#define EMBOBJSTRAIN_USESERVICEPARSER

// -- class embObjMultipleFTsensor

class yarp::dev::embObjMultipleFTsensor:    public yarp::dev::DeviceDriver,
                                    public eth::IethResource,
                                    public yarp::dev::IAnalogSensor,
                                    public yarp::dev::ITemperatureSensors,
                                    public yarp::dev::ISixAxisForceTorqueSensors
{
public:

    embObjMultipleFTsensor();
    ~embObjMultipleFTsensor();

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
    
    // ITemperatureSensors
    virtual size_t getNrOfTemperatureSensors() const override;
    virtual yarp::dev::MAS_status getTemperatureSensorStatus(size_t sens_index) const override;
    virtual bool getTemperatureSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getTemperatureSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const override;
    virtual bool getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;
    
    // ISixAxisForceTorqueSensors
    virtual size_t getNrOfSixAxisForceTorqueSensors() const override;
    virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const override;
    virtual bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;


private:
    void *mPriv;

    void cleanup(void);
    std::string getBoardInfo(void) const;
    bool updateStrainValues(eOprotID32_t id32, double timestamp, void* rxdata);
    bool updateTemperatureValues(eOprotID32_t id32, double timestamp, void* rxdata);

    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);
    bool enableTemperatureTransmission(bool enable);
};

