/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

/**
 *  @ingroup icub_hardware_modules
 *  \defgroup analogSensorEth analogSensorEth
 *
 *
 * Driver for Ethernet communication with Force-Torque sensors, the Strain2 board.
 * It is also possible read the temperature of tthe sensor.
 *
 * The parameters accepted in the config argument of the open method are described in http://wiki.icub.org/wiki/Robot_configuration.
 */




#ifndef __embObjFTsensor_h__
#define __embObjFTsensor_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include "IethResource.h"

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>


namespace yarp {
    namespace dev {
        class embObjFTsensor;
    }
}


#define EMBOBJSTRAIN_USESERVICEPARSER

// -- class embObjFTsensor

class yarp::dev::embObjFTsensor:    public yarp::dev::DeviceDriver,
                                    public eth::IethResource,
                                    public yarp::dev::IAnalogSensor,
                                    public yarp::dev::ITemperatureSensors,
                                    public yarp::dev::ISixAxisForceTorqueSensors
{
public:

    embObjFTsensor();
    ~embObjFTsensor();

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


#endif

