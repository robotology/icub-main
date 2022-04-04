/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _EMBOBJMULTIPLEFTSENSORS_H_
#define _EMBOBJMULTIPLEFTSENSORS_H_

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/sig/Vector.h>

#include <map>
#include <shared_mutex>
#include <memory>
#include <string>

#include "embObjGeneralDevPrivData.h"
#include "serviceParserMultipleFt.h"

namespace yarp::dev
{
class embObjMultipleFTsensors;
}

static constexpr int ftChannels_{6};

class FtData
{
   public:
    yarp::sig::Vector data_{0, 0, 0, 0, 0, 0};
    double timeStamp_;
    std::string sensorName_;
};

class TemperatureData
{
   public:
    eOmeas_temperature_t data_;
    double timeStamp_;
};

class yarp::dev::embObjMultipleFTsensors : public yarp::dev::DeviceDriver, public eth::IethResource, public yarp::dev::ITemperatureSensors, public yarp::dev::ISixAxisForceTorqueSensors
{
   public:
    embObjMultipleFTsensors();
    embObjMultipleFTsensors(std::shared_ptr<yarp::dev::embObjDevPrivData> device);  // For Unittesting only
    ~embObjMultipleFTsensors();

    bool open(yarp::os::Searchable& config);
    bool close();

    // IethResource interface
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

    // ITemperatureSensors
    virtual size_t getNrOfTemperatureSensors() const override;
    virtual yarp::dev::MAS_status getTemperatureSensorStatus(size_t sensorindex) const override;
    virtual bool getTemperatureSensorName(size_t sensorindex, std::string& name) const override;
    virtual bool getTemperatureSensorFrameName(size_t sensorindex, std::string& frameName) const override;
    virtual bool getTemperatureSensorMeasure(size_t sensorindex, double& out, double& timestamp) const override;
    virtual bool getTemperatureSensorMeasure(size_t sensorindex, yarp::sig::Vector& out, double& timestamp) const override;

    // ISixAxisForceTorqueSensors
    virtual size_t getNrOfSixAxisForceTorqueSensors() const override;
    virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sensorindex) const override;
    virtual bool getSixAxisForceTorqueSensorName(size_t sensorindex, std::string& name) const override;
    virtual bool getSixAxisForceTorqueSensorFrameName(size_t sensorindex, std::string& frameName) const override;
    virtual bool getSixAxisForceTorqueSensorMeasure(size_t sensorindex, yarp::sig::Vector& out, double& timestamp) const override;

   protected:
    std::shared_ptr<yarp::dev::embObjDevPrivData> device_;
    mutable std::shared_mutex mutex_;
    std::map<eOprotID32_t, FtData> ftSensorsData_;
    std::map<eOprotID32_t, TemperatureData> temperaturesensordata_;
    std::map<eOprotID32_t, eOabstime_t> timeoutUpdate_;

    bool sendConfig2boards(ServiceParserMultipleFt& parser, eth::AbstractEthResource* deviceRes);
    bool sendStart2boards(ServiceParserMultipleFt& parser, eth::AbstractEthResource* deviceRes);
    bool initRegulars(ServiceParserMultipleFt& parser, eth::AbstractEthResource* deviceRes);
    void cleanup(void);
    double calculateBoardTime(eOabstime_t current);
    bool checkUpdateTimeout(eOprotID32_t id32, eOabstime_t current);
    static constexpr eOabstime_t updateTimeout_{11000};
    std::vector<yarp::dev::MAS_status> masStatus_{MAS_OK, MAS_OK, MAS_OK, MAS_OK};

    static constexpr bool checkUpdateTimeoutFlag_{false};  // Check timer disabled
    static constexpr bool useBoardTimeFlag_{true};         // Calculate board time if true otherway use yarp time

    double firstYarpTimestamp_{0};
    eOabstime_t firstCanTimestamp_{0};
};

#endif
