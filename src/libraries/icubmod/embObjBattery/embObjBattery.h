/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _EMBOBJCANBATTERYSENSORS_H_
#define _EMBOBJCANBATTERYSENSORS_H_

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IBattery.h>
#include <yarp/sig/Vector.h>

#include <map>
#include <shared_mutex>
#include <memory>
#include <string>
#include <utility>

#include "embObjGeneralDevPrivData.h"
#include "serviceParserCanBattery.h"

// This is to provide compatibility with both YARP 3.11 and 3.12
#include <iCub/YarpDevReturnValueCompat.h>

namespace yarp::dev
{
class embObjBattery;
}

class CanBatteryData
{
   public:
    int16_t temperature_{0};
    float32_t voltage_{0};
    float32_t current_{0};
    float32_t charge_{0};
    uint16_t status_{0};
    uint16_t prevStatus_{0};
    double timeStamp_{0};
    std::string sensorName_{};
    eObrd_type_t sensorType_{eobrd_unknown};
    

    void decode(eOas_battery_timedvalue_t *data, double timestamp);
    bool operator==(const CanBatteryData &other) const;
    bool operator!=(const CanBatteryData &other) const;
};

class yarp::dev::embObjBattery : public yarp::dev::DeviceDriver, public eth::IethResource, public yarp::dev::IBattery
{
   public:
    embObjBattery();
    embObjBattery(std::shared_ptr<yarp::dev::embObjDevPrivData> device);  // For Unittesting only
    ~embObjBattery();

    bool open(yarp::os::Searchable &config);
    bool close();

    // IethResource interface
    bool initialised() override;
    eth::iethresType_t type() override;
    bool update(eOprotID32_t id32, double timestamp, void *rxdata) override;

    // IBattery
    YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryVoltage(double &voltage) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryCurrent(double &current) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryCharge(double &charge) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryStatus(Battery_status &status) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryTemperature(double &temperature) override;
    YARP_DEV_RETURN_VALUE_TYPE_CH312 getBatteryInfo(std::string &battery_info) override;

    virtual double calculateBoardTime(eOabstime_t current);

   protected:
    std::shared_ptr<yarp::dev::embObjDevPrivData> device_;
    mutable std::shared_mutex mutex_;
    CanBatteryData canBatteryData_;
    std::map<eOprotID32_t, eOabstime_t> timeoutUpdate_;

    bool sendConfig2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes);
    bool sendStart2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes);
    bool initRegulars(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes);
    void cleanup(void);
    bool checkUpdateTimeout(eOprotID32_t id32, eOabstime_t current);
    std::string updateStatusStringStream(const uint16_t &currStatus, const uint16_t &prevStatus, bool isFirstLoop);
    static constexpr eOabstime_t updateTimeout_{11000};
    std::vector<yarp::dev::MAS_status> masStatus_{MAS_OK, MAS_OK, MAS_OK, MAS_OK};

    static constexpr bool checkUpdateTimeoutFlag_{false};  // Check timer disabled
    static constexpr bool useBoardTimeFlag_{true};         // Calculate board time if true otherway use yarp time

    bool isCanDataAvailable = false;
    bool isPastFirstPrint = false;

    double firstYarpTimestamp_{0};
    eOabstime_t firstCanTimestamp_{0};
};

#endif
