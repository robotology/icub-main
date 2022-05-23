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

#include "embObjGeneralDevPrivData.h"
#include "serviceParserCanBattery.h"

namespace yarp::dev
{
class embObjCanBatterysensor;
}

class CanBatteryData
{
   public:
	yarp::sig::Vector data_{0, 0, 0, 0, 0, 0};
	double timeStamp_;
	std::string sensorName_;
};

class yarp::dev::embObjCanBatterysensor : public yarp::dev::DeviceDriver, public eth::IethResource, public yarp::dev::IBattery
{
   public:
	embObjCanBatterysensor();
	embObjCanBatterysensor(std::shared_ptr<yarp::dev::embObjDevPrivData> device);  // For Unittesting only
	~embObjCanBatterysensor();

	bool open(yarp::os::Searchable &config);
	bool close();

	// IethResource interface
	bool initialised() override;
	eth::iethresType_t type() override;
	bool update(eOprotID32_t id32, double timestamp, void *rxdata) override;

	// IBattery

	bool getBatteryVoltage(double &voltage) override;
	bool getBatteryCurrent(double &current) override;
	bool getBatteryCharge(double &charge) override;
	bool getBatteryStatus(Battery_status &status) override;
	bool getBatteryTemperature(double &temperature) override;
	bool getBatteryInfo(std::string &battery_info) override;

   protected:
	std::shared_ptr<yarp::dev::embObjDevPrivData> device_;
	mutable std::shared_mutex mutex_;
	CanBatteryData canBatteryData_;
	/*std::map<eOprotID32_t, TemperatureData> temperaturesensordata_;
	std::map<eOprotID32_t, eOabstime_t> timeoutUpdate_;
*/
	bool sendConfig2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes);
	bool sendStart2boards(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes);
	bool initRegulars(ServiceParserCanBattery &parser, eth::AbstractEthResource *deviceRes);
	void cleanup(void); /*
	 double calculateBoardTime(eOabstime_t current);
	 bool checkUpdateTimeout(eOprotID32_t id32, eOabstime_t current);
	 static constexpr eOabstime_t updateTimeout_{11000};
	 std::vector<yarp::dev::MAS_status> masStatus_{MAS_OK, MAS_OK, MAS_OK, MAS_OK};

	 static constexpr bool checkUpdateTimeoutFlag_{false};  // Check timer disabled
	 static constexpr bool useBoardTimeFlag_{true};         // Calculate board time if true otherway use yarp time

	 double firstYarpTimestamp_{0};
	 eOabstime_t firstCanTimestamp_{0};
	 */
};

#endif
