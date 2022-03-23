/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/sig/Vector.h>

#include <shared_mutex>
#include <string>
#include <map>

#include "serviceParserMultipleFt.h"
#include "embObjGeneralDevPrivData.h"

namespace yarp
{
namespace dev
{
class embObjMultipleFTsensor;
}
}  // namespace yarp

static constexpr int ftChannels_{6};
static constexpr int ftMaxNumber_{4};

class FtData
{
   public:
	FtData() : data_(ftChannels_){};
	yarp::sig::Vector data_;
	double timeStamp_;
};

class TemperatureData
{
   public:
	double data_;
	double timeStamp_;
};

class yarp::dev::embObjMultipleFTsensor : public yarp::dev::DeviceDriver, 
										  public eth::IethResource, 
										  public yarp::dev::ITemperatureSensors, 
										  public yarp::dev::ISixAxisForceTorqueSensors
{
   public:
	embObjMultipleFTsensor();
	~embObjMultipleFTsensor();

	bool open(yarp::os::Searchable& config);
	bool close();

	// IethResource interface
	virtual bool initialised();
	virtual eth::iethresType_t type();
	virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

	// ITemperatureSensors
	virtual size_t getNrOfTemperatureSensors() const override;
	virtual yarp::dev::MAS_status getTemperatureSensorStatus(size_t sens_index) const override;
	virtual bool getTemperatureSensorName(size_t sens_index, std::string& name) const override;
	virtual bool getTemperatureSensorFrameName(size_t sens_index, std::string& frameName) const override;
	virtual bool getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const override;
	virtual bool getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

	// ISixAxisForceTorqueSensors
	virtual size_t getNrOfSixAxisForceTorqueSensors() const override;
	virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const override;
	virtual bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string& name) const override;
	virtual bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string& frameName) const override;
	virtual bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

   private:
	yarp::dev::embObjDevPrivData device_;
	mutable std::shared_mutex mutex_;
	std::map<eOprotID32_t, FtData> ftData_;
	std::map<eOprotID32_t, TemperatureData> temperature_;

   private:
	bool sendConfig2boards(ServiceParserMultipleFt& parser);
	bool sendStart2boards(ServiceParserMultipleFt& parser);
	bool initRegulars(ServiceParserMultipleFt& parser);
	void cleanup(void);
};