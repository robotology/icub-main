#pragma once

#include <string.h>
#include <shared_mutex>
#include <string>

// Yarp Includes
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include "serviceParser.h"

#include "EoProtocolAS.h"

class AnalogData
{
   public:
	AnalogData();
	bool updateStrainValues(eOprotID32_t id32, double timestamp, void* rxdata);
	bool updateTemperatureValues(eOprotID32_t id32, double timestamp, void* rxdata);
	int read(yarp::sig::Vector& out) const;
	bool getSixAxisForceTorqueSensorMeasure(yarp::sig::Vector& out, double& timestamp) const;
	int calibrateSensor();
	bool getTemperatureSensorMeasure(double& out, double& timestamp) const;
	bool getTemperatureSensorMeasure(yarp::sig::Vector& out, double& timestamp) const;

    bool fromConfig(yarp::os::Searchable &config, servConfigMultipleFTsensor_t &serviceConfig);
	void setOpen(bool value);
	void setScaleFactor(const std::vector<double>& value);
	void setBoardInfo(const std::string& value);

   private:
	mutable std::shared_mutex mutex_;
	std::vector<double> analogdata_;
	std::vector<double> offset_;
	std::vector<double> scaleFactor_;
	float lastTemperature_{0};
	double timestampTemperature_{0};
	double timestampAnalogdata_{0};

	bool open_{false};
	bool useCalibValues_{false};
	std::string boardInfo_;
    std::string devicename_;
    bool useTemperature_{false};

	static constexpr int strainChannels_{6};
};
