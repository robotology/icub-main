#pragma once

#include <string.h>

#include <iostream>
#include <string>

// Yarp Includes
#include <yarp/os/Time.h>

// specific to this device driver.
#include "EOconstarray.h"
#include "EoAnalogSensors.h"
#include "EoProtocolAS.h"
#include "embObjMultipleFTsensor.h"
#include "eo_ftsens_privData.h"

class AnalogData
{
    public:
        bool updateStrainValues(eOprotID32_t id32, double timestamp, void* rxdata);
	    bool updateTemperatureValues(eOprotID32_t id32, double timestamp, void* rxdata);
        int read(yarp::sig::Vector& out);
        bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const;
        int calibrateSensor();
        void setIsOpen(bool value);
        void setScaleFactor(const std::vector<double>& value);
        void setUseCalibValues_(bool value);
        void setBoardInfo(const std::string& value);

    private:
    	mutable std::mutex mutex_;
        std::vector<double> analogdata;
        std::vector<double> offset;
        std::vector<double> scaleFactor_;
        float lastTemperature{0};
	    double timestampTemperature{0};
        double timestampAnalogdata{0};
        bool open_{false};
        bool useCalibValues_;
        std::string boardInfo_;
};
