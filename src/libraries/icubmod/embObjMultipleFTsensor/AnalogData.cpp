#include "AnalogData.h"

#include <yarp/dev/IAnalogSensor.h>

#include "EOconstarray.h"


using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

AnalogData::AnalogData()
{
	analogdata_.resize(strainChannels_, 0.0);
	offset_.resize(strainChannels_, 0.0);
}

/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return IAnalogSensor::AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int AnalogData::read(yarp::sig::Vector& out) const
{
	// This method gives analogdata_ to the analogServer

	if (false == open_)
	{
		return false;
	}

	std::shared_lock<std::shared_mutex> lck(mutex_);

	out.resize(analogdata_.size());
	for (size_t k = 0; k < analogdata_.size(); k++)
	{
		out[k] = analogdata_[k] + offset_[k];
	}

	return IAnalogSensor::AS_OK;
}

bool AnalogData::getSixAxisForceTorqueSensorMeasure(yarp::sig::Vector& out, double& timestamp) const
{
	if (false == open_)
	{
		return false;
	}

	std::shared_lock<std::shared_mutex> lck(mutex_);

	out.resize(analogdata_.size());
	for (size_t k = 0; k < analogdata_.size(); k++)
	{
		out[k] = analogdata_[k] + offset_[k];
	}

	timestamp = timestampAnalogdata_;

	return true;
}

int AnalogData::calibrateSensor()
{
	std::shared_lock<std::shared_mutex> lck(mutex_);
	for (size_t i = 0; i < analogdata_.size(); i++)
	{
		offset_[i] = -analogdata_[i];
	}
	return IAnalogSensor::AS_OK;
}

bool AnalogData::updateStrainValues(eOprotID32_t id32, double timestamp, void* rxdata)
{
	id32 = id32;
	timestamp = timestamp;

	// called by feat_manage_analogsensors_data() which is called by:
	// eoprot_fun_UPDT_as_strain_status_calibratedvalues() or eoprot_fun_UPDT_as_strain_status_uncalibratedvalues()
	// the void* parameter inside this function is a eOas_arrayofupto12bytes_t*
	// and can be treated as a EOarray

	// eOas_ft_timedvalue_t* array = (eOas_ft_timedvalue_t*)rxdata; LUCA TODO <<<<<<---------use this struct
	EOarray* array = (EOarray*)rxdata;
	uint8_t size = eo_array_Size(array);
	uint8_t itemsize = eo_array_ItemSize(array);  // marco.accame: must be 2, as the code after uses this convention
	if ((0 == size) || (2 != itemsize))
	{
		return false;
	}

	// lock analogdata_
	std::unique_lock<std::shared_mutex> lck(mutex_);
	timestampAnalogdata_ = yarp::os::Time::now();
	for (size_t k = 0; k < analogdata_.size(); k++)
	{
		// Get the kth element of the array as a 2 bytes msg
		char* tmp = (char*)eo_array_At(array, k);
		// marco.accame: i am nervous about iterating for strain_Channels instead of size of array....
		//               thus i add a protection. if k goes beyond size of array, eo_array_At() returns NULL.
		if (NULL != tmp)
		{
			uint8_t msg[2] = {0};
			memcpy(msg, tmp, 2);
			// Got from canBusMotionControl
			analogdata_[k] = (short)(((((unsigned short)(msg[1])) << 8) + msg[0]) - (unsigned short)(0x8000));

			if (true == useCalibValues_)
			{
				analogdata_[k] = analogdata_[k] * scaleFactor_[k] / float(0x8000);
			}
		}
	}

	return true;
}

// DODO REMOVE LUCA
bool AnalogData::updateTemperatureValues(eOprotID32_t id32, double timestamp, void* rxdata)
{
	eOas_temperature_status_t* temp_st = (eOas_temperature_status_t*)rxdata;

	EOconstarray* arrayofvalues = eo_constarray_Load(reinterpret_cast<const EOarray*>(&(temp_st->arrayofdata)));

	uint8_t numofIntem2update = eo_constarray_Size(arrayofvalues);

	if (numofIntem2update > 1)
		yError() << boardInfo_ << "updateTemperature: I expect 1 item, but I received " << numofIntem2update;

	for (int i = 0; i < numofIntem2update; i++)
	{
		eOas_temperature_data_t* data = (eOas_temperature_data_t*)eo_constarray_At(arrayofvalues, i);
		if (data == NULL)
		{
			yError() << boardInfo_ << "update(): I have to update " << numofIntem2update << "items, but the " << i << "-th item is null.";
			continue;
			// NOTE: I signal this strange situation with an arror for debug porpouse...maybe we can convert in in warning when the device is stable....
		}
		else
		{
			std::unique_lock<std::shared_mutex> lck(mutex_);
			lastTemperature_ = static_cast<float>(data->value);
			timestampTemperature_ = yarp::os::Time::now();
		}
	}
	return true;
}

void AnalogData::setOpen(bool value)
{
	open_ = value;
}

void AnalogData::setScaleFactor(const std::vector<double>& value)
{
	scaleFactor_ = value;
}

void AnalogData::setBoardInfo(const std::string& value)//TODO fill somewhere LUCA
{
	boardInfo_ = value;
}

bool AnalogData::getTemperatureSensorMeasure(double& out, double& timestamp) const
{
	std::shared_lock<std::shared_mutex> lck(mutex_);
	out = lastTemperature_ / 10.0;  // I need to convert from tenths of degree centigrade to degree centigrade
	timestamp = timestampTemperature_;
	return true;
}

bool AnalogData::getTemperatureSensorMeasure(yarp::sig::Vector& out, double& timestamp) const
{
	std::shared_lock<std::shared_mutex> lck(mutex_);
	out.resize(1);
	out[0] = lastTemperature_ / 10.0;  // I need to convert from tenths of degree centigrade to degree centigrade
	timestamp = timestampTemperature_;
	return true;
}

bool AnalogData::fromConfig(yarp::os::Searchable &config, servConfigMultipleFTsensor_t &serviceConfig)
{
	ServiceParser *parser = new ServiceParser;
	bool ret = parser->parseService(config, serviceConfig);
	delete parser;

	if (!ret)
	{
		yError() << boardInfo_ << "is missing some configuration parameter. Check logs and your config file.";
		return false;
	}

	useCalibValues_ = serviceConfig.useCalibration;
	if (serviceConfig.temperatureAcquisitionrate > 0)
		useTemperature_ = true;
	devicename_ = serviceConfig.nameOfStrain;

	//TODO reenable LUCA
	//if (isVerbose())
	//	printServiceConfig(serviceConfig);
	return ret;
}