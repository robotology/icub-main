/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// general purpose stuff.
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

#ifdef WIN32
#pragma warning(once : 4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#define GET_privData(x) (*((static_cast<eo_ftsens_privData*>(x))))

embObjMultipleFTsensor::embObjMultipleFTsensor()
{
	mPriv = new eo_ftsens_privData("embObjMultipleFTsensor");
}

embObjMultipleFTsensor::~embObjMultipleFTsensor()
{
	close();
	delete &GET_privData(mPriv);
}

std::string embObjMultipleFTsensor::getBoardInfo(void) const
{
	return GET_privData(mPriv).getBoardInfo();
}

bool embObjMultipleFTsensor::initialised()
{
	return GET_privData(mPriv).isOpen();
}

bool embObjMultipleFTsensor::enableTemperatureTransmission(bool enable)
{
	if (GET_privData(mPriv).res == nullptr)
	{
		// This check is necessary because the function is called by destructor and we cannot be sure that open function has been invoked before its deletion.
		// Please notice that yarp::dev::DriversHelper::load() function creates this object and then destroys it without calls the open().
		// Here I don't give error neither warning, because the load function is called to load this devicedriver at yarprobotinterface startup, so it is the wanted behaviour.
		return false;
	}

	uint8_t cmd;
	(enable) ? cmd = 1 : cmd = 0;

	eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_temperature, 0, eoprot_tag_as_temperature_cmmnds_enable);
	if (false == GET_privData(mPriv).res->setRemoteValue(id32, &cmd))
	{
		yError() << getBoardInfo() << "fails send command enableTemperatureTransmission(" << enable << ")";
		return false;
	}
	return true;
}

//----------------------- DeviceDriver -------------------

bool embObjMultipleFTsensor::open(yarp::os::Searchable& config)
{
	// - first thing to do is verify if the eth manager is available then i parse info about the eth board.

	if (!GET_privData(mPriv).prerareEthService(config, this))
		return false;

	// read stuff from config file

	servConfigMultipleFTsensor_t serviceConfig;
	if (!GET_privData(mPriv).fromConfig(config, serviceConfig))
		return false;

	if (!analogData_.fromConfig(config, serviceConfig))
		return false;

	if (!GET_privData(mPriv).res->verifyEPprotocol(eoprot_endpoint_analogsensors))
	{
		cleanup();
		return false;
	}


#if defined(EMBOBJSTRAIN_USESERVICEPARSER)

	// Fill temperature service data in servparamtemp: some of these data are copied from the serviceconfig of ft because both services used tha same board.
	eOmn_serv_parameter_t servparamtemp;
	bool ret = GET_privData(mPriv).fillTemperatureEthServiceInfo(serviceConfig.ethservice, servparamtemp);
	if (!ret)
		return false;

	const eOmn_serv_parameter_t* servparamtemp_ptr = &servparamtemp;
	const eOmn_serv_parameter_t* servparamstrain = &serviceConfig.ethservice;

#else
	const eOmn_serv_parameter_t* servparamstrain = NULL;
	const eOmn_serv_parameter_t* servparamtemp_ptr = NULL;
#endif

	if (false == GET_privData(mPriv).res->serviceVerifyActivate(eomn_serv_category_strain, servparamstrain, 5.0))
	{
		yError() << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate()";
		cleanup();
		return false;
	}

	if (false == GET_privData(mPriv).res->serviceVerifyActivate(eomn_serv_category_temperatures, servparamtemp_ptr, 5.0))
	{
		yError() << getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate()";
		cleanup();
		return false;
	}

	// we always prepare the fullscales.
	if (false == GET_privData(mPriv).fillScaleFactor(serviceConfig))
	{
		yError() << getBoardInfo() << "open() has failed in calling  embObjMultipleFTsensor::fillScaleFactor()";
		return false;
	}

	if (false == GET_privData(mPriv).sendConfig2Strain(serviceConfig))
	{
		cleanup();
		return false;
	}

	if (false == GET_privData(mPriv).initRegulars(serviceConfig))
	{
		cleanup();
		return false;
	}

	//TODO LUCA
	if (false == GET_privData(mPriv).res->serviceStart(eomn_serv_category_ft))
	{
		yError() << getBoardInfo() << "open() fails to start service ft";
		cleanup();
		return false;
	}
	else
	{
		if (GET_privData(mPriv).isVerbose())
		{
			yDebug() << getBoardInfo() << "open() correctly starts as service ft";
		}
	}

	if (false == GET_privData(mPriv).res->serviceStart(eomn_serv_category_temperatures))
	{
		yError() << getBoardInfo() << "open() fails to start service temperature";
		cleanup();
		return false;
	}
	else
	{
		if (GET_privData(mPriv).isVerbose())
		{
			yDebug() << getBoardInfo() << "open() correctly starts as service temperature";
		}
	}

	// start the configured sensors. so far, we must keep it in here. later on we can remove this command
	if (!enableTemperatureTransmission(true))
	{
		yError() << getBoardInfo() << "open() fails to enable temperature transmission";
		cleanup();
		return false;
	}

	GET_privData(mPriv).setOpen(true);
	analogData_.setOpen(true);
	return true;
}

bool embObjMultipleFTsensor::close()
{
	cleanup();
	return true;
}

void embObjMultipleFTsensor::cleanup(void)
{
	// disable temperature
	enableTemperatureTransmission(false);

	GET_privData(mPriv).cleanup(static_cast<eth::IethResource*>(this));
}

//-------------------------------  IethResource --------------------------------
bool embObjMultipleFTsensor::update(eOprotID32_t id32, double timestamp, void* rxdata) //TODO register this callback
{
	bool ret = false;

	if (false == GET_privData(mPriv).isOpen())
	{
		return ret;
		;
	}
	eOprotEntity_t entity = eoprot_ID2entity(id32);

	switch (entity)
	{
		case eoas_entity_ft:
		{
			ret = analogData_.updateStrainValues(id32, timestamp, rxdata);
		}
		break;

		case eoas_entity_temperature:
		{
			ret = analogData_.updateTemperatureValues(id32, timestamp, rxdata);
		}
		break;
		default:
		{
			ret = false;
			yError() << getBoardInfo() << "update() failed ";
		}
	};
	return ret;
}

// -----------------------------  yarp::dev::IAnalogSensor --------------------------------------
//LUCA
/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int embObjMultipleFTsensor::read(yarp::sig::Vector& out)
{
	return analogData_.read(out);
}

int embObjMultipleFTsensor::getState(int ch)
{
	return AS_OK;
}

int embObjMultipleFTsensor::getChannels()
{
	return GET_privData(mPriv).strain_Channels;
}

//Luca
int embObjMultipleFTsensor::calibrateSensor()
{
	return analogData_.calibrateSensor();
}

int embObjMultipleFTsensor::calibrateSensor(const yarp::sig::Vector& value)
{
	return AS_OK;
}

int embObjMultipleFTsensor::calibrateChannel(int ch)
{
	return AS_OK;
}

int embObjMultipleFTsensor::calibrateChannel(int ch, double v)
{
	return AS_OK;
}

eth::iethresType_t embObjMultipleFTsensor::type()
{
	return eth::iethres_analogstrain;
}

// ---------------------- ITemperatureSensors --------------------------------------------------------
size_t embObjMultipleFTsensor::getNrOfTemperatureSensors() const
{
	return 1;
}

yarp::dev::MAS_status embObjMultipleFTsensor::getTemperatureSensorStatus(size_t sens_index) const
{
	return yarp::dev::MAS_OK;
}

bool embObjMultipleFTsensor::getTemperatureSensorName(size_t sens_index, std::string& name) const
{
	name = GET_privData(mPriv).devicename;
	return true;
}

bool embObjMultipleFTsensor::getTemperatureSensorFrameName(size_t sens_index, std::string& frameName) const
{
	frameName = GET_privData(mPriv).devicename;
	return true;
}

bool embObjMultipleFTsensor::getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const
{
	return analogData_.getTemperatureSensorMeasure(out,timestamp);
}

bool embObjMultipleFTsensor::getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
	return analogData_.getTemperatureSensorMeasure(out,timestamp);
}

//------------------------- ISixAxisForceTorqueSensors -------------------------

size_t embObjMultipleFTsensor::getNrOfSixAxisForceTorqueSensors() const
{
	return 1;
}

yarp::dev::MAS_status embObjMultipleFTsensor::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
	return yarp::dev::MAS_OK;
}

bool embObjMultipleFTsensor::getSixAxisForceTorqueSensorName(size_t sens_index, std::string& name) const
{
	name = GET_privData(mPriv).devicename;
	return true;
}

bool embObjMultipleFTsensor::getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string& frameName) const
{
	frameName = GET_privData(mPriv).devicename;
	return true;
}

bool embObjMultipleFTsensor::getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
	return analogData_.getSixAxisForceTorqueSensorMeasure(out,timestamp);
}