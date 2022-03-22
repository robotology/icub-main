/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
#include "eo_ftsens_privData.h"

#include "EOnv_hid.h"
#include "EoProtocolAS.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

eo_ftsens_privData::eo_ftsens_privData(std::string name) : embObjDevPrivData(name), useCalibValues(false), useTemperature(false), scaleFactorIsFilled(false)
{
	scaleFactor.resize(strain_Channels, 1.0);
}

eo_ftsens_privData::~eo_ftsens_privData()
{
	scaleFactor.resize(0);
}


bool eo_ftsens_privData::initRegulars(servConfigMultipleFTsensor_t &serviceConfig)
{
	vector<eOprotID32_t> id32v(0);	// vector with id of nv to configure as regulars
	eOprotID32_t id32 = eo_prot_ID32dummy;

	// 1) set regulars for ft (strain) service
	if (true == serviceConfig.useCalibration)
	{
		id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_strain_status_calibratedvalues);
	}
	else
	{
		id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_strain_status_uncalibratedvalues);
	}

	id32v.push_back(id32);

	if (!serviceSetRegulars(eomn_serv_category_ft, id32v))
		return false;

	// 2) set regulars for temperature service
	id32v.resize(0);
	id32 = eo_prot_ID32dummy;

	id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_temperature, 0, eoprot_tag_as_temperature_status);
	id32v.push_back(id32);

	if (!serviceSetRegulars(eomn_serv_category_temperatures, id32v))
		return false;

	return true;
}

void eo_ftsens_privData::printServiceConfig(servConfigMultipleFTsensor_t &serviceConfig)
{
	char loc[20] = {0};
	char fir[20] = {0};
	char pro[20] = {0};
	const char *boardname = (NULL != res) ? (res->getProperties().boardnameString.c_str()) : ("NOT-ASSIGNED-YET");
	const char *ipv4 = (NULL != res) ? (res->getProperties().ipv4addrString.c_str()) : ("NOT-ASSIGNED-YET");
	const char *boardtype = eoboards_type2string2(static_cast<eObrd_type_t>(serviceConfig.ethservice.configuration.data.as.strain.boardtype.type), eobool_true);
	ServiceParser *parser = new ServiceParser();
	parser->convert(serviceConfig.ethservice.configuration.data.as.ft.canloc, loc, sizeof(loc));
	parser->convert(serviceConfig.ethservice.configuration.data.as.ft.boardtype.firmware, fir, sizeof(fir));
	parser->convert(serviceConfig.ethservice.configuration.data.as.ft.boardtype.protocol, pro, sizeof(pro));

	yInfo() << "The embObjMultipleFTsensor device using BOARD" << boardname << " w/ IP" << ipv4 << "has the following service config:";
	yInfo() << "- acquisitionrate =" << serviceConfig.acquisitionrate;
	yInfo() << "- useCalibration =" << serviceConfig.useCalibration;
	yInfo() << "- STRAIN of type" << boardtype << "named" << serviceConfig.nameOfStrain << "@" << loc << "with required protocol version =" << pro << "and required firmware version =" << fir;
	delete parser;
}

bool eo_ftsens_privData::sendConfig2Strain(servConfigMultipleFTsensor_t &serviceConfig)
{
	eOas_ft_config_t ftConfig = {0};

	ftConfig.datarate = serviceConfig.acquisitionrate;
	ftConfig.signaloncefullscale = eobool_false;
	ftConfig.mode = (true == serviceConfig.useCalibration) ? (eoas_strainmode_txcalibrateddatacontinuously) : (eoas_strainmode_txuncalibrateddatacontinuously);

	// version with read-back

	// LUCA TODO
	eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_strain_config);

	if (false == res->setcheckRemoteValue(id32, &ftConfig, 10, 0.010, 0.050))
	{
		yError() << getBoardInfo() << "FATAL: sendConfig2Strain() had an error while calling setcheckRemoteValue() for strain config ";
		return false;
	}
	else
	{
		if (isVerbose())
		{
			yDebug() << getBoardInfo() << "sendConfig2Strain() correctly configured strain coinfig ";
		}
	}

	// configure the service of temperature
	id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_temperature, 0, eoprot_tag_as_temperature_config);

	eOas_temperature_config_t tempconfig = {0};
	if (serviceConfig.temperatureAcquisitionrate > 0)
	{
		tempconfig.enabled = 1;
		tempconfig.datarate = serviceConfig.temperatureAcquisitionrate / 1000;
	}

	if (false == res->setcheckRemoteValue(id32, &tempconfig, 10, 0.010, 0.050))
	{
		yError() << getBoardInfo() << "FATAL: sendConfig2Strain(temperature) had an error while calling setcheckRemoteValue() for strain config ";
		return false;
	}
	else
	{
		if (isVerbose())
		{
			yDebug() << getBoardInfo() << "sendConfig2Strain() correctly configured strain coinfig ";
		}
	}
	return true;
}

bool eo_ftsens_privData::fillTemperatureEthServiceInfo(eOmn_serv_parameter_t &ftSrv, eOmn_serv_parameter_t &tempSrv)
{
	//     const eOmn_serv_parameter_t* servparamstrain = &serviceConfig.ethservice;
	//     eOmn_serv_parameter_t servparamtemp;
	//     const eOmn_serv_parameter_t* servparamtemp_ptr = &servparamtemp;

	tempSrv.configuration.type = eomn_serv_AS_temperatures;

	EOarray *array = eo_array_New(eOas_temperature_descriptors_maxnumber, sizeof(eOas_temperature_descriptor_t), &(tempSrv.configuration.data.as.temperature.arrayofdescriptor));
	eOas_temperature_descriptor_t descr = {0};
	descr.typeofboard = ftSrv.configuration.data.as.ft.boardtype.type;	// eobrd_strain2 ;
	descr.typeofsensor = eoas_temperature_t1;
	descr.on.can.place = eobrd_place_can;
	descr.on.can.port = ftSrv.configuration.data.as.ft.canloc.port;
	descr.on.can.addr = ftSrv.configuration.data.as.ft.canloc.addr;
	eo_array_PushBack(array, &descr);

	eOas_temperature_setof_boardinfos_t *boardInfoSet_ptr = &tempSrv.configuration.data.as.temperature.setofboardinfos;
	eOresult_t res = eoas_temperature_setof_boardinfos_clear(boardInfoSet_ptr);
	if (res != eores_OK)
	{
		yError() << getBoardInfo() << "Error in eoas_temperature_setof_boardinfos_clear()";
		return false;
	}

	eObrd_info_t boardInfo = {0};
	boardInfo.type = ftSrv.configuration.data.as.ft.boardtype.type;
	memcpy(&boardInfo.protocol, &(ftSrv.configuration.data.as.ft.boardtype.protocol), sizeof(eObrd_protocolversion_t));
	memcpy(&boardInfo.firmware, &(ftSrv.configuration.data.as.ft.boardtype.firmware), sizeof(eObrd_firmwareversion_t));
	res = eoas_temperature_setof_boardinfos_add(boardInfoSet_ptr, &boardInfo);
	if (eores_OK != res)
	{
		yError() << getBoardInfo() << "Error in eoas_temperature_setof_boardinfos_add()";
		return false;
	}

	return true;
}