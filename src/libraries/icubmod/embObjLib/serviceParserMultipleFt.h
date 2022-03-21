/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include <yarp/os/Bottle.h>

#include <map>
#include <string>

#include "EoAnalogSensors.h"
#include "EoBoards.h"
#include "EoManagement.h"
#include "EoMotionControl.h"

using namespace yarp::os;

class FtInfo
{
   public:
	int ftAcquisitionRate;
	int temperatureAcquisitionRate;
	bool useCalibration;
	std::string board{""};
	std::string location{""};
	int majorProtocol{0};
	int minorProtocol{0};
	int majorFirmware{0};
	int minorFirmware{0};
	int buildFirmware{0};
};
bool operator==(const FtInfo& right,const FtInfo& left);
bool operator!=(const FtInfo& right,const FtInfo& left);

class CanMonitor
{
   public:
	uint8_t checkPeriod;
	eObrd_canmonitor_reportmode_t reportmode;
	uint16_t periodofreport;
};

class ServiceParserMultipleFt
{
   public:
	ServiceParserMultipleFt();
	bool parse(const yarp::os::Searchable& config);
	eOmn_serv_config_data_as_ft_t toEomn() const;

   protected:
	virtual bool checkPropertyCanBoards(const Bottle& bPropertiesCanBoards, bool& formaterror);
	virtual bool checkPropertySensors(const Bottle& property, bool& formaterror);
	virtual bool checkSettings(const Bottle& settings, bool& formaterror);
	virtual bool checkServiceType(const Bottle& service, bool& formaterror);
	virtual bool checkCanMonitor(const Bottle& service, bool& formaterror);

	std::map<std::string /*sensor id*/, FtInfo> ftInfo_;
	eObrd_canmonitor_cfg_t canMonitor_;
};