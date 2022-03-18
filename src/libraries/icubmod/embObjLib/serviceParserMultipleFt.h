/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include "EoBoards.h"
#include "EoManagement.h"
#include "EoAnalogSensors.h"
#include "EoMotionControl.h"

#include <yarp/os/Bottle.h>

#include <map>
#include <string>

using namespace yarp::os;

class FtInfo
{
   public:
	int ftAcquisitionRate;
	int temperatureAcquisitionRate;
	bool useCalibration;
	std::string board;
	std::string location;
	int majorProtocol;
	int minorProtocol;
	int majorFirmware;
	int minorFirmware;
	int buildFirmware;
};

class CanMonitor
{
   public:
	uint8_t checkrate;
	eObrd_canmonitor_reportmode_t reportmode;
	uint16_t periodicreportrate;
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

   private:
	std::map<std::string /*sensor id*/, FtInfo> ftInfo_;
	eOmn_serv_config_data_as_ft_t canMonitor_;
};