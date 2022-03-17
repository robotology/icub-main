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

using namespace yarp::os;

class servAScollector_t;

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
	bool parse(yarp::os::Searchable& config);
	eOmn_serv_config_data_as_ft_t toEomn() const;

   protected:
	virtual bool checkPropertyCanBoards(const Bottle& bPropertiesCanBoards, bool& formaterror);
	virtual bool checkPropertySensors(const Bottle& property, bool& formaterror);
	virtual bool checkSettings(const Bottle& settings, bool& formaterror);
	virtual bool checkServiceType(const Bottle& service, bool& formaterror);
	virtual bool checkCanMonitor(const Bottle& service, bool& formaterror);

   private:
	const std::map<std::string, eObrd_canmonitor_reportmode_t> stringToReport = {{"NEVER", eobrd_canmonitor_reportmode_NEVER},
																				 {"LOSTFOUND", eobrd_canmonitor_reportmode_justLOSTjustFOUND},
																				 {"LOSTFOUNDLOST", eobrd_canmonitor_reportmode_justLOSTjustFOUNDstillLOST},
																				 {"ALL", eobrd_canmonitor_reportmode_ALL}};

	std::map<std::string, FtInfo> ftInfo_;
	eOmn_serv_config_data_as_ft_t canMonitor_;
};