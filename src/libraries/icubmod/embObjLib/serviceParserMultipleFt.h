/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _SERVICEPARSERMULTIPLEFT_H_
#define _SERVICEPARSERMULTIPLEFT_H_

#include <yarp/os/Bottle.h>

#include <map>
#include <string>

#include "EoAnalogSensors.h"
#include "EoBoards.h"
#include "EoManagement.h"
#include "EoMotionControl.h"
#include "ftInfo.h"

using namespace yarp::os;

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
	bool toEomn(eOmn_serv_config_data_as_ft_t& out) const;
	std::map<std::string, FtInfo>& getFtInfo();

   protected:
	virtual bool checkPropertyCanBoards(const Bottle& bPropertiesCanBoards);
	virtual bool checkPropertySensors(const Bottle& property);
	virtual bool checkSettings(const Bottle& settings);
	virtual bool checkServiceType(const Bottle& service);
	virtual bool checkCanMonitor(const Bottle& service);
	virtual eObrd_type_t checkBoardType(const std::string& boardType);

	std::map<std::string /*sensor id*/, FtInfo> ftInfo_;
	eObrd_canmonitor_cfg_t canMonitor_;
};

#endif
