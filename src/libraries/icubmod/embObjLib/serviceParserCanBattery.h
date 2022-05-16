/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _ServiceParserCanBattery_H_
#define _ServiceParserCanBattery_H_

#include <yarp/os/Bottle.h>

#include <map>
#include <string>

#include "EoAnalogSensors.h"
#include "EoBoards.h"
#include "EoManagement.h"
#include "EoMotionControl.h"
#include "batteryInfo.h"

using namespace yarp::os;


class ServiceParserCanBattery
{
   public:
	ServiceParserCanBattery();
	bool parse(const yarp::os::Searchable& config);
	bool toEomn(eOmn_serv_config_data_as_canbattery_t& out) const;
	BatteryInfo &getBatteryInfo();

   protected:
	virtual bool checkPropertyCanBoards(const Bottle& bPropertiesCanBoards);  // OK
	virtual bool checkPropertySensors(const Bottle& property);				  // OK
	virtual bool checkSettings(const Bottle& settings);						  // OK
	virtual bool checkServiceType(const Bottle& service);					  // OK
	virtual eObrd_type_t checkBoardType(const std::string& boardType);

	BatteryInfo batteryInfo_;
};

#endif
