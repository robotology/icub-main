/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _BATTERYINFO_H_
#define _BATTERYINFO_H_

#include <string>

#include "EoAnalogSensors.h"
#include "EoBoards.h"
#include "EoManagement.h"
#include "EoMotionControl.h"

class eOas_battery_sensordescriptor_t
{};
class eOmn_serv_config_data_as_canbattery_t
{};

class BatteryInfo
{
   public:
	uint8_t acquisitionRate{0};
	eObrd_type_t board{eobrd_unknown};
	int port{0};
	int address{0};
	int majorProtocol{0};
	int minorProtocol{0};
	int majorFirmware{0};
	int minorFirmware{0};
	int buildFirmware{0};

	bool toEomn(eOas_battery_sensordescriptor_t& out) const;
};

bool operator==(const BatteryInfo& right, const BatteryInfo& left);
bool operator!=(const BatteryInfo& right, const BatteryInfo& left);

#endif  // include-guard
