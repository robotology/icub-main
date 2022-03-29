/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include <string>

#include "EoAnalogSensors.h"
#include "EoBoards.h"
#include "EoManagement.h"
#include "EoMotionControl.h"

class FtInfo
{
   public:
	uint8_t ftAcquisitionRate{0};
	uint8_t temperatureAcquisitionRate{0};
	eOas_ft_mode_t useCalibration{eoas_ft_mode_calibrated};
	std::string board{"none"};
	int port{0};
	int address{0};
	int majorProtocol{0};
	int minorProtocol{0};
	int majorFirmware{0};
	int minorFirmware{0};
	int buildFirmware{0};

	bool toEomn(eOas_ft_sensordescriptor_t& out) const;
};

bool operator==(const FtInfo& right, const FtInfo& left);
bool operator!=(const FtInfo& right, const FtInfo& left);