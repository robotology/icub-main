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
	int ftAcquisitionRate{0};
	int temperatureAcquisitionRate{0};
	bool useCalibration{0};
	std::string board{""};
	int port{0};
	int address{0};
	int majorProtocol{0};
	int minorProtocol{0};
	int majorFirmware{0};
	int minorFirmware{0};
	int buildFirmware{0};

	eOas_ft_sensordescriptor_t toEomn() const;
};

bool operator==(const FtInfo& right, const FtInfo& left);
bool operator!=(const FtInfo& right, const FtInfo& left);