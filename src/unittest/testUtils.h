/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#pragma once

#include "EoBoards.h"
#include "serviceParserCanBattery.h"
#include "serviceParserMultipleFt.h"

class ServiceParserMultipleFt_mock : public ServiceParserMultipleFt
{
   public:
	using ServiceParserMultipleFt::canMonitor_;
	using ServiceParserMultipleFt::checkBoardType;
	using ServiceParserMultipleFt::checkCanMonitor;
	using ServiceParserMultipleFt::checkPropertyCanBoards;
	using ServiceParserMultipleFt::checkPropertySensors;
	using ServiceParserMultipleFt::checkServiceType;
	using ServiceParserMultipleFt::checkSettings;
	using ServiceParserMultipleFt::ftInfo_;

	ServiceParserMultipleFt_mock() : ServiceParserMultipleFt(){};
};

class ServiceParserCanBattery_mock : public ServiceParserCanBattery
{
   public:
	using ServiceParserCanBattery::batteryInfo_;
	using ServiceParserCanBattery::checkBoardType;
	using ServiceParserCanBattery::checkPropertyCanBoards;
	using ServiceParserCanBattery::checkPropertySensors;
	using ServiceParserCanBattery::checkServiceType;
	using ServiceParserCanBattery::checkSettings;

	ServiceParserCanBattery_mock() : ServiceParserCanBattery(){};
};

inline bool operator==(const eObrd_canmonitor_cfg_t &right, const eObrd_canmonitor_cfg_t &left)
{
	if (right.periodofcheck != left.periodofcheck)
		return false;
	if (right.reportmode != left.reportmode)
		return false;
	if (right.periodofreport != left.periodofreport)
		return false;
	return true;
}

inline bool operator==(const eObrd_firmwareversion_t &right, const eObrd_firmwareversion_t &left)
{
	if (right.major != left.major)
		return false;
	if (right.minor != left.minor)
		return false;
	if (right.build != left.build)
		return false;

	return true;
}
inline bool operator!=(const eObrd_firmwareversion_t &right, const eObrd_firmwareversion_t &left)
{
	return !(right == left);
}

inline bool operator==(const eObrd_protocolversion_t &right, const eObrd_protocolversion_t &left)
{
	if (right.major != left.major)
		return false;
	if (right.minor != left.minor)
		return false;

	return true;
}
inline bool operator!=(const eObrd_protocolversion_t &right, const eObrd_protocolversion_t &left)
{
	return !(right == left);
}

inline bool operator==(const eObrd_canlocation_t &right, const eObrd_canlocation_t &left)
{
	if (right.port != left.port)
		return false;
	if (right.addr != left.addr)
		return false;
	if (right.insideindex != left.insideindex)
		return false;

	return true;
}

inline bool operator!=(const eObrd_canlocation_t &right, const eObrd_canlocation_t &left)
{
	return !(right == left);
}

inline bool operator==(const eOas_ft_sensordescriptor_t &right, const eOas_ft_sensordescriptor_t &left)
{
	if (right.boardinfo.type != left.boardinfo.type)
		return false;
	if (right.boardinfo.protocol != left.boardinfo.protocol)
		return false;
	if (right.boardinfo.firmware != left.boardinfo.firmware)
		return false;
	if (right.canloc != left.canloc)
		return false;

	return true;
}
inline bool operator!=(const eOas_ft_sensordescriptor_t &right, const eOas_ft_sensordescriptor_t &left)
{
	return !(right == left);
}
inline bool operator!=(const eOas_ft_config_t &right, const eOas_ft_config_t &left)
{
	if (right.mode != left.mode)
		return false;
	if (right.ftperiod != left.ftperiod)
		return false;
	if (right.calibrationset != left.calibrationset)
		return false;
	if (right.temperatureperiod != left.temperatureperiod)
		return false;
}

inline bool operator==(const eOas_canbattery_sensordescriptor_t &right, const eOas_canbattery_sensordescriptor_t &left)
{
	if (right.boardinfo.type != left.boardinfo.type)
		return false;
	if (right.boardinfo.protocol != left.boardinfo.protocol)
		return false;
	if (right.boardinfo.firmware != left.boardinfo.firmware)
		return false;
	if (right.canloc != left.canloc)
		return false;

	return true;
}
inline bool operator!=(const eOas_canbattery_sensordescriptor_t &right, const eOas_canbattery_sensordescriptor_t &left)
{
	return !(right == left);
}

inline bool operator!=(const eOas_canbattery_config_t &right, const eOas_canbattery_config_t &left)
{
	if (right.period != left.period)
		return false;
}

