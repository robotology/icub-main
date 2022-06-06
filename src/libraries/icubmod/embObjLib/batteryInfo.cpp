/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "batteryInfo.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

bool BatteryInfo::toEomn(eOas_battery_sensordescriptor_t& out) const
{
	out.boardinfo.type = board;
	out.canloc.addr = address;
	out.canloc.insideindex = eobrd_caninsideindex_none;
	out.boardinfo.firmware = {(uint8_t)majorFirmware, (uint8_t)minorFirmware, (uint8_t)buildFirmware};
	out.boardinfo.protocol = {(uint8_t)majorProtocol, (uint8_t)minorProtocol};

	try
	{
		if (port == 1)
			out.canloc.port = eOcanport1;
		else if (port == 2)
			out.canloc.port = eOcanport2;
		else
		{
			yError() << "BatteryInfo::toEomn() invalid can port";
			out = eOas_battery_sensordescriptor_t();
			return false;
		}
	}
	catch (const std::exception& e)
	{
		yError() << "BatteryInfo::toEomn() invalid can port";
		out = eOas_battery_sensordescriptor_t();
		return false;
	}
	return true;
}

bool operator==(const BatteryInfo& right, const BatteryInfo& left)
{
	if (right.acquisitionRate != left.acquisitionRate)
		return false;
	if (right.board != left.board)
		return false;
	if (right.port != left.port)
		return false;
	if (right.address != left.address)
		return false;
	if (right.majorProtocol != left.majorProtocol)
		return false;
	if (right.minorProtocol != left.minorProtocol)
		return false;
	if (right.majorFirmware != left.majorFirmware)
		return false;
	if (right.minorFirmware != left.minorFirmware)
		return false;
	if (right.buildFirmware != left.buildFirmware)
		return false;
	return true;
};

bool operator!=(const BatteryInfo& right, const BatteryInfo& left)
{
	return !(right == left);
}
