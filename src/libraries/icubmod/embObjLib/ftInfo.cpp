/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "ftInfo.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

bool FtInfo::toEomn(eOas_ft_sensordescriptor_t& out) const
{
	out.boardinfo.type = eoboards_string2type2(board.c_str(), true);
	out.canloc.addr = address;
	out.canloc.insideindex = eobrd_caninsideindex_none;
	out.boardinfo.firmware = {majorFirmware, minorFirmware, buildFirmware};
	out.boardinfo.protocol = {majorProtocol, minorProtocol};

	try
	{
		if (port == 1)
			out.canloc.port = eOcanport1;
		else if (port == 2)
			out.canloc.port = eOcanport2;
		else
		{
			yError() << "FtInfo::toEomn() invalid can port";
			out = eOas_ft_sensordescriptor_t();
			return false;
		}
	}
	catch (const std::exception& e)
	{
		yError() << "FtInfo::toEomn() invalid can port";
		out = eOas_ft_sensordescriptor_t();
		return false;
	}

	return true;
}

bool operator==(const FtInfo& right, const FtInfo& left)
{
	if (right.ftAcquisitionRate != left.ftAcquisitionRate)
		return false;
	if (right.temperatureAcquisitionRate != left.temperatureAcquisitionRate)
		return false;
	if (right.useCalibration != left.useCalibration)
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

bool operator!=(const FtInfo& right, const FtInfo& left)
{
	return !(right == left);
}