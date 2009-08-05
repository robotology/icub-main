// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Assif Mirza
 * email:   assif.mirza@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <ace/OS.h>

#include <iCub/iha/debug.h>
#include <iCub/iha/mem_util.h> // from ControlBoardInterfaces.inl

#include <iCub/iha/Limits.h>

using namespace yarp::os;

/**
 * Function to normalize a value to range 0..1
 * using the limits arrays above
 */
double iCub::iha::Limits::normalize(double value, int index) {
	double n = (value - limLo[index]) / (limHi[index] - limLo[index]);
	if (n<0.0) return 0.0;
	if (n>1.0) return 1.0;
	return n;
}	

/**
 * Check ranges of a sensor value
 */
bool iCub::iha::Limits::inRange(double value, int index) {
	if (value < limLo[index] || value > limHi[index]) return false;
	return true;
}

// returns number of sensor/motors that are configured
int iCub::iha::Limits::readLimits(Searchable &config, int numImageSensorsX, int numImageSensorsY) {
	int dim;
	Bottle& sensorsGroup = config.findGroup("SENSORS");
	if (sensorsGroup.isNull()) {
		ACE_OS::fprintf(stderr,"Error: SENSORS group not found\n");
		return 0;
	}
	dim = sensorsGroup.size() -1 + numImageSensorsX * numImageSensorsY;

	Bottle& limitLoGroup = config.findGroup("LIMIT_LO");
	if (limitLoGroup.isNull()) {
		ACE_OS::fprintf(stderr,"Error: LIMIT_LO group not found\n");
		return 0;
	}
	Bottle& limitHiGroup = config.findGroup("LIMIT_HI");
	if (limitHiGroup.isNull()) {
		ACE_OS::fprintf(stderr,"Error: LIMIT_HI group not found\n");
		return 0;
	}
	IhaDebug::pmesg(DBGL_STATUS1,"sensorsGroup.size() %d\n", sensorsGroup.size());
	IhaDebug::pmesg(DBGL_STATUS1,"limitLoGroup.size() %d\n", limitLoGroup.size());
	IhaDebug::pmesg(DBGL_STATUS1,"limitHiGroup.size() %d\n", limitHiGroup.size());

	if (limitLoGroup.size() != sensorsGroup.size() || limitHiGroup.size() != sensorsGroup.size()) {
		ACE_OS::fprintf(stderr,"Error: sensors and limits do not match in config file\n");
		return 0;
	}

	limLo = new double[dim];
	limHi = new double[dim];

	// read through the limits groups
	int groupindex = 1;
	int sensorindex = 0;
	while (groupindex < limitLoGroup.size()) {
		// get sensor name
		//char sensorName[MAX_SENSORNAME_LEN];
		//strcpy(sensorName, sensorsGroup.get(groupindex).asString().c_str());

		// get limits
		limLo[sensorindex] = limitLoGroup.get(groupindex).asDouble();
		limHi[sensorindex] = limitHiGroup.get(groupindex).asDouble();
		IhaDebug::pmesg(DBGL_DEBUG2,"Limits: %d %s Lo: %f Hi %f \n",sensorindex,sensorsGroup.get(groupindex).asString().c_str(),limLo[sensorindex],limHi[sensorindex]);
		sensorindex++;
		groupindex++;
	}
	IhaDebug::pmesg(DBGL_STATUS1,"Read %d sensor limit pairs\n",sensorindex);
	groupindex=0;
	while (groupindex < numImageSensorsX * numImageSensorsY) {
		limLo[sensorindex] = 0;
		limHi[sensorindex] = 255;
		sensorindex++;
		groupindex++;
	}
	IhaDebug::pmesg(DBGL_STATUS1,"Read %d image limit pairs. Total number of limit pairs %d\n",groupindex,sensorindex);

	return dim;

}




