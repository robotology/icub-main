#ifndef IHA_LIMITS__H__
#define IHA_LIMITS__H__

/*
 * Copyright (C) 2006 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
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

#include <iCub/iha/debug.h>
#include <iCub/iha/mem_util.h> // from ControlBoardInterfaces.inl


namespace iCub {
	namespace iha {
		class Limits;
	}
}
using namespace iCub::iha;

/** 
 * @ingroup icub_iha_ExperienceMetricSpace
 *
 * \brief Sensor Limits for calculating the normalized value of a sensor.
 */
class iCub::iha::Limits {
public:
	Limits() {
	}
	~Limits() {
		delete [] limLo;
		delete [] limHi;
	}
	double normalize(double value, int index);
	bool inRange(double value, int index);
	int readLimits(yarp::os::Searchable &config, int x, int y);

private:
	// limits
	double* limLo;
	double* limHi;
};

#endif
