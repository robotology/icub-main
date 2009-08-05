// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Basilio Noris
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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

#ifndef _ABSORIENT_H_
#define _ABSORIENT_H_

#include <cmath>
#include <vector>
#include "quatMath.h"

namespace AO
{
	void AbsOrient(std::vector<vec3> L, std::vector<vec3> R, vec3 *rot=NULL, float *theta=NULL, float *scale=NULL, vec3 *translation=NULL);
	void AbsOrient(vec3 *L, vec3 *R, int pointCnt, vec3 *rot=NULL, float *theta=NULL, float *scale=NULL, vec3 *translation=NULL);
}

#endif // _ABSORIENT_H_
