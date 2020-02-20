// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
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

#include "OdeWorldManager.h"

#include "OdeInit.h"
#include "iCub_Sim.h"

#ifndef DENSITY
#define DENSITY (1.0)		// density of all objects
#endif

using namespace yarp::os;

void OdeWorldManager::clear() {
    OdeInit& odeinit = OdeInit::get();
    for (int x =0; x < a; x++)
        dTriMeshXDestroy(odeinit._wrld->trimesh[x]);
}

