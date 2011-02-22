// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
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

#include "FakeSimulationBundle.h"

#include "FakeLogicalJoints.h"
#include "FakeWorldManager.h"
#include "FakeSimulation.h"

bool FakeSimulationBundle::onBegin() {
    return true;
}

LogicalJoints *FakeSimulationBundle::createJoints(RobotConfig& config) {
    config.setFlags();
    return new FakeLogicalJoints;
}

WorldManager *FakeSimulationBundle::createWorldManager(RobotConfig& config) {
    return new FakeWorldManager;
}

Simulation *FakeSimulationBundle::createSimulation(RobotConfig& config) {
    return new FakeSimulation;
}

bool FakeSimulationBundle::onEnd() {
    return true;
}


