// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:    paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
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
#include "WorldOp.h"

#include <stdio.h>
#include <yarp\os\Log.h>

void show(const WorldOpTriplet& x) {
    if (!x.valid) {
        yDebug("not set");
    } else {
        yDebug("%g %g %g", x.x[0], x.x[1], x.x[2]);
    }
    yDebug("\n");
}

void show(const WorldOpScalar& x) {
    if (!x.valid) {
        yDebug("not set");
    } else {
        yDebug("%g", x.val);
    }
    yDebug("\n");
}

void show(const WorldOpFlag& x, 
          const char *yes="True", 
          const char *no="False") {
    if (!x.valid) {
        yDebug("not set");
    } else {
        yDebug("%s", x.setting?yes:no);
    }
    yDebug("\n");
}

void show(const WorldOpIndex& x) {
    if (!x.valid) {
        yDebug("not set");
    } else {
        yDebug("%d", x.index);
    }
    yDebug("\n");
}

void show(const WorldOpName& x) {
    if (!x.valid) {
        yDebug("not set");
    } else {
        yDebug("%s", x.name.c_str());
    }
    yDebug("\n");
}

void WorldOp::show() const {
    yDebug("Operation:\n");
    yDebug("  cmd?: %s\n", yarp::os::Vocab::decode(cmd).c_str());
    yDebug("  kind: ");
    ::show(kind);
    yDebug("  name: ");
    ::show(name);
    yDebug("  dynamic: ");
    ::show(dynamic);
    yDebug("  location: ");
    ::show(location);
    yDebug("  size: ");
    ::show(size);
    yDebug("  color: ");
    ::show(color);
    yDebug("  rotation: ");
    ::show(rotation);
    yDebug("  radius: ");
    ::show(radius);
    yDebug("  length: ");
    ::show(length);
    yDebug("  active: ");
    ::show(active);
    yDebug("  index: ");
    ::show(index);
    yDebug("  collide: ");
    ::show(collide);
    yDebug("  parameter: ");
    ::show(parameter);
    yDebug("  rightHanded?: ");
    ::show(rightHanded,"right","left");
    yDebug("  modelName: ");
    ::show(modelName);
    yDebug("  modelTexture: ");
    ::show(modelTexture);
}

void WorldResult::show() {
    yDebug("Result:\n");
    yDebug("  success: %s\n", success?"true":"false");
    yDebug("  msg: %s\n", msg.c_str());
    yDebug("  location: ");
    ::show(location);
    yDebug("  color: ");
    ::show(color);
    yDebug("  path: ");
    ::show(path);
 }

