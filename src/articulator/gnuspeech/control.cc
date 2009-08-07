// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "control.h"
#include "yarpy.h"

#include <stdio.h>

using namespace yarp::os;
using namespace yarp::dev;

int xmap[] = { 0, 0, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
int ymap[] = { 0, 1, 0, 0, 1, 2, 3, 0, 1, 2, 3, 4, 5, 6, 7, 0 };

static const char *names[] = {
    "glottal",
    "pitch",

    "aspiration",

    "frication",
    "frication_position",
    "frication_center",
    "frication_width",
    
    "tube_0",
    "tube_1",
    "tube_2",
    "tube_3",
    "tube_4",
    "tube_5",
    "tube_6",
    "tube_7",

    "velum"
};

static void transform(int j, int& x, int& y) {
    x = xmap[j];
    y = ymap[j];
}

bool ArticulatorMotor::open(Searchable& config) {
    nj = 16;
    return true;
}

bool ArticulatorMotor::close() {
    return true;
}

bool ArticulatorMotor::positionMove(int j, double ref) {
    if (j>=0 && j<nj) {
        int x, y;
        transform(j,x,y);
        setParamTarget(x,y,ref);
        return true;
    }
    return false;
}

bool ArticulatorMotor::positionMove(const double *refs) {
    bool ok = true;
    for (int j=0; j<nj; j++) {
        bool alt = positionMove(j,refs[j]);
        ok = ok && alt;
    }
    return ok;
}


bool ArticulatorMotor::getAxisName(int axis, yarp::os::ConstString& name) {
    if (axis>=0 && axis<nj) {
        name = names[axis];
        return true;
    }
    return false;
}


bool ArticulatorMotor::getLimits(int axis, double *min, double *max) {
    ConstString name;
    getAxisName(axis,name);
    double top = 100;
    if (name == "velum") {
        top = 1;
    }
    *min = 0;
    *max = top;
    return true;
}


bool ArticulatorMotor::getEncoder(int j, double *v) {
    if (j>=0 && j<nj) {
        int x, y;
        transform(j,x,y);
        *v = getParam(x,y);
        return true;
    }
    return false;
}


bool ArticulatorMotor::getEncoders(double *encs) {
    bool ok = true;
    for (int j=0; j<nj; j++) {
        bool alt = getEncoder(j,&(encs[j]));
        ok = ok && alt;
    }
    return ok;
}
