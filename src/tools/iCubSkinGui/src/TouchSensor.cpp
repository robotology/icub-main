// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#define VERY_FAR 1E+20
double TouchSensor::dXmin= VERY_FAR;
double TouchSensor::dXmax=-VERY_FAR;
double TouchSensor::dYmin= VERY_FAR;
double TouchSensor::dYmax=-VERY_FAR;

int TouchSensor::m_maxRange=0;
double* TouchSensor::Exponential=0;
