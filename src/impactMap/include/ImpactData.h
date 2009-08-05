// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Chris McCarthy
 *
 */


#ifndef IMPACTDATA_H
#define IMPACTDATA_H

#include <yarp/os/all.h>
using namespace yarp::os;


typedef struct impactData{
   int updated;
   int imageWidth;
   int imageHeight;
   int interestPointX;
   int interestPointY;
   float ttc;
	float approachAngle;
   float impactLocationX;
   float impactLocationY;
} ImpactData;

void packMessage(ImpactData d, Bottle *b);
void unpackMessage(Bottle b, ImpactData *d);

#endif
