// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:   paulfitz@alum.mit.edu
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

#include "FakeSimulation.h"

#include <stdio.h>

void FakeSimulation::simLoop(int h, int w) {
    int n = 600;
    for (int i=0; i<n; i++) {
        if ((i+1)%10==0) {
            printf("Fake simulation cycle %d of %d\n", i+1, n);
        }
        streamer->sendVision();
        yarp::os::Time::delay(0.1);
    }
}

bool FakeSimulation::getTrqData(yarp::os::Bottle data)
{
    return true;
}


bool FakeSimulation::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img) {
    img.resize(320,240);
    img.zero();
    for (int i=0; i<img.width(); i++) {
        img(i,at).b = 255;
    }
    at = (at+1) % img.height();
    return true;
}
