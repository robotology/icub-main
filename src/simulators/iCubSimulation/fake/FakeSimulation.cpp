// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#include "FakeSimulation.h"

#include <stdio.h>

void FakeSimulation::simLoop(int h, int w) {
    int n = 600;
    for (int i=0; i<n; i++) {
        printf("Fake simulation cycle %d of %d\n", i+1, n);
        streamer->sendVision();
        yarp::os::Time::delay(0.1);
    }
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
