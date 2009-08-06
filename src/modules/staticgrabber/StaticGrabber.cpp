// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */
 
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/Drivers.h>

#include "StaticGrabber.h"
#include <memory.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

// cmake cannot guess c/c++ language with just a .h

bool StaticGrabber::close() { 
    return true; // easy
}

bool StaticGrabber::getRawBuffer(unsigned char *buffer) {
    memcpy(buffer, bayer.getRawImage(), bayer.getRawImageSize());
    return true;
}

// DF2 bayer sequence.
// first row GBGBGB, second row RGRGRG.
bool StaticGrabber::makeSimpleBayer(
        ImageOf<PixelRgb>& src, 
        ImageOf<PixelMono>& bayer) {

    bayer.resize(img.width(), img.height());

    const int w = img.width();
    const int h = img.height();

    int i, j;
    for (i = 0; i < h; i++) {
        PixelRgb *row = (PixelRgb *)img.getRow(i);
        PixelMono *rd = (PixelMono *)bayer.getRow(i);

        for (j = 0; j < w; j++) {

            if ((i%2) == 0) {
                switch (j%4) {
                    case 0:
                    case 2:
                        *rd++ = row->g;
                        row++;
                        break;

                    case 1:
                    case 3:
                        *rd++ = row->b;
                        row++;
                        break;
                }                
            }

            if ((i%2) == 1) {
                switch (j%4) {
                    case 1:
                    case 3:
                        *rd++ = row->g;
                        row++;
                        break;

                    case 0:
                    case 2:
                        *rd++ = row->r;
                        row++;
                        break;
                }                
            }
        }
    }

    return true;
}
