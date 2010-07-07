// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/vis/RuddySalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::vis;

RuddySalience::RuddySalience() {
    transformWeights[0] = 0.21742F;
    transformWeights[1] = -0.36386F;
    transformWeights[2] = 0.90572F;
    transformWeights[3] = 0.00096756F;
    transformWeights[4] = -0.00050073F;
    transformWeights[5] = -0.00287F;
    transformDelta = -50.1255F;
}

bool RuddySalience::open(yarp::os::Searchable& config){

    bool ok = Salience::open(config);
    if (!ok)
        return false;

    return true;
}

float RuddySalience::eval(PixelRgb& pix) {
    float r = pix.r;
    float g = pix.g;
    float b = pix.b;
    bool mask = (r>g) && (r<3*g) && (r>0.9*b) && (r<3*b) && (r>70);
    float judge = 0;
    if (mask) {
        float r2 = r*r;
        float g2 = g*g;
        float b2 = b*b;
        judge = 
            r*transformWeights[0] +
            g*transformWeights[1] +
            b*transformWeights[2] +
            r2*transformWeights[3] +
            g2*transformWeights[4] +
            b2*transformWeights[5] +
            transformDelta;
        judge *= 1.5;
    }
    if (judge<0) judge = 0;
    if (judge>255) judge = 255;
    return judge;
}

void RuddySalience::applyImpl(ImageOf<PixelRgb>& src, 
                          ImageOf<PixelRgb>& dest,
                          ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);
    IMGFOR(src,x,y) {
        PixelRgb pix = src(x,y);
        sal(x,y) = eval(pix);
    }

    dest.copy(sal);
}
