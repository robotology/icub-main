// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/vis/MotionSalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace iCub::vis;

bool MotionSalience::open(yarp::os::Searchable& config){

    bool ok = Salience::open(config);
    if (!ok)
        return false;

    targetLen = config.check("gap", yarp::os::Value(5),
                                 "time constant expressed in frames").asInt();

    return true;
}

void MotionSalience::applyImpl(ImageOf<PixelRgb>& src, 
                           ImageOf<PixelRgb>& dest,
                           ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);
    sal.copy(src);

    history.push_back(src);

    ImageOf<PixelRgb>& past = history.front();

    float mx = 0;
    float my = 0;
    float mc = 0;

    IMGFOR(src,x,y) {
        PixelRgb& pix1 = src(x,y);

        PixelRgb& pix0 = past(x,y);
        float dr = pix0.r-pix1.r;
        float dg = pix0.g-pix1.g;
        float db = pix0.b-pix1.b;
        float v = dr*dr + dg*dg + db*db;

        v = v/10;
        float v2 = v*10;
        float v3 = v/10;
        if (v>255) { v = 255; }
        sal(x,y) = v;
        PixelRgb& pixd = dest(x,y);
        if (v2>255) { v2 = 255; }
        if (v3>255) { v3 = 255; }
        pixd.r = (unsigned char)v;
        pixd.g = (unsigned char)v2;
        pixd.b = (unsigned char)0;
    }

    if (history.size()>targetLen) {
        history.pop_front();
    }
}
