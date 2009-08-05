// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/ColorSalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::contrib;

bool ColorSalience::open(yarp::os::Searchable& config){
    bool ok = Salience::open(config);
    return ok;
}


void ColorSalience::applyImpl(ImageOf<PixelRgb>& src, 
                          ImageOf<PixelRgb>& dest,
                          ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);
    IMGFOR(src,x,y) {
        PixelRgb pix = src(x,y);
        float lum3 = (pix.r+pix.g+pix.b);
        if (lum3/3.0<60) {
            pix.r = pix.g = pix.b = 0;
        }
        float rn = 255.0 * pix.r / lum3;
        float gn = 255.0 * pix.g / lum3;
        float bn = 255.0 * pix.g / lum3;
        // in theory, apparently, yellow should be (rn+gn)/2 ...
        float yw = (rn+gn)-fabs(rn-gn)-bn;
        yw *= (pix.r>1.5*pix.b);
        yw *= (pix.g>1.5*pix.b);
        float s = 0;
        float r = rn - (gn+bn)/2;
        float g = gn - (rn+bn)/2;
        float b = bn - (rn+gn)/2;
        if (r>s) s = r;
        if (g>s) s = g;
        if (yw>s) s = yw;
        if (b>s) s = b;
        if (s>255) s = 255;
        if (s<0) s = 0;
        if (r<0) r = 0;
        if (g<0) g = 0;
        if (b<0) b = 0;
        if (r>255) r = 255;
        if (g>255) g = 255;
        if (b>255) b = 255;
        sal(x,y) = (unsigned char)s;
        dest(x,y) = PixelRgb((unsigned char)r,
                             (unsigned char)g,
                             (unsigned char)b);
    }
}
