// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Giorgio Metta and Francesco Orabona
 * email:   francesco.rea@iit.it
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

/**
 * @file ColorVQ.h
 * @brief definition of the color quantization class.
 */

#ifndef _COLORVQ_H_
#define _COLORVQ_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// bounding box type.
class ColorVQ {
private:
    yarp::sig::ImageOf<yarp::sig::PixelInt> tmp1;
    yarp::sig::ImageOf<yarp::sig::PixelInt> tmp2;
    
    int height, width;

public:  
    ColorVQ() {}
    ColorVQ(int x, int y, int fovea) { Resize(x, y, fovea); }
    ~ColorVQ() {}

    void Resize(int x, int y, int fovea);
    void Variance(yarp::sig::ImageOf<yarp::sig::PixelMono> &src, yarp::sig::ImageOf<yarp::sig::PixelInt> &dst, int size);
    void Compactness(yarp::sig::ImageOf<yarp::sig::PixelMono> &src, int fovea, int val, int eps);
    void DominantQuantization(yarp::sig::ImageOf<yarp::sig::PixelBgr> &src, yarp::sig::ImageOf<yarp::sig::PixelBgr> &dst, unsigned char t);
    void DominantQuantization(yarp::sig::PixelBgr src, yarp::sig::PixelBgr &dst, unsigned char t);
};


#endif //_COLORVQ_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
