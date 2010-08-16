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
 * @file YARPBox.h
 * @brief definition of the YARP box class (bounding box). This is very old code by Giorgio from 2001 or so.
 */

#ifndef __YARPBOX__
#define __YARPBOX__

#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>

class YARPBox {
public:
    YARPBox() { valid = false; }

    ~YARPBox() {}

    bool operator==(const YARPBox& x)
    {
        bool ret = true;
        ret &= (xmax == x.xmax);
        ret &= (xmin == x.xmin);
        ret &= (ymax == x.ymax);
        ret &= (ymin == x.ymin);
        ret &= (cmax == x.cmax);
        ret &= (cmin == x.cmin);
        ret &= (rmax == x.rmax);
        ret &= (rmin == x.rmin);
        return ret;
    }

    bool valid;

    // logpolar
    int cmax, rmax;
    int cmin, rmin;
    int areaLP;
    
    // cartesian
    int xmax, ymax;
    int xmin, ymin;
    double areaCart;
    int xsum, ysum;
    double centroid_x;
    double centroid_y;

    bool cutted;
    
    long int id;

    //__OLD//bool edge;

    // I use long to allow blob of dimension>255, but in MSVC int is = long!
    long int rgSum;
    long int grSum;
    long int bySum;
    
    unsigned long int rSum;
    unsigned long int gSum;
    unsigned long int bSum;
    
    yarp::sig::PixelMono meanRG;
    yarp::sig::PixelMono meanGR;
    yarp::sig::PixelMono meanBY;

    yarp::sig::PixelBgr meanColors;
    
    /*char cRG;
    char cGR;
    char cBY;*/

    int cRG;
    int cGR;
    int cBY;

    double salienceBU;
    double salienceTD;
    double salienceTotal;

    double elev;
    double az;

    double cmp;
    double ect;

    int indexCutted;
};

#endif


//----- end-of-file --- ( next line intentionally left blank ) ------------------
