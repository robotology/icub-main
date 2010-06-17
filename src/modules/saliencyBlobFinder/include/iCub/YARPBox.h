// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARPBOX__
#define __YARPBOX__

#include <yarp/sig/Image.h>//#include <yarp/YARPImage.h>
//#include <yarp/YARPlogpolar.h>
//#include <yarp/YARPBabybotHeadKin.h>
#include <yarp/math/Math.h>//#include <yarp/YARPMath.h>
//#include <yarp/YARPIntegralImage.h>
//#include "YARPColorVQ.h"

#define PI 3.14

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
    
    PixelMono meanRG;
    PixelMono meanGR;
    PixelMono meanBY;

    PixelBgr meanColors;
    
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
