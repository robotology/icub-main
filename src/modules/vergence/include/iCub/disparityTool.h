// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
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

#if !defined(AFX_DISPARITY_H__E3019D2A_4BC3_4AD0_8355_AAA601391249__INCLUDED_)
#define AFX_DISPARITY_H__E3019D2A_4BC3_4AD0_8355_AAA601391249__INCLUDED_

#include <time.h>
#include <yarp/sig/Image.h>
// std
#include <stdio.h>
#include <iostream>
#include <iCub/rcLogPolarAuxFunc.h>

#ifdef YARP_HAS_PRAGMA_ONCE
#	pragma once
#endif

const int __nMaxes = 4;

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace _logpolarParams;

class DisparityTool {
public:
    DisparityTool();
    virtual ~DisparityTool();

    inline int getShiftLevels() { 
        return _shiftLevels; 
    }
    inline int zeroShift() { 
        return (_shiftLevels/2); 
    }
    inline double getCorrValue(int n) { 
        return _corrFunct[n]; 
    }
    inline double getSSDValue(int n) { 
        return _ssdFunct[n]; 
    }
    inline void setInhibition(int max, int min) { 
        _shiftMax = disparityToShift((double)max); _shiftMin = disparityToShift((double)min); 
    }
    inline int getLimitsMax() { 
        return _shiftMax; 
    }
    inline int getLimitsMin() { 
        return _shiftMin; 
    }	
    inline void setRings(int r) { 
        _actRings = r; computeCountVector(_count); 
    }
    inline shift_Struct getMax(int n = 0) { 
        return _maxShifts[n]; 
    }
    inline double shiftToDisparity(int shift) { 
        return _shiftFunction[shift]; 
    }
    inline int disparityToShift(double disp) { 
        double min = find_min_value(_shiftFunction, _shiftLevels);
        double max = find_max_value(_shiftFunction, _shiftLevels);
        int index = (int)(disp - min);
        if (index < 0)
            index = 0;
        else if (index > (max-min))
            index = (int)(max-min);
        return _shiftFunctionInv[index]; 
    }

    int computeDisparityCorrRGBsum (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeMono (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, double value);
    /*int computeDisparityCorrRGBsum2 (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeDisparityCorrRGBsum3 (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeDisparityCorrRGBprod (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeDisparitySSD_RGBsum (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeDisparitySSD_RGBprod (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeDisparitySSDAvg_RGBsum (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
    int computeDisparitySSDAvg_RGBprod (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);*/
    void makeHistogram(ImageOf<PixelMono> & hImg);
    void makeNormSSDHistogram(ImageOf<PixelMono> & hImg);
    void Remap(ImageOf<PixelRgb>  & lpIn, ImageOf<PixelRgb>  & cartOut);
    void init(int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr, int actR);
    void print_Data(char *filename);
    void print_dataBlock(FILE *fout, shift_Struct *shifts, int numb);
    void print_Data_SSD(char *filename);
    void print_dataBlock_SSD(FILE *fout, shift_Struct *shifts, int numb);

    shift_Struct filterMaxes();
    void findShiftMax(const double *ssd);
    void findSecondMaxes(const double *ssd, int posMin);
    void computeCountVector(int *count);
    void LoadShiftMap();
    void AllocateVectors();
    void setSize( ImageOf<PixelRgb> *inRImg );

private:

    Image_Data _img;
    int imageSize;
    unsigned char *rPtr, *lPtr;

    int   _shiftLevels;
    int * _shiftMap;
    double * _shiftFunction;
    int * _shiftFunctionInv;
    double * _corrFunct;
    double *_ssdFunct;
    shift_Struct _maxShifts[__nMaxes];

    int *_count;
    int _maxCount;

    int _actRings;

    int _shiftMax;
    int _shiftMin;

    char _path[256];

    double TimeStart;
    int i,j,k,k1;
    int iL,iR;
    shift_Struct ret;
    double average_Lr, average_Lg, average_Lb, average_Rr, average_Rg, average_Rb;

    double numr, den_Lr, den_Rr, numg, den_Lg, den_Rg, numb, den_Lb, den_Rb, pixelL, pixelR;
    
    double R_corr, G_corr, B_corr;
};

#endif // !defined(AFX_DISPARITY_H__E3019D2A_4BC3_4AD0_8355_AAA601391249__INCLUDED_)
