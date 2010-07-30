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
#include <iCub/disparityTool.h>

DisparityTool::DisparityTool() {

    _shiftFunction	  = NULL;
    _shiftFunctionInv = NULL;
    _shiftMap		  = NULL;
    _corrFunct		  = NULL;
    _ssdFunct         = NULL;
    _count			  = NULL;
    _maxCount = 0;
    rPtr              = NULL; 
    lPtr              = NULL;
    init(_srho, _stheta, 1, _overlap, _xsize, _ysize, _xsizeR, _ysizeR, _srho);
}

DisparityTool::~DisparityTool() {

    if (_shiftFunction != NULL)
        delete [] _shiftFunction;

    if (_shiftMap != NULL)
        delete [] _shiftMap;

    if (_corrFunct != NULL)
        delete [] _corrFunct;

    if (_ssdFunct != NULL)
        delete [] _ssdFunct;

    if (_count != NULL)
        delete [] _count;

    if (_shiftFunctionInv == NULL)
        delete [] _shiftFunctionInv;

    delete[] lPtr;
    delete[] rPtr;
}

void DisparityTool::setSize( ImageOf<PixelRgb> *inLImg ) {

    imageSize = inLImg->getPixelSize();
    lPtr = new unsigned char [ _img.Size_LP * imageSize ];
    rPtr = new unsigned char [ _img.Size_LP * imageSize ];
}

void DisparityTool::init( int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr, int actR) {
	
    _img = SetParam( rho, theta, mode, overlap, xo, yo, xr, yr );
    _actRings = actR;
    AllocateVectors();	
    LoadShiftMap();
    computeCountVector( _count );
}


void DisparityTool::AllocateVectors() {	

    if (_shiftFunction != NULL)
        delete [] _shiftFunction;
    if (_shiftMap != NULL)
        delete [] _shiftMap;
    if (_corrFunct != NULL)
        delete [] _corrFunct;
    if (_ssdFunct != NULL)
        delete [] _ssdFunct;
    if (_count != NULL)
        delete [] _count;

    char File_Name [256];
    FILE * fin;
    int n;
    int cSize = (int)_min((double)_img.Size_X_Orig, (double)_img.Size_Y_Orig);
    sprintf(_path,"%s",".");
    sprintf(File_Name,"%s%dx%d_%dx%d_ShiftMap.gio",_path, cSize, cSize, _img.Size_Theta, _img.Size_Rho);

    if ((fin = fopen(File_Name,"rb")) == NULL) {
        Build_Shift_Table(_img, _path);
        fin = fopen(File_Name,"rb");
    }

    fread(&n,sizeof(int),1,fin);
    fclose(fin);

    _shiftFunction = new double[n];
    _shiftMap = new int[n*_img.Size_LP];
    _count = new int[n];
    _corrFunct = new double[n];
    _ssdFunct = new double[n];
}


void DisparityTool::LoadShiftMap() {
    _shiftLevels = Load_Shift_Table(_img, _shiftMap, _shiftFunction, _path);
    _shiftMax = _shiftLevels - 1;
    _shiftMin = 0;


    if (_shiftFunctionInv != NULL)
        delete [] _shiftFunctionInv;
    
    int tmpIndex;
    double dmax, dmin;
    dmax = find_max_value(_shiftFunction, _shiftLevels);
    dmin = find_min_value(_shiftFunction, _shiftLevels);

    _shiftFunctionInv = new int[(int)(dmax-dmin)+1];
    for (int i = 0; i <= dmax-dmin; i++) {
        for (int j = 0; j < _shiftLevels; j++) {
            tmpIndex = j;
            if (_shiftFunction[j] >= i+dmin)
            break;
        }
    _shiftFunctionInv[i] = tmpIndex;
    }	
}


void DisparityTool::makeHistogram(ImageOf<PixelMono>& hImg) {
    int i,j;
    int height = hImg.height();
    int width = hImg.width();

    unsigned char bgColor = 0;
    unsigned char hColor = 190;
    unsigned char lineColor = 255;

    unsigned char * hist = new unsigned char [height*width*hImg.getPixelSize()];
    img2unpaddedVect(hist, hImg);

    int offset = (width - _shiftLevels + 1)/2;

    for (j = 0; j < height*width; j++)
        hist[j] = bgColor;

    for (i = 0; i < _shiftLevels-1; i++) {
    
        if ((i+offset >=0)&&(i+offset<width)) {

            for (j = height-(int)(height*_corrFunct[i]); j < height; j++)
                hist[(j*width+i+offset)] = hColor;
        }
    }

    for (i = 0; i < 3; i++) {

        if ((_maxShifts[i].index+offset >=0)&&(_maxShifts[i].index+offset<width)) {
            for (j = height-(int)(height*_corrFunct[_maxShifts[i].index]); j < height; j++)
                hist[(j*width+_maxShifts[i].index+offset)] = lineColor;
        }
    }

    //Drawing Zero Reference
    for (j = 0; j < height; j += height/9) {
        for (int k = 0; k < height/18; k++) {
    	    hist[((j+k)*width + _shiftLevels/2 + offset)] = lineColor;
        }
    }

	//Drawing Limits (dark gray sides)
/*	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = 120;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = 120;
	}/***/

    //Partially inverted color
    for (int y = 0; y < height; y++) {
        for (int x1 = 0; x1 < _shiftMin; x1++)
            hist[(y*width+x1+offset)] = hist[(y*width+x1+offset)]+100;
        for (int x2 = width - 1; x2 > _shiftMax; x2--)
            hist[(y*width+x2+offset)] = hist[(y*width+x2+offset)]+100;
    }

	//Drawing Limits (inverted colors sides)
/*	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = -hist[(y*width+x1+offset)]+255;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = -hist[(y*width+x2+offset)]+255;
	}/***/

    unpaddedVect2img(hist, hImg);
    delete [] hist;
}

void DisparityTool::computeCountVector(int *count)
{	
    int tIndex;
    int k,i,j,k1, i2;
    if (count == NULL)
        count = new int [_shiftLevels];
    for (k = 0; k < _shiftLevels; k++) {
        k1 = k * _img.Size_LP;
        count[k] = 0;
        for (j=0; j<_actRings; j++) {
            tIndex = j*_img.Size_Theta;
            for (i=0; i<_img.Size_Theta; i++) {
                i2 = _shiftMap[k1 + tIndex+i];
                if (i2 > 0)
                count[k]++;
            }
        }
    }

    // find max
    _maxCount = 0;
    for (k = 0; k < _shiftLevels; k++) {
        if (count[k] > _maxCount)
        _maxCount = count[k];
    }
}

void DisparityTool::findShiftMax(const double *corr)
{
    // search max
    double corrMax = 0;
    int ret = 0;
    int l;


    for(l = _shiftMin; l <= _shiftMax; l++) {
        if (corr[l] > corrMax) {
            corrMax = corr[l];
            ret = l;
        }
    }

    _maxShifts[0].index = ret;
    _maxShifts[0].corr = corrMax;
    _maxShifts[0].disp = shiftToDisparity(ret);
}


void DisparityTool::findSecondMaxes(const double *corr, int posMax)
{
    // search max
    double corrMax;
    double threshold = 0.005;
    int max1;
    int max2;

    int ret = 0;
    int l;

    //Searching in left direction
    l = posMax-1;
    while( (l > _shiftMin) && (corr[l] <= corr[l+1]) )
        l--;	
    corrMax = corr[l];
    max1 = l;
    for(; l >= _shiftMin; l--) {
        if (corr[l] > (corrMax + threshold)) {
            corrMax = corr[l];
            max1 = l;
        }
    }

    //Searching in right direction
    l = posMax+1;
    while( (l < _shiftMax) && (corr[l-1] >= corr[l]) )
        l++;
    corrMax = corr[l];
    max2 = l;
    for(; l <= _shiftMax; l++) {
        if (corr[l] > (corrMax + threshold)) {
           corrMax = corr[l];
            max2 = l;
        }
    }

    if (corr[max1]>corr[max2]) {
        _maxShifts[1].index = max1;
        _maxShifts[2].index = max2;
    }
    else {
        _maxShifts[2].index = max1;
        _maxShifts[1].index = max2;
    }

    _maxShifts[3].index = zeroShift();

    for (int k = 1; k < 4; k++) {
        _maxShifts[k].corr = corr[_maxShifts[k].index];
        _maxShifts[k].disp = shiftToDisparity(_maxShifts[k].index);
    }
}


shift_Struct DisparityTool::filterMaxes() {
    int maxIndex;
    int closest;

    float threshold = 0.005f;
    float threshold2 = 0.05f;
    float vals[__nMaxes];
    for (int i = 0; i < __nMaxes - 1; i++) {
        vals[i] = (float)_maxShifts[i].corr;
        if ( !isWithin(_maxShifts[i].index, _shiftMax, _shiftMin) || (vals[i] < threshold) )
            vals[i] = -1;
    }
    vals[__nMaxes - 1] = 0;

    // find closest
    closest = 0;
    int zs = zeroShift();
    for (int ii = 0; ii < (__nMaxes - 1); ii++) {
        if (abs(_maxShifts[ii].index-zs) < (_maxShifts[closest].index-zs))
        closest = ii;
    }

    // find higher, starting from closest, so that if no best matches are found we go 
    // to the closest peak
    maxIndex = closest;
    for (int iii = 0; iii < __nMaxes; iii++) {
        if (vals[iii] > (vals[maxIndex] + threshold2) )
        maxIndex = iii;
    }

    return _maxShifts[maxIndex];
}


int DisparityTool::computeDisparityCorrRGBsum(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step) {

    img2unpaddedVectMultiple(lPtr, inLImg, rPtr, inRImg);

    //Correlation Function Computation
    for (k = 0; k < _shiftLevels; k++) {

        average_Lr = 0;
        average_Lg = 0;
        average_Lb = 0;
        average_Rr = 0;
        average_Rg = 0;
        average_Rb = 0;

        R_corr = 0;
        G_corr = 0;
        B_corr = 0;

        pixelL = 0;
        pixelR = 0;

        k1 = k *_img.Size_LP;

        for (j = 0; j < _actRings; j++) {

            for (i = 0; i < _img.Size_Theta; i++) {

                iR = _shiftMap[k1 + j *_img.Size_Theta + i];
                iL = 3 * (j*_img.Size_Theta + i);

                if (iR > 0) {
                    average_Lr += lPtr[iL];
                    average_Rr += rPtr[iR];
                    average_Lg += lPtr[iL+1];
                    average_Rg += rPtr[iR+1];
                    average_Lb += lPtr[iL+2];
                    average_Rb += rPtr[iR+2];
                }
            }
        }

        if (_count[k] != 0) {
            average_Lr /= _count[k];
            average_Rr /= _count[k];
            average_Lg /= _count[k];
            average_Rg /= _count[k];
            average_Lb /= _count[k];
            average_Rb /= _count[k];
        }

        numr   = 0;
        den_Lr = 0;
        den_Rr = 0;
        numg   = 0;
        den_Lg = 0;
        den_Rg = 0;
        numb   = 0;
        den_Lb = 0;
        den_Rb = 0;

        for (j = _actRings - 1; j >= 0; j-=step) {

            for (i = _img.Size_Theta - 1; i >= 0; i-=step) {

                iR = _shiftMap[k1 + j*_img.Size_Theta + i];
                iL = 3 * (j*_img.Size_Theta + i);

                if (iR > 0) {
                //Red
                    pixelL = lPtr[iL] - average_Lr;
                    pixelR = rPtr[iR] - average_Rr;
                    numr   += (pixelL * pixelR);
                    den_Lr += (pixelL * pixelL);
                    den_Rr += (pixelR * pixelR);
                    //Green
                    pixelL = lPtr[iL+1] - average_Lg;
                    pixelR = rPtr[iR+1] - average_Rg;
                    numg   += (pixelL * pixelR);
                    den_Lg += (pixelL * pixelL);
                    den_Rg += (pixelR * pixelR);
                    //Blue
                    pixelL = lPtr[iL+2] - average_Lb;
                    pixelR = rPtr[iR+2] - average_Rb;
                    numb   += (pixelL * pixelR);
                    den_Lb += (pixelL * pixelL);
                    den_Rb += (pixelR * pixelR);

                }
            }
        }

        R_corr = numr / sqrt(den_Lr * den_Rr + 0.00001);
        G_corr = numg / sqrt(den_Lg * den_Rg + 0.00001);
        B_corr = numb / sqrt(den_Lb * den_Rb + 0.00001);
        _corrFunct[k] = (R_corr + G_corr + B_corr) / 3.0;
        _corrFunct[k] *= _count[k];
        _corrFunct[k] /= _maxCount;

    }

    findShiftMax(_corrFunct);
    findSecondMaxes(_corrFunct, _maxShifts[0].index);
    ret = filterMaxes();

    //double endtime = Time::now();
    //double result = endtime - TimeStart;
    //TimeStart = Time::now(); 
    //cout << "TIME " << result << endl;

    return (int)ret.disp;
}
