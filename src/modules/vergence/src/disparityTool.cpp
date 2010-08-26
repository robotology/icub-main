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
#include <yarp/os/Time.h>

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

    if (_shiftFunctionInv != NULL)
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
    cout << "RINGS" << actR << endl;
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

    size_t discard=fread(&n,sizeof(int),1,fin);
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

    //design the area
    for (i = 0; i < _shiftLevels-1; i++) {
    
        if ((i+offset >=_shiftMin)&&(i+offset< _shiftMax)) {

            for (j = height-(int)(height*_corrFunct[i]); j < height; j++)
                hist[(j*width+i+offset)] = hColor;
        }
    }
    //design the maxes
    for (i = 0; i < 3; i++) {

        if ((_maxShifts[i].index+offset >=_shiftMin)&&(_maxShifts[i].index+offset<_shiftMax)) {
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

  //  cout << "ShiftLevels: " << _shiftLevels << endl;
  //  cout << "ActRings: " << _actRings << endl;
 //   cout << "Theta: " << _img.Size_Theta << endl;
   // cout << "Step: " << step << endl;

   // TimeStart = Time::now();
    
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
     //   cout << "Loop1 time: " << (Time::now() - TimeStart) << endl;

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
       // cout << "Loop2 time: " << (Time::now() - TimeStart) << endl;

    }
    //cout << "Loops time: " << (Time::now() - TimeStart) << endl;

    findShiftMax(_corrFunct);
    findSecondMaxes(_corrFunct, _maxShifts[0].index);
    ret = filterMaxes();

    //cout << "Final time: " << (Time::now() - TimeStart) << endl;
    cout << "index " << ret.index << " corr " << ret.corr << " disp " << ret.disp << endl; 
    return (int)ret.disp;
}

int DisparityTool::computeMono (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, double value) {

    //TimeStart = Time::now();
    cout << "ShiftLevels: " << _shiftLevels << endl;
    cout << "ActRings: " << _actRings << endl;
    cout << "Theta: " << _img.Size_Theta << endl;
    
    img2unpaddedVectMultiple(lPtr, inLImg, rPtr, inRImg);

    int i,j,k, k1;
    int i2,i1;//iR,iL
    double numr   = 0;
    double den_1r = 0;
    double den_2r = 0;
    double numg   = 0;
    double den_1g = 0;
    double den_2g = 0;
    double numb   = 0;
    double den_1b = 0;
    double den_2b = 0;

    double sigma2 = 0;
    double sigma1 = 0;
    double grayAv1 = 0;
    double grayAv2 = 0;

    //YarpPixelRGBFloat pixel1;
    //YarpPixelRGBFloat pixel2;
    PixelRgb pixel1, pixel2;
    
    
    double sum1R = 0;
    double sum2R = 0;
    double sum1G = 0;
    double sum2G = 0;
    double sum1B = 0;
    double sum2B = 0;
    double nElem = 0;

    //unsigned char * fullPtr,* fovPtr;
   // fullPtr = (unsigned char*)inRImg.getRawImage();
   // fovPtr = (unsigned char*)inLImg.getRawImage();
    int tIndex;

   // int AddedPadSize = computePadSize(_img.Size_Theta*_img.LP_Planes,_img.padding) - _img.Size_Theta*_img.LP_Planes;

    for (k=0; k<_shiftLevels; k++) {
        k1 = k * _img.Size_LP; //Positioning on the table

        numr   = 0;
        den_1r = 0;
        den_2r = 0;
        numg   = 0;
        den_1g = 0;
        den_2g = 0;
        numb   = 0;
        den_1b = 0;
        den_2b = 0;

        sum1R = 0;
        sum2R = 0;
        sum1G = 0;
        sum2G = 0;
        sum1B = 0;
        sum2B = 0;
        nElem = 0;

        sigma1 = 0;
        sigma2 = 0;
        grayAv1 = 0;
        grayAv2 = 0;

      //  fullPtr = (unsigned char*)inRImg.getRawImage();
      //  fovPtr = (unsigned char*)inLImg.getRawImage();

        if (_count[k] == _maxCount) {
            for (j=0; j<_actRings; j++) {
                tIndex = j*_img.Size_Theta;
                for (i=0; i<_img.Size_Theta; i++) {
                    i2 = _shiftMap[k1 + tIndex+i];
                    i1 = 3 * (tIndex+i);

                    if (i2 > 0) {
                        pixel1.r = *lPtr++;
                        pixel2.r = rPtr[i2];
                        pixel1.g = *lPtr++;
                        pixel2.g = rPtr[i2+1];
                        pixel1.b = *lPtr++;
                        pixel2.b = rPtr[i2+2];

                        double gray1 = (pixel1.r + pixel1.g+pixel1.b)/3.0;
                        double gray2 = (pixel2.r+pixel2.g+pixel2.b)/3.0;

                        numr   += (pixel1.r * pixel2.r);
                        sum1R += pixel1.r;
                        sum2R += pixel2.r;
                        den_1r += (pixel1.r * pixel1.r);
                        den_2r += (pixel2.r * pixel2.r);

                        numg   += (pixel1.g * pixel2.g);
                        sum1G += pixel1.g;
                        sum2G += pixel2.g;
                        den_1g += (pixel1.g * pixel1.g);
                        den_2g += (pixel2.g * pixel2.g);

                        numb   += (pixel1.b * pixel2.b);
                        sum1B += pixel1.b;
                        sum2B += pixel2.b;
                        den_1b += (pixel1.b * pixel1.b);
                        den_2b += (pixel2.b * pixel2.b);

                        sigma1 += gray1*gray1;
                        sigma2 += gray2*gray2;
                        grayAv1 += gray1;
                        grayAv2 += gray2;

                        nElem++;
                    }
                   else rPtr +=3;
                }
               // fovPtr+=AddedPadSize;
            }

            double tmpR;
            double tmpG;
            double tmpB;
            tmpR = (den_1r-sum1R*sum1R/nElem)*(den_2r-sum2R*sum2R/nElem);
            tmpG = (den_1g-sum1G*sum1G/nElem)*(den_2g-sum2G*sum2G/nElem);
            tmpB = (den_1b-sum1B*sum1B/nElem)*(den_2b-sum2B*sum2B/nElem);

            _corrFunct[k] = 0;
            if (tmpR>0){
                cout << "tmpR" << endl;
                _corrFunct[k] += (numr-sum1R*sum2R/nElem)/sqrt(tmpR);
            }
            if (tmpG>0){
                cout << "tmpG" << endl;
                _corrFunct[k] += (numg-sum1G*sum2G/nElem)/sqrt(tmpG);
            }
            if (tmpB>0){
                cout << "tmpB" << endl;
                _corrFunct[k] += (numb-sum1B*sum2B/nElem)/sqrt(tmpB);
            }
            _corrFunct[k] /= 3.0;

      //   _std2[k] = (1/nElem)*(sigma2-grayAv2*grayAv2/nElem)/(128*128);
       //  _std1[k]= (1/nElem)*(sigma1-grayAv1*grayAv1/nElem)/(128*128);
        }
        else{
        _corrFunct[k] = 0;
     //   _std2[k] = 0;
     //   _std1[k] = 0;
        }
    }

    findShiftMax(_corrFunct);
    findSecondMaxes(_corrFunct, _maxShifts[0].index);
    ret = filterMaxes();
    
    cout << "index " << ret.index << " corr " << ret.corr << " disp " << ret.disp << endl; 

    // cout << "Final time: " << (Time::now() - TimeStart) << endl;
    return (int)ret.disp;
}
