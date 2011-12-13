// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/vis/AdvancedDirectionalSalience.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::vis;

AdvancedDirectionalSalience::AdvancedDirectionalSalience(){
    MAX_SALIENCE = 255.0f;
    _dirs = NULL;
    _sigs = NULL;
    _wavs = NULL;
    _imgInRgbResized = NULL;
    _imgInGray = NULL;
    _imgInFloat = NULL;
    _imgGReal = NULL;
    _imgGImag = NULL;
    //_imgGAbs = NULL;
    _imgTmp = NULL;
    _imgTmpSur = NULL;
    _imgTmpCen = NULL;
    _imgDSumCen = NULL;
    _imgDSumSur = NULL;
    _imgDSumCenEx = NULL;
    _imgDSumSurEx = NULL;
    _imgCinh = NULL;
    _imgGabor = NULL;
    _imgDCen = NULL;
    _imgDSur = NULL;
    _imgDPlu = NULL;
    _imgDMin = NULL;
    _imgDPluComp = NULL;
    _imgDMinComp = NULL;
    _imgResult = NULL;
    _conspDPlu = NULL;
    _conspDMin = NULL;
    _gabor = NULL;
    _gaussSur = NULL;
    _gaussCen = NULL;
    _gaussCompSur = NULL;
    _gaussCompCen = NULL;
}

AdvancedDirectionalSalience::~AdvancedDirectionalSalience(){
    close();
}

bool AdvancedDirectionalSalience::open(yarp::os::Searchable& config){
    bool ok = Salience::open(config);
    if (!ok)
        return false;

    _scale = config.check("scale",
                          Value(1.0),
                          "Calculate directional salience at given scale of the input image (double [0...1]).").asDouble();
    _sig2wav = config.check("sig2wav",
                          Value(4.0),
                          " gabor_wavelength = sig2wav * gabor_sigma (double).").asDouble();
    _debug = config.check("debug",
                          Value(0),
                          "Activate debugging capabilities (int [0|1]).").asInt()!=0;
    _verbose = config.check("verbose",
                            Value(0),
                            "Activate debugging output (int [0|1]).").asInt()!=0;
    _dbgImgArray = config.check("dbgImgArray",
                          Value("gab"),
                          "Write from this image array to the debugging port (string).").asString(); 
    _dbgDirIndex = config.check("dbgDirIndex",
                          Value(0),
                          "Select this direction from image array to debug (int).").asInt();
    _dbgSizIndex = config.check("dbgSizIndex",
                          Value(0),
                          "Select this filter size from image array to debug (int).").asInt();

    // filters for 4 directions
    _numDirs = 4;
    _dirs = new double[_numDirs];
    _dirs[0] = 0.0;
    _dirs[1] = 45.0;
    _dirs[2] = 90.0;
    _dirs[3] = 135.0;
    // 3 scales for each direction
    _numSizs = 3;
    _sigs = new double[_numSizs];
    _sigs[0] = 2.0;
    _sigs[1] = 4.0;
    _sigs[2] = 8.0;
    /*_sigs[0] = 1.0;
    _sigs[1] = 2.0;
    _sigs[2] = 4.0;*/
    _wavs = new double[_numSizs];
    for (int i = 0; i < _numSizs; i++)
        _wavs[i] = _sigs[i] * _sig2wav;
    // gabor filters
    _gabor = new FastGabor**[_numSizs];
    for (int s = 0; s < _numSizs; s++)
        _gabor[s] = new FastGabor*[_numDirs];
    for (int s = 0; s < _numSizs; s++)
        for (int d = 0; d < _numDirs; d++)
            _gabor[s][d] = new FastGabor();
    // gauss filters
    _gaussCen = new FastGauss*[_numSizs];
    _gaussSur = new FastGauss*[_numSizs];
    _gaussCompCen = new FastGauss*[_numSizs];
    _gaussCompSur = new FastGauss*[_numSizs];
    for (int s = 0; s < _numSizs; s++){
        _gaussCen[s] = new FastGauss();
        _gaussSur[s] = new FastGauss();
        _gaussCompCen[s] = new FastGauss();
        _gaussCompSur[s] = new FastGauss();
    }
    // conspicuity factors
    _conspDPlu = new float*[_numSizs];
    _conspDMin = new float*[_numSizs];
    for (int s = 0; s < _numSizs; s++){
        _conspDPlu[s] = new float[_numDirs];
        _conspDMin[s] = new float[_numDirs];
    }
    if (_debug)
        _prtDebug.open("/debug/salience/directional/o:img");
    yarp::os::Time::turboBoost();
    return true;
}


void AdvancedDirectionalSalience::applyImpl(ImageOf<PixelRgb>& src, 
                          ImageOf<PixelRgb>& dest,
                          ImageOf<PixelFloat>& sal) {
    // time measurement                              
    double tStart = yarp::os::Time::now();
    // block other threads (e.g. configuration)
    _mutex.wait();
    // resizing/init
    sal.resize(src);
    if (_w != src.width() || _h != src.height()){
        _w = src.width();
        _h = src.height();
        _ws = (int)(_w * _scale + 0.5);
        _hs = (int)(_h * _scale + 0.5);
        init(_ws, _hs);
    }
    // scaling if _scale != 1.0
    cvResize((IplImage*)src.getIplImage(), _imgInRgbResized, CV_INTER_LINEAR );
    // convert input to grayscale and float image
    cvCvtColor( _imgInRgbResized, _imgInGray, CV_BGR2GRAY );
    cvConvert(_imgInGray, _imgInFloat);
    // compute D-maps (24 maps: _numSizs*_numDirs * 2 (center & surround))
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
                // compute gabor maps
                _gabor[s][d]->GaborFiltZeroMean((float*)_imgInFloat->imageData, 
                    (float*)_imgGReal->imageData, 
                    (float*)_imgGImag->imageData);
                // combine real and imaginary parts
                _gabor[s][d]->ComputeMagnitude((float*)_imgGReal->imageData, 
                    (float*)_imgGImag->imageData, 
                    (float*)_imgGabor[s][d]->imageData);
                // center- & surround-maps
                _gaussCen[s]->GaussFilt((float*)_imgGabor[s][d]->imageData, (float*)_imgDCen[s][d]->imageData);
                _gaussSur[s]->GaussFilt((float*)_imgGabor[s][d]->imageData, (float*)_imgDSur[s][d]->imageData);
        }
    }
    // compute DPlus & DMinus maps
    float max = 0.0f;
    int maxX, maxY;
    for (int s = 0; s < _numSizs; s++){
        // sum all directions of size s
        cvZero(_imgDSumCen);
        cvZero(_imgDSumSur);
        for (int d = 0; d < _numDirs; d++){
            cvAdd(_imgDSumCen, _imgDCen[s][d], _imgDSumCen);
            cvAdd(_imgDSumSur, _imgDSur[s][d], _imgDSumSur);
        }
        // create DPlus & DMinus maps for size s
        for (int d = 0; d < _numDirs; d++){
            // subtract the current direction from the total sum (Sum(i!=j))
            cvSub(_imgDSumCen, _imgDCen[s][d], _imgDSumCenEx);
            cvSub(_imgDSumSur, _imgDSur[s][d], _imgDSumSurEx);
            // calculate DPlus
            cvSub(_imgDCen[s][d], _imgDSur[s][d], _imgTmp);
            cvAdd(_imgTmp, _imgDSumSurEx, _imgTmp);
            cvMul(_imgDCen[s][d], _imgTmp, _imgDPlu[s][d]);
            // calculate DMinus
            cvSub(_imgDCen[s][d], _imgDSumCenEx, _imgTmp);
            cvAdd(_imgTmp, _imgDSumSurEx, _imgTmp);
            cvMul(_imgDCen[s][d], _imgTmp, _imgDMin[s][d]);
            // discard negative values
            cvThreshold(_imgDPlu[s][d], _imgDPlu[s][d], 0.0, 0.0, CV_THRESH_TOZERO); // this is probably not necessary
            cvThreshold(_imgDMin[s][d], _imgDMin[s][d], 0.0, 0.0, CV_THRESH_TOZERO); 
            // scale to MAX_SALIENCE 
            // DPlus
            //calcGlobalMaximum(_imgDPlu[s][d], max, maxX, maxY);
            //if (max > 0.0f)
            //    cvScale(_imgDPlu[s][d], _imgDPlu[s][d], MAX_SALIENCE/max);
            // DMinus
            //calcGlobalMaximum(_imgDMin[s][d], max, maxX, maxY);
            //if (max > 0.0f)
            //    cvScale(_imgDMin[s][d], _imgDMin[s][d], MAX_SALIENCE/max);
        
            // old:
            // calc maximum, conspicuity factors and scale to MAX_SALIENCE
            // DPlus
            //calcGlobalMaximum(_imgDPlu[s][d], max, maxX, maxY);
            //if (max > 0.0f)
            //    cvScale(_imgDPlu[s][d], _imgDPlu[s][d], MAX_SALIENCE/max);
            //calcAdvancedConspicuityFactor(_imgDPlu[s][d], _sigs[s]*_sig2wav, maxX, maxY, _conspDPlu[s][d]); 
            //calcConspicuityFactor(_imgDPlu[s][d], _conspDPlu[s][d]); 
            // DMinus
            //calcGlobalMaximum(_imgDMin[s][d], max, maxX, maxY);
            //if (max > 0.0f)
            //    cvScale(_imgDMin[s][d], _imgDMin[s][d], MAX_SALIENCE/max);
            // calc conspicuity factors
            //calcAdvancedConspicuityFactor(_imgDMin[s][d], _sigs[s]*_sig2wav, maxX, maxY, _conspDMin[s][d]);
            //calcConspicuityFactor(_imgDMin[s][d], _conspDMin[s][d]);
        }
    }
    // center-surround competition to extract isolated peaks
    // M <- |M+M*DoG-Cinh|>0
    for (int s = 0; s < _numSizs; s++){   
        // gaussian filters for each size
        // cen = sigma
        // sur = 4*sigma
        for (int d = 0; d < _numDirs; d++){
            // copy to working imgDPlu/MinComp
            cvCopy(_imgDPlu[s][d], _imgDPluComp[s][d]);
            cvCopy(_imgDMin[s][d], _imgDMinComp[s][d]);
            for (int k = 0; k < 3; k++){
                // imgDPluComp
                // calculate M*DoG
                _gaussCompCen[s]->GaussFilt((float*)_imgDPluComp[s][d]->imageData, (float*)_imgTmpCen->imageData);
                _gaussCompSur[s]->GaussFilt((float*)_imgDPluComp[s][d]->imageData, (float*)_imgTmpSur->imageData);
                cvSub(_imgTmpCen, _imgTmpSur, _imgTmp); // calculate center surround (mexican hat) convolution
                // add M + M&DoG
                cvAdd(_imgDPluComp[s][d], _imgTmp, _imgDPluComp[s][d]);
                // subtract Cinh
                cvSub(_imgDPluComp[s][d], _imgCinh, _imgDPluComp[s][d]);
                // threshold to >0
                cvThreshold(_imgDPluComp[s][d], _imgDPluComp[s][d], 0.0, 0.0, CV_THRESH_TOZERO);
                // imgDMinComp
                // calculate M*DoG
                _gaussCompCen[s]->GaussFilt((float*)_imgDMinComp[s][d]->imageData, (float*)_imgTmpCen->imageData);
                _gaussCompSur[s]->GaussFilt((float*)_imgDMinComp[s][d]->imageData, (float*)_imgTmpSur->imageData);
                cvSub(_imgTmpCen, _imgTmpSur, _imgTmp); // calculate center surround (mexican hat) convolution
                //// add M + M&DoG
                cvAdd(_imgDMinComp[s][d], _imgTmp, _imgDMinComp[s][d]);
                //// subtract Cinh
                cvSub(_imgDMinComp[s][d], _imgCinh, _imgDMinComp[s][d]);
                //// threshold to >0
                cvThreshold(_imgDMinComp[s][d], _imgDMinComp[s][d], 0.0, 0.0, CV_THRESH_TOZERO);
            }
            // scale final _imgD*Comp's
            calcGlobalMaximum(_imgDPluComp[s][d], max, maxX, maxY);
            if (max > 0.0f)
                cvScale(_imgDPluComp[s][d], _imgDPluComp[s][d], MAX_SALIENCE/max);
            calcGlobalMaximum(_imgDMinComp[s][d], max, maxX, maxY);
            if (max > 0.0f)
                cvScale(_imgDMinComp[s][d], _imgDMinComp[s][d], MAX_SALIENCE/max);
        }
    }

    // summing up all maps (normalized)
    cvZero(_imgResult);
    float mapWeight = 1.0f/((float)(2*_numSizs*_numDirs));
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            cvScaleAdd(_imgDPlu[s][d], cvScalar(mapWeight), _imgResult, _imgResult);
            cvScaleAdd(_imgDMin[s][d], cvScalar(mapWeight), _imgResult, _imgResult);
        }
    }

    // summing up all maps (normalized)
    /*cvZero(_imgResult);
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            cvScaleAdd(_imgDPlu[s][d], cvScalar(_conspDPlu[s][d]), _imgResult, _imgResult);
            cvScaleAdd(_imgDMin[s][d], cvScalar(_conspDMin[s][d]), _imgResult, _imgResult);
        }
    }*/

    // resize/copy back to output image
    cvResize(_imgResult, (IplImage*)sal.getIplImage());
   
    // output debug/analysis of requested map [name, size, direction]
    if (_debug){
        IplImage ***ocvImgArrayDebug = _mapImgTable[_dbgImgArray];
        IplImage *ocvImgDebug = ocvImgArrayDebug[_dbgSizIndex][_dbgDirIndex];
        ImageOf<PixelFloat> &yrpImgDebug = _prtDebug.prepare();
        yrpImgDebug.resize(ocvImgDebug->width, ocvImgDebug->height);
        cvCopy(ocvImgDebug, (IplImage*)yrpImgDebug.getIplImage());
        _prtDebug.write();
    }
    
    // testing output
    //IplImage *test = _imgGImag;
    /*IplImage *test = _imgDCen[0][0];
    max = 0.0f;
    calcGlobalMaximum(test, max);
    cvScale(test, test, 255.0f/max);
    cvResize(test, (IplImage*)sal.getIplImage());*/
    _mutex.post();
    double tEnd = yarp::os::Time::now();
    if (_verbose)
        cout << "Update time: " << tEnd - tStart << " Processed image size: " << _ws << " x " << _hs << " original size: " << _w << " x " << _h << endl;
    // tmp
    //cvConvertScale(_result, (IplImage*)sal.getIplImage(), 1/_scale);
}

void AdvancedDirectionalSalience::init(int w, int h){
    release();
    // images
    _imgInRgbResized = cvCreateImage(cvSize(w,h),IPL_DEPTH_8U, 3);
    _imgInGray = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    _imgInFloat = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
    _imgGReal = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgGImag = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    //_imgGAbs = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgTmp = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgTmpSur = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgTmpCen = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgDSumCen = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgDSumSur = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgDSumCenEx = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgDSumSurEx = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgCinh = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgResult = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
    _imgGabor = new IplImage**[_numSizs];
    _imgDCen = new IplImage**[_numSizs];
    _imgDSur = new IplImage**[_numSizs];
    _imgDPlu = new IplImage**[_numSizs];
    _imgDMin = new IplImage**[_numSizs];
    _imgDPluComp = new IplImage**[_numSizs];
    _imgDMinComp = new IplImage**[_numSizs];
    for (int i = 0; i < _numSizs; i++){
        _imgGabor[i] = new IplImage*[_numDirs];
        _imgDCen[i] = new IplImage*[_numDirs];
        _imgDSur[i] = new IplImage*[_numDirs];
        _imgDPlu[i] = new IplImage*[_numDirs];
        _imgDMin[i] = new IplImage*[_numDirs];
        _imgDPluComp[i] = new IplImage*[_numDirs];
        _imgDMinComp[i] = new IplImage*[_numDirs];
    }
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            _imgGabor[s][d] = cvCreateImage(cvSize(w,h), IPL_DEPTH_32F, 1);
            _imgDCen[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgDSur[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgDPlu[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgDMin[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgDPluComp[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
            _imgDMinComp[s][d] = cvCreateImage(cvSize(w, h),IPL_DEPTH_32F, 1);
        }
    }
    // filters
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            _gabor[s][d]->AllocateResources(h,w,_sigs[s],_dirs[d],_sig2wav * _sigs[s]);
        }
    }
    for (int s = 0; s < _numSizs; s++){
        //_gaussCen[s]->AllocateResources(h,w, _sigs[s]);
        //_gaussSur[s]->AllocateResources(h,w, 4.0*_sigs[s]);
        // DPlus/DMinus center-surround filters
		_gaussCen[s]->AllocateResources(h,w, 2.0*_sigs[s]);
        _gaussSur[s]->AllocateResources(h,w, 8.0*_sigs[s]);
        // competition center-surround filters
        _gaussCompCen[s]->AllocateResources(h,w, _sigs[s]);
        _gaussCompSur[s]->AllocateResources(h,w, 4.0*_sigs[s]);
    }
    // inhibition image for competition calculation
    cvSet(_imgCinh, cvScalar(0.02f*(float)MAX_SALIENCE));
    // image table
    _mapImgTable["gab"] = _imgGabor;
    _mapImgTable["cen"] = _imgDCen;
    _mapImgTable["sur"] = _imgDSur;
    _mapImgTable["pos"] = _imgDPlu;
    _mapImgTable["neg"] = _imgDMin;
    _mapImgTable["poscomp"] = _imgDPluComp;
    _mapImgTable["negcomp"] = _imgDMinComp;
}
 
void AdvancedDirectionalSalience::release(){
    // images
    if (_imgInRgbResized != NULL)
        cvReleaseImage(&_imgInRgbResized);
    _imgInRgbResized = NULL;
    if (_imgInGray != NULL)
        cvReleaseImage(&_imgInGray);
    _imgInGray = NULL;
    if (_imgInFloat != NULL)
        cvReleaseImage(&_imgInFloat);
    _imgInFloat = NULL;
    if (_imgGReal != NULL)
        cvReleaseImage(&_imgGReal);
    _imgGReal = NULL;
    if (_imgGImag != NULL)
        cvReleaseImage(&_imgGImag);
    _imgGImag = NULL;
    if (_imgDSumCen != NULL)
        cvReleaseImage(&_imgDSumCen);
    _imgDSumCen = NULL;
    if (_imgDSumSur != NULL)
        cvReleaseImage(&_imgDSumSur);
    _imgDSumSur = NULL;
    if (_imgDSumCenEx != NULL)
        cvReleaseImage(&_imgDSumCenEx);
    _imgDSumCenEx = NULL;
    if (_imgDSumSurEx != NULL)
        cvReleaseImage(&_imgDSumSurEx);
    _imgDSumSurEx = NULL;
    if (_imgCinh != NULL)
        cvReleaseImage(&_imgCinh);
    _imgCinh = NULL;
    if (_imgTmp != NULL)
        cvReleaseImage(&_imgTmp);
    _imgTmp = NULL;
    if (_imgTmpCen != NULL)
        cvReleaseImage(&_imgTmpCen);
    _imgTmpCen = NULL;
    if (_imgTmpSur != NULL)
        cvReleaseImage(&_imgTmpSur);
    _imgTmpSur = NULL;
    if (_imgResult != NULL)
        cvReleaseImage(&_imgResult);
    _imgResult = NULL;
    if (_imgGabor != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgGabor[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgGabor[s];
        delete [] _imgGabor;
        _imgGabor = NULL;
    }
    if (_imgDCen != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDCen[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDCen[s];
        delete [] _imgDCen;
        _imgDCen = NULL;
    }
    if (_imgDSur != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDSur[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDSur[s];
        delete [] _imgDSur;
        _imgDSur = NULL;
    }
    if (_imgDPlu != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDPlu[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDPlu[s];
        delete [] _imgDPlu;
        _imgDPlu = NULL;
    }
    if (_imgDMin != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDMin[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDMin[s];
        delete [] _imgDMin;
        _imgDMin = NULL;
    }
    if (_imgDPluComp != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDPluComp[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDPluComp[s];
        delete [] _imgDPluComp;
        _imgDPluComp = NULL;
    }
    if (_imgDMinComp != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                cvReleaseImage(&_imgDMinComp[s][d]); 
        for (int s = 0; s < _numSizs; s++)
            delete [] _imgDMinComp[s];
        delete [] _imgDMinComp;
        _imgDMinComp = NULL;
    }
    // filters
    if (_gabor != NULL){
        for (int s = 0; s < _numSizs; s++)
            for (int d = 0; d < _numDirs; d++)
                _gabor[s][d]->FreeResources();
    }
    if (_gaussCen != NULL){
        for (int s = 0; s < _numSizs; s++)
            _gaussCen[s]->FreeResources();
    }
    if (_gaussSur != NULL){
        for (int s = 0; s < _numSizs; s++)
            _gaussSur[s]->FreeResources();
    }
    if (_gaussCompCen != NULL){
        for (int s = 0; s < _numSizs; s++)
            _gaussCompCen[s]->FreeResources();
    }
    if (_gaussCompSur != NULL){
        for (int s = 0; s < _numSizs; s++)
            _gaussCompSur[s]->FreeResources();
    }
}

bool AdvancedDirectionalSalience::close(){
    if (_debug)
        _prtDebug.close();
    release();
    if (_dirs != NULL)
        delete [] _dirs;
    _dirs = NULL;
    if (_sigs != NULL)
        delete [] _sigs;
    _sigs = NULL;
    if (_wavs != NULL)
        delete [] _wavs;
    _wavs = NULL;
    // gabor filters
    for (int s = 0; s < _numSizs; s++)
        for (int d = 0; d < _numDirs; d++)
            delete _gabor[s][d];
    for (int s = 0; s < _numSizs; s++)
        delete [] _gabor[s];
    delete [] _gabor;
    // gauss filters
    for (int s = 0; s < _numSizs; s++){
        delete _gaussCen[s]; 
        delete _gaussSur[s];
        delete _gaussCompCen[s]; 
        delete _gaussCompSur[s];
    }
    delete [] _gaussCen;
    delete [] _gaussSur;
    delete [] _gaussCompCen;
    delete [] _gaussCompSur;
    // conspicuity factors
    for (int s = 0; s < _numSizs; s++){
        delete [] _conspDPlu[s];
        delete [] _conspDMin[s];
    }
    delete [] _conspDPlu;
    delete [] _conspDMin;
    return true;
}

bool AdvancedDirectionalSalience::respond(const Bottle &command,Bottle &reply){
    bool ok = false; // command executed successfully
    bool rec = false; // command recognized

    switch (command.get(0).asVocab()) {
    case SALIENCE_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX:{
                int index = command.get(2).asInt();
                ok = this->setDebugFilterScaleIndex(index);
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX:{
                int index = command.get(2).asInt();
                ok = this->setDebugFilterDirectionIndex(index);
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME:{
                string name(command.get(2).asString().c_str());
                ok = this->setDebugImageArrayName(name);
            }
                break;
            default:
                cout << "AdvancedDirectionalSalience::respond(): received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;
    case SALIENCE_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(SALIENCE_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case SALIENCE_VOCAB_DIRECTIONAL_NUM_DIRECTIONS:{
                int num = this->getNumberDirections();
                reply.addInt(num);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_NUM_SCALES:{
                int num = this->getNumberScales();
                reply.addInt(num);
                ok = true;
            }
                break;
            case SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES:{
                Bottle &bot = reply.addList();
                bot.fromString(this->getDebugImageArrayNames().toString());
                ok = true;
            }
                break;
		    default:
                cout << "AdvancedDirectionalSalience::respond(): received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    return ok;
}

void AdvancedDirectionalSalience::calcGlobalMaximum(const IplImage *img, float &max, int &maxX, int &maxY) {
    float* data = (float*)img->imageData;
    int posRow, pos;
    max = 0.0f;
    for (int y = 0; y < img->height; y++){
        posRow = img->width * y;
        for (int x = 0; x < img->width; x++){
            pos = posRow + x;
            if (data[pos] > max){
                max = data[pos];
                maxX = x;
                maxY = y;
            }
        }
    }
}

void AdvancedDirectionalSalience::calcGlobalMinimum(const IplImage *img, float &min) {
    float* data = (float*)img->imageData;
    int posRow, pos;
    min = 99999.9f; // todo better use -> float.h
    for (int y = 0; y < img->height; y++){
        posRow = img->width * y;
        for (int x = 0; x < img->width; x++){
            pos = posRow + x;
            if (data[pos] < min){
                min = data[pos];
            }
        }
    }
}

void AdvancedDirectionalSalience::calcAdvancedConspicuityFactor(const IplImage *img, const double &sigmaSurround, const int &globMaxX, const int &globMaxY, float &factor){

    double sigma = 4*sigmaSurround;
    // calculate pow((M-mAvg),2) according to Itti & Koch (Rapid Scene Analysis)
    float *data = (float*)img->imageData;
    float sum = 0.0f;
    float dist;
    int posRow, pos, posPlus, posMinus, count = 0;
    for (int y = 1; y < (img->height-1); y++){
        posRow = img->width * y;
        for (int x = 1; x < (img->width-1); x++){
            pos = posRow + x;
            posMinus = pos - img->width; // row before
            posPlus = pos + img->width; // next row
            // is (x,y) a local maxima?
            // Flat regions are discarded (peak needed)
            if ( data[pos] > data[pos-1] &&
                 data[pos] > data[pos+1] &&
                 data[pos] > data[posMinus] &&
                 data[pos] > data[posPlus] &&
                 data[pos] > data[posMinus-1] &&
                 data[pos] > data[posMinus+1] &&
                 data[pos] > data[posPlus-1] &&
                 data[pos] > data[posPlus+1] &&
                 data[pos] > 0.1f){
                dist = pow((float)(globMaxX-x),2) + pow((float)(globMaxY-y),2);
                if (dist < (float)pow(sigma*3.5,2)){
                    sum += (float)data[pos] * (float)exp(-0.5 * dist / pow(sigma,2));
                    count++;
				}
                else{
                    //sum += 0.0f;
                }
            }
        }
    }
    //cout << "Global Maximum 2: " << max << endl;
    //cout << endl;
    //cout << "Factor: num local max's: " << count << endl;
    if (count == 1){
        factor = 1.0f;
        //cout << "1 global maximum!" << endl;
    }
    else if (count > 1){
        sum -= MAX_SALIENCE; // subtract global maximum
        count--; 
        factor = pow( (MAX_SALIENCE - (sum/(float)count))/MAX_SALIENCE ,2);
        //cout <<  " average of non global max's: " << factor <<  endl;
    }
    else
        factor = 0.0f;
}

void AdvancedDirectionalSalience::calcConspicuityFactor(const IplImage *img, float &factor){

    // calculate pow((M-mAvg),2) according to Itti & Koch (Rapid Scene Analysis)
    float *data = (float*)img->imageData;
    //float max = 0.0f;
    float sum = 0.0f;
    int posRow, pos, posPlus, posMinus, count = 0;
    for (int y = 1; y < (img->height-1); y++){
        posRow = img->width * y;
        for (int x = 1; x < (img->width-1); x++){
            pos = posRow + x;
            posMinus = pos - img->width; // row before
            posPlus = pos + img->width; // next row
            // is (x,y) a local maxima?
            // Flat regions are discarded (peak needed)
            if ( data[pos] > data[pos-1] &&
                 data[pos] > data[pos+1] &&
                 data[pos] > data[posMinus] &&
                 data[pos] > data[posPlus] &&
                 data[pos] > data[posMinus-1] &&
                 data[pos] > data[posMinus+1] &&
                 data[pos] > data[posPlus-1] &&
                 data[pos] > data[posPlus+1]){
                sum += data[pos];
                count++;
            }
        }
    }
    //cout << "Global Maximum 2: " << max << endl;
    //cout << endl;
    //cout << "Factor: num local max's: " << count << endl;
    if (count == 1){
        factor = 1.0f;
        //cout << "1 global maximum!" << endl;
    }
    else if (count > 1){
        sum -= MAX_SALIENCE; // subtract global maximum
        count--; 
        factor = pow( (MAX_SALIENCE - (sum/(float)count))/MAX_SALIENCE ,2);
        //cout <<  " average of non global max's: " << factor <<  endl;
    }
    else
        factor = 0.0f;
}

int AdvancedDirectionalSalience::getNumberScales(){
    return _numSizs;
}

int AdvancedDirectionalSalience::getNumberDirections(){
    return _numDirs;
}

bool AdvancedDirectionalSalience::setDebugFilterScaleIndex(int size){
    bool ok = true;
    _mutex.wait(); 
    if (size < _numSizs){
        _dbgSizIndex = size; 
    }
    else{
        ok = false;
    }
    _mutex.post(); 
    return ok;
}

bool AdvancedDirectionalSalience::setDebugFilterDirectionIndex(int direction){
    bool ok = true;
    _mutex.wait(); 
    if (direction < _numDirs){
        _dbgDirIndex = direction; 
    }
    else{
        ok = false;
    }
    _mutex.post(); 
    return ok;
}

Bottle AdvancedDirectionalSalience::getDebugImageArrayNames(){
    Bottle bottle;
    for(std::map<string, IplImage***>::const_iterator i = _mapImgTable.begin(); i != _mapImgTable.end(); i++){
        bottle.add(Value(i->first.c_str()));
    }
    return bottle;
}

bool AdvancedDirectionalSalience::setDebugImageArrayName(string imgArray){
    bool ok = true;
    _mutex.wait();
    //if (_mapImgTable.find(imgArray) != NULL){
        _dbgImgArray = imgArray;
    //}
    //else{
    //    ok = false;
    //}
    _mutex.post();
    return ok;
}

/*
// normalize conspicuity factors
    if (_verbose)
        cout << endl << "Conspicuity Factors: " << endl;
    float sum = 0.0f;
    for (int s = 0; s < _numSizs; s++){
        for (int d = 0; d < _numDirs; d++){
            sum += _conspDPlu[s][d];
            sum += _conspDMin[s][d];
        }
    }
    if (sum > 0.0f){
        for (int s = 0; s < _numSizs; s++){
            for (int d = 0; d < _numDirs; d++){
                _conspDPlu[s][d] /= sum;
                _conspDMin[s][d] /= sum;
                if (_verbose)
                    cout << "plus: " << _conspDPlu[s][d] << " minus: " << _conspDMin[s][d] << " sigma: " << _sigs[s] << " direction: " << _dirs[d] << endl;
            }
        }
    }
    */
