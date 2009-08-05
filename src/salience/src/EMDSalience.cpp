// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/EMDSalience.h>

using namespace yarp::sig;
using namespace iCub::contrib;

EMDSalience::EMDSalience(){
    
    _ocvBgrImgInput = NULL;
    _ocvGryImgNew = NULL;	
	_ocvGryImgOld = NULL;		
    _ocvFltImgX = NULL;		
	_ocvFltImgY = NULL;		
	_ocvFltImgAbs = NULL;	   

    _sizeOld.width = -1;
    _sizeOld.height = -1;
}

EMDSalience::~EMDSalience(){
    close();
}

bool EMDSalience::open(yarp::os::Searchable& config){
    
    // group filter already checks for a 
    // subconfiguration group of name [filter_name]
    return configure(config);
}

bool EMDSalience::close(){
    bool ok = Salience::close();
    _emd.close();
    releaseImages();
    return ok;
}

bool EMDSalience::configure(yarp::os::Searchable& config){

    bool ok = Salience::open(config);
    if (!ok)
        return false;

    _blurInput = (bool)config.check("blurInput",
                                    yarp::os::Value(1),
                                    "Gaussian blur (3x3) of input image (int [0|1]).").asInt();
    _blurOutput = (bool)config.check("blurOutput",
                                    yarp::os::Value(1),
                                    "Gaussian blur (3x3) of motion output (int [0|1]).").asInt();

    return _emd.configure(config);
}

void EMDSalience::initImages(CvSize size){
    releaseImages();

    _ocvBgrImgInput = cvCreateImage(size, IPL_DEPTH_8U, 3);
    _ocvGryImgNew = cvCreateImage(size, IPL_DEPTH_8U,1);
    _ocvGryImgOld = cvCreateImage(size, IPL_DEPTH_8U,1);	
	
	_ocvFltImgX = cvCreateImage(size, IPL_DEPTH_32F,1);	
	_ocvFltImgY = cvCreateImage(size, IPL_DEPTH_32F,1);	
	_ocvFltImgAbs = cvCreateImage(size, IPL_DEPTH_32F,1);
}

void EMDSalience::releaseImages(){
    if(_ocvBgrImgInput != NULL)
        cvReleaseImage(&_ocvBgrImgInput);
    _ocvBgrImgInput = NULL;

    if(_ocvGryImgNew != NULL)
        cvReleaseImage(&_ocvGryImgNew);
    _ocvGryImgNew = NULL;
    if(_ocvGryImgOld != NULL)
        cvReleaseImage(&_ocvGryImgOld);
    _ocvGryImgOld = NULL;

    if(_ocvFltImgX != NULL)
        cvReleaseImage(&_ocvFltImgX);
    _ocvFltImgX = NULL;
    if(_ocvFltImgY != NULL)
        cvReleaseImage(&_ocvFltImgY);
    _ocvFltImgY = NULL;
    if(_ocvFltImgAbs != NULL)
        cvReleaseImage(&_ocvFltImgAbs);
    _ocvFltImgAbs = NULL;   
}

void EMDSalience::applyImpl(ImageOf<PixelRgb>& src, 
                           ImageOf<PixelRgb>& dest,
                           ImageOf<PixelFloat>& sal) {
    dest.resize(src);
    sal.resize(src);

    if (_sizeOld.width != src.width() || 
        _sizeOld.height != src.height())
        initImages(cvSize(src.width(), src.height()));

    // convert input image from rgb to bgr (opencv default)
    cvCvtColor((IplImage*)src.getIplImage(), _ocvBgrImgInput, CV_RGB2BGR);

    // smooth the resized image
	if (_blurInput)
		cvSmooth( _ocvBgrImgInput, _ocvBgrImgInput); // CV_GAUSSIAN 3x3

    // convert image to grayscale for motion calculation
	cvCvtColor(_ocvBgrImgInput, _ocvGryImgNew, CV_BGR2GRAY);

    // calculate the optical flow
    //_emd.calculate_flow(_ocvGryImgNew, _ocvGryImgOld, _ocvFltImgX, _ocvFltImgY, _ocvFltImgAbs);
    _emd.calculate_flow(_ocvGryImgNew, _ocvGryImgOld, _ocvFltImgX, _ocvFltImgY, (IplImage*)sal.getIplImage());

    // blur motion images
	if (_blurOutput)
		cvSmooth( (IplImage*)sal.getIplImage(), (IplImage*)sal.getIplImage()); // CV_GAUSSIAN 3x3

    //colorRgbFromFloat((IplImage*)sal.getIplImage(), (IplImage*)dest.getIplImage(), 1.0f);

    // buffering _ocvGryImgNew for motion calculation
    cvCopy(_ocvGryImgNew, _ocvGryImgOld);

    // buffering old image size
    _sizeOld.width = src.width();
    _sizeOld.height = src.height();
}


void EMDSalience::colorRgbFromFloat(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor){
    
    float flValue = 0.0f;
    int   arraypos = 0;
 
    for (int y = 0; y < imgFloat->height; y++){
    	arraypos = imgRgb->widthStep*y;
        for (int x = 0; x < imgFloat->width; x++){
            flValue = scaleFactor * ((float*)(imgFloat->imageData + imgFloat->widthStep*y))[x];
            if (flValue < 0.0f){
                flValue = -flValue;
                if (flValue > 255.0f)
                    flValue = 255.0f;
                //cout << "<0.0: " << flValue << endl;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+0] = (uchar)flValue;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+1] = (uchar)0;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+2] = (uchar)0;
            }
            else if (flValue >= 0.0f){
                if (flValue > 255.0f)
                    flValue = 255.0f;
                //cout << ">0.0: " << flValue << endl;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+1] = (uchar)flValue;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+2] = (uchar)0;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+0] = (uchar)0;
            }
        }
    }                
}
