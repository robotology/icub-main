// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#include <iCub/SphericalCalibTool.h>
#include <stdio.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

SphericalCalibTool::SphericalCalibTool(){
    _mapX = NULL;
    _mapY = NULL;
    _oldImgSize.width = -1;
    _oldImgSize.height = -1;
    _needInit = true;
}

SphericalCalibTool::~SphericalCalibTool(){
   
}

bool SphericalCalibTool::open(Searchable &config){
    return configure(config);
}

bool SphericalCalibTool::close(){
    if (_mapX != NULL)
        cvReleaseImage(&_mapX);
    _mapX = NULL;
    if (_mapY != NULL)
        cvReleaseImage(&_mapY);
    _mapY = NULL;
    return true;
}

void SphericalCalibTool::stopConfig( std::string val ){

    fprintf(stdout,"There seem to be an error loading parameters \"%s\", stopping module\n", val.c_str());
}

bool SphericalCalibTool::configure (Searchable &config){
    
    // Defaults will correspond to a view field of 90 deg.
    _calibImgSize.width = config.check("w",
                                      Value(320),
                                      "Image width for which calibration parameters were calculated (int)").asInt();
    _calibImgSize.height = config.check("h",
                                      Value(240),
                                      "Image height for which calibration parameters were calculated (int)").asInt();
    _drawCenterCross = config.check("drawCenterCross",
                                    Value(0),
                                    "Draw a cross at calibration center (int [0|1]).").asInt()!=0;

    _fx = config.check("fx", Value(320.0), "Focal distance (on horizontal pixel size units) (double)").asDouble();
    _fy = config.check("fy", Value(240.0), "Focal distance (on vertical pixel size units) (double)").asDouble();
    _cx = config.check("cx", Value(320.0), "Image center (on horizontal pixel size units) (double)").asDouble();
    _cy = config.check("cy", Value(240.0), "Image center (on vertical pixel size units) (double)").asDouble();
    _k1 = config.check("k1", Value(0.0), "Radial distortion (first parameter) (double)").asDouble();
    _k2 = config.check("k2", Value(0.0), "Radial distortion (second parameter) (double)").asDouble();
    _p1 = config.check("p1", Value(0.0), "Tangential distortion (first parameter) (double)").asDouble();
    _p2 = config.check("p2", Value(0.0), "Tangential distortion (second parameter) (double)").asDouble();

    //check to see if the value is read correctly without caring about the default values.
    if ( !config.check("drawCenterCross") ) { stopConfig("drawCenterCross"); return false; }
    if ( !config.check("w") ) { stopConfig("w"); return false;}
    if ( !config.check("h") ) { stopConfig("h"); return false;}
    if ( !config.check("fx") ) { stopConfig("fx"); return false;}
    if ( !config.check("fy") ) { stopConfig("fy"); return false;}
    if ( !config.check("cx") ) { stopConfig("cx"); return false;}
    if ( !config.check("cy") ) { stopConfig("cy"); return false;}
    if ( !config.check("k1") ) { stopConfig("k1"); return false;}
    if ( !config.check("k2") ) { stopConfig("k2"); return false;}
    if ( !config.check("p1") ) { stopConfig("p1"); return false;}
    if ( !config.check("p2") ) { stopConfig("p2"); return false;}

    _fx_scaled = _fx;
    _fy_scaled = _fy;
    _cx_scaled = _cx;
    _cy_scaled = _cy;

    _needInit = true;

    return true;
}


bool SphericalCalibTool::init(CvSize currImgSize, CvSize calibImgSize){

     if (_mapX != NULL)
        cvReleaseImage(&_mapX);
    _mapX = NULL;
    if (_mapY != NULL)
        cvReleaseImage(&_mapY);
    _mapY = NULL;

    // Scale the intrinsics if required:
    // if current image size is not the same as the size for
    // which calibration parameters are specified we need to
    // scale the intrinsic matrix components.
    if (currImgSize.width != calibImgSize.width ||
        currImgSize.height != calibImgSize.height){
        float scaleX = (float)currImgSize.width / (float)calibImgSize.width;
        float scaleY = (float)currImgSize.height / (float)calibImgSize.height;
        _fx_scaled = _fx * scaleX;
        _fy_scaled = _fy * scaleY;
        _cx_scaled = _cx * scaleX;
        _cy_scaled = _cy * scaleY;
    }
    else{
        _fx_scaled = _fx;
        _fy_scaled = _fy;
        _cx_scaled = _cx;
        _cy_scaled = _cy;
    }

    _mapX = cvCreateImage(currImgSize, IPL_DEPTH_32F, 1);
    _mapY = cvCreateImage(currImgSize, IPL_DEPTH_32F, 1);

    

    if(!compute_sp_map(currImgSize.height, currImgSize.width, 
                       currImgSize.height, currImgSize.width,
                        _fx_scaled, _fy_scaled, _cx_scaled, _cy_scaled, 
                        _k1, _k2, _p1, _p2, 
                        (float*)_mapX->imageData, (float*)_mapY->imageData))
        return false;

    _needInit = false;
    return true;
}

void SphericalCalibTool::apply(const ImageOf<PixelRgb> & in, ImageOf<PixelRgb> & out){

    CvSize inSize = cvSize(in.width(),in.height());

    // check if reallocation required
    if ( inSize.width  != _oldImgSize.width || 
         inSize.height != _oldImgSize.height || 
        _needInit)
        init(inSize,_calibImgSize);

    out.resize(inSize.width, inSize.height);

    cvRemap( in.getIplImage(), out.getIplImage(),
           _mapX, _mapY,
           CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

    // painting crosshair at calibration center
    if (_drawCenterCross){
	    yarp::sig::PixelRgb pix = yarp::sig::PixelRgb(255,255,255);
		yarp::sig::draw::addCrossHair(out, pix, (int)_cx_scaled, (int)_cy_scaled, 10);
    }

    // buffering old image size
    _oldImgSize.width  = inSize.width;
    _oldImgSize.height = inSize.height;
}
