/*
 * Copyright (C) 2007 Lijin Aryananda, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <utility>
#include <yarp/cv/Cv.h>
#include <iCub/PinholeCalibTool.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::cv;

PinholeCalibTool::PinholeCalibTool(){
    _intrinsic_matrix = cv::Mat::eye(3, 3, CV_32F);
    _intrinsic_matrix_scaled = cv::Mat::eye(3, 3, CV_32F);
    _distortion_coeffs = cv::Mat::zeros(1, 4, CV_32F);
    _oldImgSize = cv::Size(-1, -1);
    _needInit = true;
}

PinholeCalibTool::~PinholeCalibTool(){
    
}

bool PinholeCalibTool::close(){
    _mapUndistortX.release();
    _mapUndistortY.release();
    _intrinsic_matrix.release();
    _intrinsic_matrix_scaled.release();
    _distortion_coeffs.release();
    return true;
}

bool PinholeCalibTool::open(Searchable &config){
    return configure(config);
}

void PinholeCalibTool::stopConfig( string val ){

    fprintf(stdout,"There seem to be an error loading parameters \"%s\", stopping module\n", val.c_str());
}

bool PinholeCalibTool::configure (Searchable &config){

    _calibImgSize.width = config.check("w",
                                      Value(320),
                                      "Image width for which calibration parameters were calculated (int)").asInt32();

    _calibImgSize.height = config.check("h",
                                      Value(240),
                                      "Image height for which calibration parameters were calculated (int)").asInt32();

    _drawCenterCross = config.check("drawCenterCross",
                                    Value(0),
                                    "Draw a cross at calibration center (int [0|1]).").asInt32()!=0;


    _intrinsic_matrix.at<float>(0, 0) = (float)config.check("fx",
                                                        Value(320.0),
                                                        "Focal length x (double)").asFloat64();

    _intrinsic_matrix.at<float>(0, 1) = 0.0f;
    _intrinsic_matrix.at<float>(0, 2) = (float)config.check("cx",
                                                        Value(160.0),
                                                        "Principal point x (double)").asFloat64();
    _intrinsic_matrix.at<float>(1, 0) = 0.0f;
    _intrinsic_matrix.at<float>(1, 1) = (float)config.check("fy",
                                                        Value(320.0),
                                                        "Focal length y (double)").asFloat64();
    _intrinsic_matrix.at<float>(1, 2) = (float)config.check("cy",
                                                        Value(120.0),
                                                        "Principal point y (double)").asFloat64();
    _intrinsic_matrix.at<float>(2, 0) = 0.0f;
    _intrinsic_matrix.at<float>(2, 1) = 0.0f;
    _intrinsic_matrix.at<float>(2, 2) = 1.0f;


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


    fprintf(stdout,"fx=%g\n",config.find("fx").asFloat64());
    fprintf(stdout,"fy=%g\n",config.find("fy").asFloat64());
    fprintf(stdout,"cx=%g\n",config.find("cx").asFloat64());
    fprintf(stdout,"cy=%g\n",config.find("cy").asFloat64());


    _intrinsic_matrix.copyTo(_intrinsic_matrix_scaled);

     /* init the distortion coeffs */
    _distortion_coeffs.at<float>(0, 0) = (float)config.check("k1",
                                                        Value(0.0),
                                                        "Radial distortion 1(double)").asFloat64();
    _distortion_coeffs.at<float>(0, 1) = (float)config.check("k2",
                                                        Value(0.0),
                                                        "Radial distortion 2(double)").asFloat64();
    _distortion_coeffs.at<float>(0, 2) = (float)config.check("p1",
                                                        Value(0.0),
                                                        "Tangential distortion 1(double)").asFloat64();
    _distortion_coeffs.at<float>(0, 3) = (float)config.check("p2",
                                                        Value(0.0),
                                                        "Tangential distortion 2(double)").asFloat64();
    _needInit = true;

    return true;
}


bool PinholeCalibTool::init(cv::Size currImgSize, cv::Size calibImgSize){
    // Scale the intrinsics if required:
    // if current image size is not the same as the size for
    // which calibration parameters are specified we need to
    // scale the intrinsic matrix components.
    if (currImgSize.width != calibImgSize.width ||
        currImgSize.height != calibImgSize.height){
        float scaleX = (float)currImgSize.width / (float)calibImgSize.width;
        float scaleY = (float)currImgSize.height / (float)calibImgSize.height;
        _intrinsic_matrix.copyTo(_intrinsic_matrix_scaled);
        _intrinsic_matrix_scaled.at<float>(0, 0) *= scaleX;
        _intrinsic_matrix_scaled.at<float>(0, 2) *= scaleX;
        _intrinsic_matrix_scaled.at<float>(1, 1) *= scaleY;
        _intrinsic_matrix_scaled.at<float>(1, 2) *= scaleY;
    }
    else{
        _intrinsic_matrix.copyTo(_intrinsic_matrix_scaled);
    }
    
    /* init the undistortion matrices */
    cv::initUndistortRectifyMap(_intrinsic_matrix_scaled, _distortion_coeffs, cv::Mat(),
                                _intrinsic_matrix_scaled, currImgSize,
                                CV_32FC1, _mapUndistortX, _mapUndistortY);

    _needInit = false;
    return true;
}

void PinholeCalibTool::apply(const ImageOf<PixelRgb> & in, ImageOf<PixelRgb> & out){

    cv::Size inSize(in.width(), in.height());

    // check if reallocation required
    if ( inSize.width  != _oldImgSize.width || 
         inSize.height != _oldImgSize.height || 
        _needInit)
        init(inSize,_calibImgSize);

    cv::Mat outMat;
    cv::remap( toCvMat(const_cast<ImageOf<PixelRgb>&>(in)), outMat,
               _mapUndistortX, _mapUndistortY,
               cv::INTER_LINEAR );
    out=fromCvMat<PixelRgb>(outMat);

    // painting crosshair at calibration center
    if (_drawCenterCross){
        yarp::sig::PixelRgb pix = yarp::sig::PixelRgb(255,255,255);
        yarp::sig::draw::addCrossHair(out, pix, (int)_intrinsic_matrix_scaled.at<float>(0, 2),
                                            (int)_intrinsic_matrix_scaled.at<float>(1, 2),
                                            10);
    }

    // buffering old image size
    _oldImgSize.width  = inSize.width;
    _oldImgSize.height = inSize.height;
}
