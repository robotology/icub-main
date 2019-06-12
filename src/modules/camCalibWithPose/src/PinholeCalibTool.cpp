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
    _mapUndistortX = NULL;
    _mapUndistortY = NULL;
    _intrinsic_matrix = cvCreateMat(3,3, CV_32F);
    _intrinsic_matrix_scaled = cvCreateMat(3,3, CV_32F);
    _distortion_coeffs = cvCreateMat(1, 4, CV_32F);
    _oldImgSize.width = -1;
    _oldImgSize.height = -1;
    _needInit = true;
}

PinholeCalibTool::~PinholeCalibTool(){

}

bool PinholeCalibTool::close(){
    if (_mapUndistortX != NULL)
        cvReleaseImage(&_mapUndistortX);
    _mapUndistortX = NULL;
    if (_mapUndistortY != NULL)
        cvReleaseImage(&_mapUndistortY);
    _mapUndistortY = NULL;
    cvReleaseMat(&_intrinsic_matrix);
    cvReleaseMat(&_intrinsic_matrix_scaled);
    cvReleaseMat(&_distortion_coeffs);
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
                                      "Image width for which calibration parameters were calculated (int)").asInt();

    _calibImgSize.height = config.check("h",
                                      Value(240),
                                      "Image height for which calibration parameters were calculated (int)").asInt();

    _drawCenterCross = config.check("drawCenterCross",
                                    Value(0),
                                    "Draw a cross at calibration center (int [0|1]).").asInt()!=0;


    CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 0) = (float)config.check("fx",
                                                        Value(320.0),
                                                        "Focal length x (double)").asDouble();

    CV_MAT_ELEM( *_intrinsic_matrix, float, 0, 1) = 0.0f;
    CV_MAT_ELEM( *_intrinsic_matrix, float, 0, 2) = (float)config.check("cx",
                                                        Value(160.0),
                                                        "Principal point x (double)").asDouble();
    CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 0) = 0.0f;
    CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 1) = (float)config.check("fy",
                                                        Value(320.0),
                                                        "Focal length y (double)").asDouble();
    CV_MAT_ELEM( *_intrinsic_matrix, float, 1, 2) = (float)config.check("cy",
                                                        Value(120.0),
                                                        "Principal point y (double)").asDouble();
    CV_MAT_ELEM( *_intrinsic_matrix, float, 2, 0) = 0.0f;
    CV_MAT_ELEM( *_intrinsic_matrix, float, 2, 1) = 0.0f;
    CV_MAT_ELEM( *_intrinsic_matrix, float, 2, 2) = 1.0f;


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


    fprintf(stdout,"fx=%g\n",config.find("fx").asDouble());
    fprintf(stdout,"fy=%g\n",config.find("fy").asDouble());
    fprintf(stdout,"cx=%g\n",config.find("cx").asDouble());
    fprintf(stdout,"cy=%g\n",config.find("cy").asDouble());


    // copy to scaled matrix ;)
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 0);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 1);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 2);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 0);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 1);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 2);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 0);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 1);
    CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 2);

     /* init the distortion coeffs */
    CV_MAT_ELEM( *_distortion_coeffs, float, 0, 0) = (float)config.check("k1",
                                                        Value(0.0),
                                                        "Radial distortion 1(double)").asDouble();
    CV_MAT_ELEM( *_distortion_coeffs, float, 0, 1) = (float)config.check("k2",
                                                        Value(0.0),
                                                        "Radial distortion 2(double)").asDouble();
    CV_MAT_ELEM( *_distortion_coeffs, float, 0, 2) = (float)config.check("p1",
                                                        Value(0.0),
                                                        "Tangential distortion 1(double)").asDouble();
    CV_MAT_ELEM( *_distortion_coeffs, float, 0, 3) = (float)config.check("p2",
                                                        Value(0.0),
                                                        "Tangential distortion 2(double)").asDouble();
    _needInit = true;

    return true;
}


bool PinholeCalibTool::init(CvSize currImgSize, CvSize calibImgSize){

    if (_mapUndistortX != NULL)
        cvReleaseImage(&_mapUndistortX);
    _mapUndistortX = NULL;
    if (_mapUndistortY != NULL)
        cvReleaseImage(&_mapUndistortY);
    _mapUndistortY = NULL;


    // Scale the intrinsics if required:
    // if current image size is not the same as the size for
    // which calibration parameters are specified we need to
    // scale the intrinsic matrix components.
    if (currImgSize.width != calibImgSize.width ||
        currImgSize.height != calibImgSize.height){
        float scaleX = (float)currImgSize.width / (float)calibImgSize.width;
        float scaleY = (float)currImgSize.height / (float)calibImgSize.height;
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 0) * scaleX;
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 1);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 2) * scaleX;
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 0);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 1) * scaleY;
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 2) * scaleY;
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 0);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 1);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 2);
    }
    else{
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 0);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 1);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 0, 2);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 0);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 1);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 1, 2);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 0) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 0);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 1) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 1);
        CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 2, 2) = CV_MAT_ELEM( *_intrinsic_matrix , float, 2, 2);
    }

    _mapUndistortX = cvCreateImage(currImgSize, IPL_DEPTH_32F, 1);
    _mapUndistortY = cvCreateImage(currImgSize, IPL_DEPTH_32F, 1);

    /* init the undistortion matrices */
    cvInitUndistortMap( _intrinsic_matrix_scaled, _distortion_coeffs,
                        _mapUndistortX, _mapUndistortY);

    _needInit = false;
    return true;
}

void PinholeCalibTool::apply(const ImageOf<PixelRgb> & in, ImageOf<PixelRgb> & out){

    CvSize inSize = cvSize(in.width(),in.height());

    // check if reallocation required
    if ( inSize.width  != _oldImgSize.width ||
         inSize.height != _oldImgSize.height ||
        _needInit)
        init(inSize,_calibImgSize);

    cv::Mat outMat;
    cv::remap( toCvMat(const_cast<ImageOf<PixelRgb>&>(in)), outMat,
               cv::cvarrToMat(_mapUndistortX), cv::cvarrToMat(_mapUndistortY),
               cv::INTER_LINEAR );
    out=fromCvMat<PixelRgb>(outMat);

    // painting crosshair at calibration center
    if (_drawCenterCross){
        yarp::sig::PixelRgb pix = yarp::sig::PixelRgb(255,255,255);
        yarp::sig::draw::addCrossHair(out, pix, (int)CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 0, 2),
                                            (int)CV_MAT_ELEM( *_intrinsic_matrix_scaled , float, 1, 2),
                                            10);
    }

    // buffering old image size
    _oldImgSize.width  = inSize.width;
    _oldImgSize.height = inSize.height;
}

