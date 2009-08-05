// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#include <iCub/SphereProjector.h>

using namespace yarp::sig;

SphereProjector::SphereProjector(){
    
    _oldInSize.width = -1;
    _oldInSize.height = -1;
    _oldSphereSize.width = -1;
    _oldSphereSize.height = -1;
    _azSpan = -1.0;
    _elSpan = -1.0;
    _mapx = NULL;
    _mapy = NULL;
    _needInit = true;
}

SphereProjector::~SphereProjector(){
   
}

bool SphereProjector::open(Searchable &config){
    return configure(config);
}

bool SphereProjector::close(){
   
    if (_mapx != NULL)
        cvReleaseImage(&_mapx);
    _mapx = NULL;
    if (_mapy != NULL)
        cvReleaseImage(&_mapy);
    _mapy = NULL;
    return true;
}

bool SphereProjector::configure (Searchable &config){
    
    // Defaults will correspond to a view field of 90 deg.
    _calibImgSize.width = config.check("w",
                                      Value(320),
                                      "Image width for which calibration parameters were calculated (int)").asInt();
    _calibImgSize.height = config.check("h",
                                      Value(240),
                                      "Image height for which calibration parameters were calculated (int)").asInt();
    _fx = config.check("fx", Value(320.0), "Focal distance (on horizontal pixel size units) (double)").asDouble();
    _fy = config.check("fy", Value(240.0), "Focal distance (on vertical pixel size units) (double)").asDouble();
    _cx = config.check("cx", Value(320.0), "Image center (on horizontal pixel size units) (double)").asDouble();
    _cy = config.check("cy", Value(240.0), "Image center (on vertical pixel size units) (double)").asDouble();
    _k1 = config.check("k1", Value(0.0), "Radial distortion (first parameter) (double)").asDouble();
    _k2 = config.check("k2", Value(0.0), "Radial distortion (second parameter) (double)").asDouble();
    _p1 = config.check("p1", Value(0.0), "Tangential distortion (first parameter) (double)").asDouble();
    _p2 = config.check("p2", Value(0.0), "Tangential distortion (second parameter) (double)").asDouble();
    
    _fx_scaled = _fx;
    _fy_scaled = _fy;
    _cx_scaled = _cx;
    _cy_scaled = _cy;

    //_map = cvCreateMat(2,3, CV_32F);

    _needInit = true;

    return true;
}


bool SphereProjector::init(CvSize currImgSize, CvSize calibImgSize){

    cout << "SphereProjector::init called..." << endl;
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

    compute_sp_params( currImgSize.height, currImgSize.width, 
                       currImgSize.height, currImgSize.width,
                       _fx_scaled, _fy_scaled, _cx_scaled, _cy_scaled, 
                       //_fx, _fy, _cx, _cy,
                       _k1, _k2,
                       _p2, _p2,
                       &_fa, &_fe,
                       &_ca, &_ce);

    _azSpan = 2.0 * atan(_ca/_fa);
    _elSpan = 2.0 * atan(_ce/_fe);
    cout << "fa: " << _fa << " ca: " << _ca << endl;
    cout << "fe: " << _fe << " ce: " << _ce << endl;
    cout << "_azSpan: " << _azSpan * 180.0 / M_PI << " _elSpan: " << _elSpan * 180.0 / M_PI << endl;

    //cout << "fx: " << _fx_scaled << " cx: " << _cx_scaled << endl;
    //_pixPerDegXIn = (currImgSize.width / 2.0) / (atan(_cx_scaled / _fx_scaled) * 180.0 / 3.14159265);
    //_pixPerDegYIn = (currImgSize.height / 2.0) / (atan(_cy_scaled / _fy_scaled) *180.0 / 3.14159265);

    _needInit = false;
    return true;
}

bool SphereProjector::initSphere(CvSize sphereSize){

    cout << "SphereProjector::initSphere called..." << endl;

    if (_mapx != NULL)
        cvReleaseImage(&_mapx);
    _mapx = NULL;
    if (_mapy != NULL)
        cvReleaseImage(&_mapy);
    _mapy = NULL;
    _mapx = cvCreateImage(sphereSize, IPL_DEPTH_32F, 1);
    _mapy = cvCreateImage(sphereSize, IPL_DEPTH_32F, 1);

    //_pixPerDegXSphere = sphereSize.width / 360.0;
    //_pixPerDegYSphere = sphereSize.height / 360.0;
   
    // build distortion map
    //_yrpImgBase.resize(sphereSize.width, sphereSize.height);

    return true;
}

void SphereProjector::project(const Image &in, 
                             Image &sphere, 
                             double *R){

//
//void SphereProjector::project(const ImageOf<PixelRgb> &in, 
//                             ImageOf<PixelRgb> &sphere, 
//                             double azimuth, 
//                             double elevation,
//                             double rotation){
                             
    //CvSize inSize = cvSize(in.width(),in.height());
    /*double inSizeX2 = ((double)inSize.width)/2.0;
    double inSizeY2 = ((double)inSize.height)/2.0;*/
    //CvSize sphereSize = cvSize(sphere.width(), sphere.height());
    /*double sphereSizeX2 = ((double)sphereSize.width)/2.0;
    double sphereSizeY2 = ((double)sphereSize.height)/2.0;
    double scale = _pixPerDegXSphere / _pixPerDegXIn;*/
    //cout << "scale: " << scale << endl;

    // check if reallocation required
    if ( in.width()  != _oldInSize.width || 
         in.height() != _oldInSize.height || 
        _needInit)
        init(cvSize(in.width(), in.height()),_calibImgSize);

    if ( sphere.width() != _oldSphereSize.width ||
         sphere.height() != _oldSphereSize.height)
        initSphere(cvSize(sphere.width(), sphere.height()));

    
    /*cout << _fa << " " << _fe << " " << endl;
    cout << _ca << " " << _ce << " " << endl;
    cout << endl;*/

    /*cout << _fx_scaled << " " << _fy_scaled << " " << endl;
    cout << _cx_scaled << " " << _cy_scaled << " " << endl;
    cout << endl;*/

    //compute_icub_egosp_map -> conversion done now in egospheremodule already
    compute_egosp_map( in.height(), in.width(),
                        sphere.height(), sphere.width(),
                        _fx_scaled, _fy_scaled, // azimuth elevation
                        _cx_scaled,  _cy_scaled,
                        R, //world to camera rotation
                        (float*)_mapx->imageData, (float*)_mapy->imageData);

    cvRemap(in.getIplImage(), sphere.getIplImage(), _mapx, _mapy, CV_INTER_LINEAR);

    //cout << "pixel per degree in: " << _pixPerDegXIn << " pixel per degree sphere: " << _pixPerDegXSphere << endl;

    // rotate around input image center (rotation) + scaling
    //cv2DRotationMatrix( cvPoint2D32f( inSizeX2, inSizeY2),
    //                    rotation,
    //                    scale, 
    //                    _map);
    //// set correct shift within sphere (azimuth, elevation)
    //CV_MAT_ELEM(*_map, float, 0, 2) += azimuth*_pixPerDegXSphere + 
    //                                   sphereSizeX2 - 
    //                                   inSizeX2;
    //CV_MAT_ELEM(*_map, float, 1, 2) += elevation*_pixPerDegYSphere + 
    //                                   sphereSizeY2 - 
    //                                   inSizeY2;

    /*cvWarpAffine( in.getIplImage(), 
                  sphere.getIplImage(), 
                  _map,
                  CV_INTER_LINEAR,
                  cvScalarAll(0) );*/
   /* cvWarpAffine( in.getIplImage(), 
                  _yrpImgBase.getIplImage(), 
                  _map,
                  CV_INTER_LINEAR,
                  cvScalarAll(0) );*/

    //// temp. build map
    //IplImage *_mapFlatX = cvCreateImage(sphereSize, IPL_DEPTH_32F, 1);
    //IplImage *_mapFlatY = cvCreateImage(sphereSize, IPL_DEPTH_32F, 1);
    //float *mapX = (float*)_mapFlatX->imageData;
    //float *mapY = (float*)_mapFlatY->imageData;
    //const double pi = 4.0 * atan(1.0);
    //double stretch = 0;
    //for (int y = 0; y < sphereSize.height; y++){
    //    //((double)y)-sphereSizeY2) * 360.0 / sphereSize.height
    //    stretch = cos(elevation*pi/180.0); // stretch factor
    //    for (int x = 0; x < sphereSize.width; x++){
    //        *mapX++ = (x-sphereSizeX2)*stretch+sphereSizeX2;
    //        *mapY++ = y;
    //    }
    //}
    //cvRemap(_yrpImgBase.getIplImage(), sphere.getIplImage(), _mapFlatX, _mapFlatY);
    //cvReleaseImage(&_mapFlatX);
    //cvReleaseImage(&_mapFlatY);

    //cout << "projecting..." << endl;

    // buffering old image size
    _oldInSize.width  = in.width();
    _oldInSize.height = in.height();
    _oldSphereSize.width = sphere.width();
    _oldSphereSize.height = sphere.height();
}
