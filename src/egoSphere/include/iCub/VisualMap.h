// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __VISUALMAP__
#define __VISUALMAP__

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// opencv
#include <cv.h>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/spherical_projection.h>
#include <iCub/IModalityMap.h>

namespace iCub {
    namespace contrib {
        class VisualMap;
    }
}

/**
 *
 * Visual map 
 *
 * \see icub_egosphere
 *
 */
class iCub::contrib::VisualMap : public iCub::contrib::IModalityMap, public yarp::os::IConfig {

private:

    int _resXVisual; // resolution of visual maps
    int _resYVisual;

	bool			_drawGazeDirectionREye;
	bool			_drawGazeDirectionLEye;
	bool			_useRightEye; // use left eye if 'false'
    double          _fx, _fx_scaled_flt, _fx_scaled_rgb;
    double          _fy, _fy_scaled_flt, _fy_scaled_rgb;
    double          _cx, _cx_scaled_flt, _cx_scaled_rgb;
    double          _cy, _cy_scaled_flt, _cy_scaled_rgb;
    double          _k1, _k2, _p1, _p2;
    double          _fa_flt, _fe_flt, _ca_flt, _ce_flt;
	double          _fa_rgb, _fe_rgb, _ca_rgb, _ce_rgb;
    double          _azSpanFlt;    // total image angle horizontal
    double          _elSpanFlt;    // total image angle vertical 
	double          _azSpanRgb;    // total image angle horizontal
    double          _elSpanRgb;    // total image angle vertical 

	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >		_prtImgRgbIn;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >		_prtImgRgbOut;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >		_prtImgFloatIn;
	yarp::os::BufferedPort<yarp::os::Bottle>					_prtImgRgbClickIn;

	yarp::sig::ImageOf<yarp::sig::PixelFloat>	*_imgFloatIn; // pointer to input image read from port
	yarp::sig::ImageOf<yarp::sig::PixelRgb>	*_imgRgbIn; // pointer to input image read from port
	yarp::os::Bottle				*_botClick; // pointer to input bottle read from port

    yarp::sig::ImageOf<yarp::sig::PixelFloat>	_imgRemapX; // cartesian to spherical remap map X
    yarp::sig::ImageOf<yarp::sig::PixelFloat>	_imgRemapY; // cartesian to spherical remap map Y
    yarp::sig::ImageOf<yarp::sig::PixelFloat>	_imgMapResA; // final map, visual resolution
    yarp::sig::ImageOf<yarp::sig::PixelRgb>		_imgMapResARgb; // final map rgb visualization
    yarp::sig::ImageOf<yarp::sig::PixelFloat>	_imgMapResB; // final map, final resolution
    yarp::sig::ImageOf<yarp::sig::PixelRgb>		_imgMapResBRgb; // final map rgb visualization, final resolution
    
    double _salienceDecayRate;

	CvSize _egoImgSize;
    CvSize _calibImgSize;
    CvSize _oldImgSizeFlt;
	CvSize _oldImgSizeRgb;

	CvPoint _clickPoint;

	yarp::sig::PixelRgb _gazePixREye;
	yarp::sig::PixelRgb _gazePixLEye;
	yarp::sig::PixelFloat _clickPix;

    // convert from RobMatrix to 1 dimensional 3x3 -> 1x9 array
    void convertRobMatrix(RobMatrix &robMatrix, double *matrix);
    inline void transpose(double *src, double *dst){
        dst[0] = src[0]; dst[1] = src[3]; dst[2] = src[6];
        dst[3] = src[1]; dst[4] = src[4]; dst[5] = src[7];
        dst[6] = src[2]; dst[7] = src[5]; dst[8] = src[8];
    }
    void initFlt(	CvSize currImgSizeFlt, CvSize calibImgSize);
	void initRgb(   CvSize currImgSizeRgb, CvSize calibImgSize);

	void processClick();

public:

    VisualMap();
    virtual ~VisualMap();
    
	bool read();
	bool updateDecay();
	bool process(	double azREye, double elREye, int xREye, int yREye, double *rotREye,
					double azLEye, double elLEye, int xLEye, int yLEye, double *rotLEye,
					double azHead, double elHead, int xHead, int yHead, double *rotHead,
					bool blnSaccadicSuppression);
	bool write(bool saccadicSuppression);    

    /** 
    Resizes internal acoustic map to match requested resolution and 
    returns a reference to the resized map
    @param resX requested width
    @param resY requested height
    */
	yarp::sig::ImageOf<yarp::sig::PixelFloat>& getSalienceMap(int width, int height);	
	yarp::sig::ImageOf<yarp::sig::PixelRgb>& getVisualizationMap(int width, int height);

    /** allow to plug to emote reset */
    bool reset();

    /** Passes config on to iCub::contrib::CalibTool */
	bool open(yarp::os::Searchable& config);
    bool close();
	bool interrupt();

    // some egosphere controls
    bool setSalienceDecay(double rate);
    double getSalienceDecay();

};


#endif
