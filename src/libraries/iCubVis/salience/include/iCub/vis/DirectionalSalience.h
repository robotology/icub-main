// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_DIRECTIONALSALIENCE_INC
#define ICUB_DIRECTIONALSALIENCE_INC

// std
#include <map>

//yarp
#include <yarp/os/all.h>

// iCub
#include <iCub/vis/Salience.h>
#include <iCub/vis/FastGabor.h>
#include <iCub/vis/FastGauss.h>

namespace iCub {
    namespace vis {
        class DirectionalSalience;
    }
}

/**
 * Gabor filters with conspicuity following Alexandre Bernardino
 */
class iCub::vis::DirectionalSalience : public Salience,
                                           public IDirectionalSalienceControls {
private:
    float MAX_SALIENCE; // salience ranges from 0.0...MAX_SALIENCE
    int _w, _ws;    // src image width and scaled width
    int _h, _hs;    // src image height and scaled height
    double _scale;  // calculate directional salience at scale _scale 
    int _numDirs;
    double *_dirs;  // filter directions (angles in degree)
    int _numSizs;   // number of wavelet sizes per direction
    double *_sigs;  // sigmas for gabors
    double *_wavs;  // wavelengths for gabors
    double _sig2wav; // _wav[i] = _sig[i] * _sig2wav;
    double _centerSize; // _sigs[s] * _centerSize
    double _surroundSize; // _sigs[s] * _surroundSize
    std::map<string, IplImage***>  _mapImgTable; // name to image (-arrays) mapping for debug/analysis
    string _dbgImgArray; // name of the requested image array to output to debuggin port
    int _dbgDirIndex; // index of filter direction to debug
    int _dbgSizIndex; // index of filter size to debug
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > _prtDebug; // debug
    IplImage *_imgInRgbResized;
    IplImage *_imgInGray;
    IplImage *_imgInFloat;
    IplImage *_imgGReal; // gabor filter images
    IplImage *_imgGImag;
    //IplImage *_imgGAbs;
    //IplImage *_imgTmp;
    //IplImage *_imgTmpCen;
    //IplImage *_imgTmpSur;
    //IplImage *_imgDSumCen; // sum of direction center D's for a certain size
    //IplImage *_imgDSumSur; // sum of direction surround D's for a certain size
    //IplImage *_imgDSumCenEx; // _imgDSumCen - currentDirection
    //IplImage *_imgDSumSurEx; // _imgDSumSur - currentDirection
    //IplImage *_imgCinh; // constant inhibitory values for competition calculation
    // 2d arrays of image pointers, index [sigma_index][theta_index], length _numSizs, _numDirs
    // sigma = gabor filter size, (wavelength = 4*sigma)
    // theta = gabor filter orientation
    IplImage ***_imgGabor; // gabor filtered images (kept in memory for analysis purposes)
    IplImage ***_imgDCen; // center D's gaussian sigma = gabor sigma
    IplImage ***_imgDSur; // surround D's gaussian sigma = 4 * gabor sigma
    IplImage ***_imgD; // D plus maps
    //IplImage ***_imgDMin; // D minus maps
    //IplImage ***_imgDPluComp; // D plus competition maps
    //IplImage ***_imgDMinComp; // D minus competition maps
    IplImage *_imgResult; // the final (resized) image
    float **_conspD; // 2d array of conspicuity factors for the _imgDPlu array
    //float **_conspDMin; // 2d array of conspicuity factors for the _imgDMin array
    // 2d array of gabor filters of different sizes and different orientations
    FastGabor ***_gabor;    // index [sigma_index][theta_index], length _numSizs, _numDirs
    // 1d arrays of gaussian filters of different sizes
    FastGauss **_gaussCen;  // index [sigma_index], length _numSizs
    FastGauss **_gaussSur;  // index [sigma_index], size _numSizs
    //FastGauss **_gaussCompCen; // index [sigma_index], length _numSizs
    //FastGauss **_gaussCompSur; // index  [sigma_index], length _numSizs
    bool _debug; // enable debugging stuff
    bool _verbose; // output debugging information
    yarp::os::Semaphore _mutex;
    void init(int w, int h);
    void release();
    inline void calcGlobalMaximum(const IplImage *img, float &max, int &maxX, int &maxY);
    inline void calcGlobalMinimum(const IplImage *img, float &min);
    void calcConspicuityFactor(const IplImage *img, float &factor);
    void calcAdvancedConspicuityFactor(const IplImage *img, const double &sigmaSurround, const int &globMaxX, const int &globMaxY, float &factor);
public:
    DirectionalSalience();
    virtual ~DirectionalSalience();
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    virtual bool respond(const Bottle &command,Bottle &reply);
    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
               yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
               yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);
    // DirectionalSalienceControls
    virtual int getNumberScales();
    virtual int getNumberDirections();
    virtual bool setDebugFilterScaleIndex(int size);
    virtual bool setDebugFilterDirectionIndex(int direction);
    virtual yarp::os::Bottle getDebugImageArrayNames();
    virtual bool setDebugImageArrayName(string imgArray);
};

#endif
