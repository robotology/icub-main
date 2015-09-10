/*
 * A bottom-up approach for generating particles for the "pf3dTracker" module
 *
 * Copyright (C) 2010 RobotCub Consortium
 *
 * Author: Martim Brandao
 * Note: Should you use or reference my work on your own research, please let me know (mbrandao _AT_ isr.ist.utl.pt)
 *
 * Image sequence as input, N particles (3D position of balls) as output.
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <string>
#include <sstream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/ScaleSpace.h>

#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#endif

// my definitions
#define PI 3.1415926535897932384626433832795
#define SCATTER(s) ( (s*(double)rand()/(double)RAND_MAX)-0.5*s )

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

// module
class pf3dBottomup : public RFModule
{

private:


// my definitions
typedef struct MbMoments
{
    double  m00, m10, m01;
    double  m[12];
}MbMoments;

typedef struct CameraModel
{
    double fx;
    double fy;
    double cx;
    double cy;
    double fov;
    double aspect;
    double znear;
    double zfar;
}CameraModel;

typedef struct ObjectModel
{
/*
    static const int QUANT_A=12;        //this should be made variable... (dynamic init.)
    static const int NUM_MOM=12;        //this should be made variable... (dynamic init.)
    static const int NUM_POSES=1008;    //this should be made variable... (dynamic init.)
    double data[NUM_POSES][7+NUM_MOM];
    double moment_variance[NUM_MOM];
    double prob[NUM_POSES];
    double prob_acum[NUM_POSES];
    float prob_particles[NUM_POSES];
*/
    double raio_esfera;
    CvHistogram *hist;
    CvMat *particles;
}ObjectModel;


// parameters set during initialization.
ConstString _inputVideoPortName;
BufferedPort<ImageOf<PixelRgb> > _inputVideoPort;
ConstString _outputParticlePortName;
BufferedPort<Bottle> _outputParticlePort;

double _perspectiveFx;
double _perspectiveFy;
double _perspectiveCx;
double _perspectiveCy;
int _calibrationImageWidth;
int _calibrationImageHeight;
bool _doneInitializing;

int _nParticles;

ImageOf<PixelRgb> *_yarpImage;


// my parameters
int _maskVmin, _maskVmax, _maskSmin, _blur;
int _scaleSpaceLevels;
double _scaleSpaceScales[3];


// global instances
CameraModel _camera;
ObjectModel _object_model;

ScaleSpace ss;
IplImage *image, *infloat, *hsv, *hue, *sat, *val, *mask, *backproject, *backprojectmask2;


void calc_hist_from_model_2D(string file, CvHistogram **objhist, int _vmin, int _vmax);
void normalize_to_global_max(IplImage *img);
void scale_space_segmentation(IplImage *img, ScaleSpace *ss, IplImage *result);
int object_localization_simple(IplImage *segm, ObjectModel *model, CameraModel *camera);


public:

pf3dBottomup(); //constructor
~pf3dBottomup(); //destructor

virtual bool configure(ResourceFinder &rf); //member to set the object up.
virtual bool close();                       //member to close the object.
virtual bool interruptModule();             //member to close the object.
virtual bool updateModule();                //member that is repeatedly called by YARP

};


// changed functions
void cvFloodFill2( CvArr* arr, CvPoint seed_point, CvScalar newVal, CvScalar lo_diff, CvScalar up_diff, CvConnectedComp* comp, int flags, CvArr* maskarr );


