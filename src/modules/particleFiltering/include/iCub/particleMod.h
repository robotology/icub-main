/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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

#ifndef __ICUB_PARTICLE_MOD_H__
#define __ICUB_PARTICLE_MOD_H__

#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <time.h>

/* From GSL */
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

/* default number of particles */
#define PARTICLES 200
/* maximum number of objects to be tracked */
#define MAX_OBJECTS 1
/* number of bins of HSV in histogram */
#define NH 10
#define NS 10
#define NV 10
/* low thresholds on saturation and value for histogramming */
#define S_THRESH 0.1
#define V_THRESH 0.2
/* max HSV values */
#define H_MAX 360.0
#define S_MAX 1.0
#define V_MAX 1.0
/* standard deviations for gaussian sampling in transition model */
#define TRANS_X_STD 1.0f
#define TRANS_Y_STD 0.5f
#define TRANS_S_STD 0.001f
/* autoregressive dynamics parameters for transition model */
#define pfot_A1  2.0f
#define pfot_A2 -1.0f
#define pfot_B0  1.0000f
/* distribution parameter */
#define LAMBDA 20
  
class PARTICLEThread : public yarp::os::Thread {
private:

    /*port name strings*/
    std::string inputPortNameTemp;
    std::string inputPortNameLeft;
    std::string inputPortNameRight;

    std::string outputPortNameLeft;  
    std::string outputPortNameRight; 
    std::string outputPortNameLeftBlob;  
    std::string outputPortNameRightBlob;     
    std::string outputPortNameTarget;

    /* thread parameters: they are pointers so that they refer to the original variables */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageInTemp;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageInLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageInRight;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageOutLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageOutRight;  
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutLeftBlob;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutRightBlob;  

    yarp::os::BufferedPort < yarp::sig::Vector >    target; //vector containing the tracked target 2D and its probability
    
    CvPoint		minloc, maxloc;
	double		minval, maxval;
    CvPoint		minloc1, maxloc1;
	double		minval1, maxval1;

    bool init;
    bool getImageLeft, getImageRight, getTemplate, gotTemplate, sendTarget;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *iCubleft, *iCubright;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *tpl;
    IplImage *temp, *res_left, *res_right, *res;

    IplImage* left_frame, * right_frame, *left_frame_blob, * right_frame_blob;
    int width_L, height_L, width_R, height_R, tpl_width, tpl_height, res_width, res_height;
    double scale;
	IplImage* left_hsv;
    IplImage* right_hsv;
	gsl_rng* rng;
	bool firstFrame;
  	CvScalar color;
  	CvRect** left_regions;
    CvRect** right_regions;
  	int num_objects;
  	float s;
  	int i, j, k, w, h, x, y;
	int num_particles;
    //string containing module name
    std::string moduleName;

public:

    /* class methods */
    PARTICLEThread();
    ~PARTICLEThread();

    typedef struct params {
  		CvPoint loc1[MAX_OBJECTS];
  		CvPoint loc2[MAX_OBJECTS];
  		IplImage* objects[MAX_OBJECTS];
  		char* win_name;
  		IplImage* orig_img;
  		IplImage* cur_img;
  		int n;
	} params;

	typedef struct histogram {
  		float histo[NH*NS + NV];   /* histogram array */
  		int n;                     /* length of histogram array */
	} histogram;
	
	typedef struct particle {
	  	float x;          /* current x coordinate */
	  	float y;          /* current y coordinate */
	  	float s;          /* scale */
	  	float xp;         /* previous x coordinate */
	  	float yp;         /* previous y coordinate */
	  	float sp;         /* previous scale */
	  	float x0;         /* original x coordinate */
	  	float y0;         /* original y coordinate */
	  	int width;        /* original width of region described by particle */
	  	int height;       /* original height of region described by particle */
  		histogram* histo; /* reference histogram describing region being tracked */
  		float w;          /* weight */
	} particle;

	histogram** ref_histos_left;
    histogram** ref_histos_right;
	particle* particles_l, * new_particles_l;
    particle* particles_r, * new_particles_r;

    void runAll(IplImage *left, IplImage *right, yarp::sig::Vector& target);

    void free_histos( histogram** histo, int n );
    void free_regions( CvRect** regions, int n);

    histogram** compute_ref_histos( IplImage* img, CvRect* rect, int n );
	histogram* calc_histogram( IplImage** imgs, int n );
	particle transition( particle p, int w, int h, gsl_rng* rng );
	particle* init_distribution( CvRect* regions, histogram** histos, int n, int p);
	IplImage* bgr2hsv( IplImage* bgr );
	float likelihood( IplImage* img, int r, int c, int w, int h, histogram* ref_histo );
	void normalize_weights( particle* particles, int n );
	float histo_dist_sq( histogram* h1, histogram* h2 );
	int histo_bin( float h, float s, float v );
	float pixval32f(IplImage* img, int r, int c);
	void setpix32f(IplImage* img, int r, int c, float val);
	int get_regions( IplImage* frame, CvRect** regions );
    int get_regionsImage( IplImage* frame, CvRect** regions );
	particle* resample( particle* particles, int n );
	void display_particle( IplImage* img, particle p, CvScalar color, yarp::sig::Vector& target );
    void display_particleBlob( IplImage* img, particle p, yarp::sig::Vector& target );
	
	void normalize_histogram( histogram* histo );
    void initAll();
    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();
    void setName(std::string module);
};

class particleMod:public yarp::os::RFModule {

    /* module parameters */
    std::string moduleName;
    std::string handlerPortName;

    yarp::os::Port handlerPort;      //a port to handle messages 
    
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    PARTICLEThread *particleThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif
//empty line to make gcc happy

int particle_cmp( const void* p1, const void* p2 );
