/** 
 * @ingroup icub_module
 *
 * \defgroup icub_particleFiltering particleModule
 *
 * Object tracking is a tricky problem. A general, all-purpose object tracking algorithm must deal with difficulties like camera motion, erratic object motion, cluttered backgrounds, and other moving objects. Such hurdles render general image processing techniques an inadequate solution to the object tracking problem.

Particle filtering is a Monte Carlo sampling approach to Bayesian filtering. It has many uses but has become the state of the art in object tracking. Conceptually, a particle filtering algorithm maintains a probability distribution over the state of the system it is monitoring, in this case, the state -- location, scale, etc. -- of the object being tracked. In most cases, non-linearity and non-Gaussianity in the object's motion and likelihood models yields an intractable filtering distribution. Particle filtering overcomes this intractability by representing the distribution as a set of weighted samples, or particles. Each particle represents a possible instantiation of the state of the system. In other words, each particle describes one possible location of the object being tracked. The set of particles contains more weight at locations where the object being tracked is more likely to be. We can thus determine the most probable state of the object by finding the location in the particle filtering distribution with the highest weight.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP
 * OPENCV
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters <\b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c particleMod.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c particleFiltering/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c particleMod \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * <b>Configuration File Parameters </b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c inputPortNameTemp \c /particleMod/template/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inputPortNameLeft \c /particleMod/left/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inputPortNameRight \c /particleMod/right/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameLeft \c /particleMod/left/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameRight \c /particleMod/right/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 *   \c outputPortNameLeftBlob \c /particleMod/leftblob/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameLeftBlob \c /particleMod/rightblob/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outputPortNameTarget \c /particleMod/target:o \n  
 *   specifies the output port name (this string will be prefixed by \c /particleMod 
 *   or whatever else is specifed by the name parameter
 *
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /particleMod \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /yuvProc
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /particleMod/template/image:i \n
 *  - \c /particleMod/left/image:i \n
 *  - \c /particleMod/right/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /particleMod \n
 *    see above
 *
 *  - \c /particleMod/left/image:o \n
 *  - \c /particleMod/right/image:o \n
 *  - \c /particleMod/leftblob/image:o \n
 *  - \c /particleMod/rightblob/image:o \n
 *  - \c /particleMod/target:o \n
 *
 * <b>Port types </b>
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c particleMod.ini  in \c $ICUB_ROOT/app/particleFiltering/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10 and Debian Stable 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>YUVProcessor --name yuvProc --context particleFiltering/conf --from particleMod.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff, Andrew Dankers
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/particleMod/include/iCub/particleMod.h
 * 
 */


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
#define TRANS_X_STD 1.0
#define TRANS_Y_STD 0.5
#define TRANS_S_STD 0.001
/* autoregressive dynamics parameters for transition model */
#define pfot_A1  2.0
#define pfot_A2 -1.0
#define pfot_B0  1.0000
/* distribution parameter */
#define LAMBDA 20

 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  
class PARTICLEThread : public Thread {
private:

    /*port name strings*/
    string inputPortNameTemp;
    string inputPortNameLeft;
    string inputPortNameRight;

    string outputPortNameLeft;  
    string outputPortNameRight; 
    string outputPortNameLeftBlob;  
    string outputPortNameRightBlob;     
    string outputPortNameTarget;

    /* thread parameters: they are pointers so that they refer to the original variables */

    BufferedPort<ImageOf<PixelRgb> >  imageInTemp;
    BufferedPort<ImageOf<PixelRgb> >  imageInLeft;
    BufferedPort<ImageOf<PixelRgb> >  imageInRight;

    BufferedPort<ImageOf<PixelBgr> > imageOutLeft;
    BufferedPort<ImageOf<PixelBgr> > imageOutRight;  
    BufferedPort<ImageOf<PixelMono> > imageOutLeftBlob;
    BufferedPort<ImageOf<PixelMono> > imageOutRightBlob;  

    BufferedPort < Vector >    target; //vector containing the tracked target 2D and its probability
    
    CvPoint		minloc, maxloc;
	double		minval, maxval;
    CvPoint		minloc1, maxloc1;
	double		minval1, maxval1;

    bool init;
    bool getImageLeft, getImageRight, getTemplate, gotTemplate, sendTarget;
    ImageOf<PixelRgb> *iCubleft, *iCubright;
    ImageOf<PixelRgb> *tpl;
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
    string moduleName;

public:

    /* class methods */
    PARTICLEThread( );
    
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

    void runAll(IplImage *left, IplImage *right, Vector& target);

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
	void display_particle( IplImage* img, particle p, CvScalar color, Vector& target );
    void display_particleBlob( IplImage* img, particle p, Vector& target );
	
	void normalize_histogram( histogram* histo );
    void initAll();
    bool threadInit();     
    void threadRelease();
    void run(); 
    //void onStop();
    void setName(string module);
};

class particleMod:public RFModule {

    /* module parameters */
    string moduleName;
    string handlerPortName;

    Port handlerPort;      //a port to handle messages 
    
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    PARTICLEThread *particleThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif
//empty line to make gcc happy

int particle_cmp( const void* p1, const void* p2 );
