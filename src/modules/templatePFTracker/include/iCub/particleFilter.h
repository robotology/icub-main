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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/os/RpcClient.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <mutex>
#include <time.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <deque>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

/* default number of particles */
#define PARTICLES 1000
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
#define TRANS_X_STD 10.0f
#define TRANS_Y_STD 7.5f
#define TRANS_S_STD 0.001f
/* autoregressive dynamics parameters for transition model */
#define pfot_A1  2.0f
#define pfot_A2 -1.0f
#define pfot_B0  1.0000f
/* distribution parameter */
#define LAMBDA 10

/* templates list parameters */
#define TEMP_LIST_SIZE                  10
#define TEMP_LIST_PARTICLE_THRES_HIGH   0.9
#define TEMP_LIST_PARTICLE_THRES_LOW    0.5

typedef struct TemplateStruct 
{
    float                                       w;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>     *templ;
} TemplateStruct;


int particle_cmp( const void* p1, const void* p2 );


class PARTICLEThread : public yarp::os::Thread 
{
public:
    typedef struct histogram 
    {
        float histo[NH*NS + NV];  /* histogram array */
        int n;                    /* length of histogram array */
    } histogram;

    typedef struct particle 
    {
        float x;                  /* current x coordinate */
        float y;                  /* current y coordinate */
        float s;                  /* scale */
        float xp;                 /* previous x coordinate */
        float yp;                 /* previous y coordinate */
        float sp;                 /* previous scale */
        float x0;                 /* original x coordinate */
        float y0;                 /* original y coordinate */
        int width;                /* original width of region described by particle */
        int height;               /* original height of region described by particle */
        histogram* histo;         /* reference histogram describing region being tracked */
        float w;                  /* weight */
    } particle;

    float average;

private:
    /*port name strings*/    
    std::string inputPortName;
    std::string outputPortName;  
    std::string outputPortNameBlob;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >  imageOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutBlob;

    bool init;
    bool getImage, getTemplate, gotTemplate, sendTarget;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *iCubImage;

    cv::Mat temp;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *tpl;

    cv::Mat frame;
    cv::Mat frame_blob;
    int width, height, tpl_width, tpl_height;
    cv::Mat img_hsv;
    gsl_rng* rng;
    bool firstFrame;
    cv::Scalar color;
    std::vector<cv::Rect> regions;
    int num_objects;
    float s;
    int i, j, k, w, h, x, y;
    int num_particles;
    yarp::sig::Vector targetTemp;
    //string containing module name
    std::string moduleName;
    std::mutex targetMutex, averageMutex, templateMutex;

    std::deque< TemplateStruct > tempList;

    bool                        updateNeeded;
    TemplateStruct              bestTempl;
    yarp::os::Stamp             targetStamp;

    histogram** ref_histos;
    particle* particles, * new_particles;    

    void free_histos( histogram** histo, int n );

    histogram** compute_ref_histos( const cv::Mat& img, const std::vector<cv::Rect>& rects );
    histogram* calc_histogram( const cv::Mat* imgs, int n );
    particle transition( const particle &p, int w, int h, gsl_rng* rng );
    particle* init_distribution( const std::vector<cv::Rect>& regions, histogram** histos, int n, int p);
    cv::Mat bgr2hsv( const cv::Mat& bgr );
    float likelihood( const cv::Mat& img, int r, int c, int w, int h, histogram* ref_histo );
    void normalize_weights( particle* particles, int n );
    float histo_dist_sq( histogram* h1, histogram* h2 );
    int histo_bin( float h, float s, float v );
    int get_regionsImage( const cv::Mat& frame, std::vector<cv::Rect>& regions );
    particle* resample( particle* particles, int n );
    void display_particle( cv::Mat& img, const particle &p, const cv::Scalar& color, yarp::sig::Vector& target );
    void display_particleBlob( cv::Mat& img, const particle &p, yarp::sig::Vector& target );
    void trace_template( const cv::Mat& img, const particle &p );
    void normalize_histogram( histogram* histo );
    void initAll();
    void runAll(cv::Mat& img);
   

public:
    /* class methods */
    PARTICLEThread();
    ~PARTICLEThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void setName(std::string module);
    void setTemplate(yarp::sig::ImageOf<yarp::sig::PixelRgb> *tpl);
    void pushTarget(yarp::sig::Vector &target, yarp::os::Stamp &stamp);
    float getAverage();
    TemplateStruct getBestTemplate();
};


class PARTICLEManager : public yarp::os::PeriodicThread 
{
private:
    
    std::string inputPortNameTemp;
    std::string outputPortNameTarget;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageInTemp;
    yarp::os::Port target; //vector containing the tracked target 2D and its probability
    yarp::os::Port disparityPort;
    yarp::os::Port target3D;

    PARTICLEThread *particleThreadLeft;
    PARTICLEThread *particleThreadRight;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *tpl;
    std::string moduleName;
    

public:
    PARTICLEManager();
    ~PARTICLEManager();
    bool            shouldSend;

    void setName(std::string module);
    bool threadInit();     
    void threadRelease();
    void run(); 
};


class PARTICLEModule:public yarp::os::RFModule 
{

    /* module parameters */
    std::string moduleName;
    std::string handlerPortName;

    yarp::os::Port handlerPort;      //a port to handle messages 
    
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    PARTICLEManager *particleManager;
    

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
