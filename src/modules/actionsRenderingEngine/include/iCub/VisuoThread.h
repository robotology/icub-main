/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Carlo Ciliberto, Vadim Tikhanoff
* email:   carlo.ciliberto@iit.it vadim.tikhanoff@iit.it
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

#ifndef __VISUO_THREAD__
#define __VISUO_THREAD__



#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <cv.h>

#include <string>
#include <deque>
#include <map>

#include <iCub/utils.h>

#define PARAM_FIXATION              VOCAB4('f','i','x','a')
#define PARAM_MOTION                VOCAB4('m','o','t','i')
#define PARAM_TRACK                 VOCAB4('t','r','a','c')
#define PARAM_RAW                   VOCAB3('r','a','w')

#define MODE_TRACK_TEMPLATE 0
#define MODE_TRACK_MOTION   1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


struct StereoTracker
{
    Vector      vec;
    int         side;
};




class VisuoThread: public RateThread
{
private:
    ResourceFinder                          &rf;

    StereoTarget                            &stereo_target;
    ObjectPropertiesCollectorPort           &opcPort;

    int                                     trackMode;

    bool                                    interrupted;

    Port                                    outPort[2];
    BufferedPort<ImageOf<PixelRgb> >        imgPort[2];
    BufferedPort<Bottle>                    mCUTPort[2];
    BufferedPort<Bottle>                    rawInPort[2];
    BufferedPort<Vector>                    pftInPort;
    Port                                    pftOutPort;         //send template to the pft throught this port


    Port                                    segPort;

    //MIL Ports
    Port                                    boundMILPort;
    RpcClient                               cmdMILPort;
    BufferedPort<Bottle>                    recMILPort;

    //multiSensoryObjectRecognition Ports
    Port                                    cmdMSRPort;
    BufferedPort<Bottle>                    recMSRPort;

    Semaphore                   imgMutex;
    Semaphore                   motMutex;
    Semaphore                   MILMutex;
    Semaphore                   trackMutex;

    unsigned int                minMotionBufSize;
    unsigned int                minTrackBufSize;
    unsigned int                maxTrackBufSize;
    double                      timeTol;
    double                      motionStdThresh;
    double                      speedStdThresh;
    double                      stereoDistThresh;

    double                      rawWaitThresh;
    double                      motionWaitThresh;
    double                      objectWaitThresh;

    int                         dominant_eye;
    ImageOf<PixelRgb>           *img[2];
    ImageOf<PixelBgr>           tpl;

    StereoTracker               stereoTracker;
    deque<Vector>               trackBuffer;
    bool                        tracking;

    bool                        newImage[2];

    bool                        show;
    bool                        closed;

    struct Item
    {
        double  t;
        double  size;
        CvPoint p;
    };

    deque<Item>                 buffer[2];
    //map<string,CvPoint>         locations[2];
    map<string,CvPoint>         locations;


    bool getFixation(Bottle &bStereo);
    bool getMotion(Bottle &bStereo);
    bool getTrack(Bottle &bStereo);
    bool getRaw(Bottle &bStereo);
    bool getObject(const string &object_name, Bottle &bStereo);


    void close();
    void updateImages();
    void updateLocationsMIL();
    void updateMotionCUT();
    void updatePFTracker();
public:
    VisuoThread(ResourceFinder &_rf, Initializer *initializer);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();


    bool isTracking()
    {
        trackMutex.wait();
        bool track=tracking;
        trackMutex.post();

        return track;
    }

    bool checkTracker(Vector *vec);

    void startTracker(const Vector &stereo, const int &side);
    void restartTracker();


    bool   trackMotion()
    {
        trackMode=MODE_TRACK_MOTION;
        return true;
    }

    bool getTarget(Value &type, Bottle &target);

    Bottle recogMSR(string &obj_name);


    void doShow()
    {
        show=!show;
    }



    bool startLearningMIL(const string &obj_name);
    bool suspendLearningMIL();
    bool resumeLearningMIL();
    bool trainMIL();

    bool startLearningMSR(const string &obj_name, const int &arm);
    bool startRecogMSR(const int &arm);
    bool suspendLearningMSR();
    bool resumeLearningMSR();
    bool stopMSR();
    void checkDoneMSR(bool &done);

    void interrupt();
    void reinstate();
};

#endif


