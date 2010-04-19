/** 
\defgroup faceTracker faceTracker 
 
@ingroup icub_module  
 
A face tracker based on the Haar detector.

Copyright (C) 2009 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module uses the OpenCV Haar detector in order to find a 
face in front of the robot. Once detected it sends out data to 
control the head motion for tracking through the iKinGazeCtrl 
module and also to move the hand to point at the gazing location
through the iKinArmCtrl module. 
 
The robot will gaze around until it will find a face and then it 
will remain on the last detected face (even in presence of more 
than one). 
 
\note A video on iCub tracking a face can be seen <a 
      href="http://eris.liralab.it/misc/icubvideos/faceTracker.avi">here</a>.

\section lib_sec Libraries 
- YARP libraries. 
- OpenCV libraries 

\section parameters_sec Parameters
--name \e modName 
- The parameter \e modName identifies the stem-name of the open 
  ports.
 
--period \e T 
- The integer \e T specifies the module period in [ms].
 
--eye \e sel 
- The parameter \e sel selects the used eye: it can be "left" or
  "right".
 
--arm \e sel 
- The parameter \e sel selects the arm used for pointing: it can
  be "left" or "right".
 
--eyeDist \e d 
- The double \e d specifies the fixed distance of the recognized
  face from the selected eye (to be passed to the \ref
  icub_iKinHead module).
 
--holdoff \e dT 
- The double \e dT specifies the temporal window after a 
  detected top-down input during which the "search-for-face"
  stage is disabled.
 
--descriptor \e fileName 
- The parameter \e fileName identifies the description file for 
  the Haar detector.
 
\section portsa_sec Ports Accessed
 
/icub/cam/left 
/icub/cam/right 
/iKinGazeCtrl/head/x:o 
/eyeTriangulation/rpc 
/icub/face/emotions/in 

\section portsc_sec Ports Created 
 
- \e /faceTracker/img:i receives the image.
 
- \e /faceTracker/gazeat:o sends out the target 3-d component of
  the new fixation point (to be connected to the iKinGazeCtrl
  module).
 
- \e /faceTracker/reach:o sends out the target 3-d component of 
  the new fixation point (to be connected to the iKinArmCtrl
  module for reaching purpose).
 
- \e /faceTracker/img:o sends out the image acquired from the 
  left camera with a superimposed rectangle which identifies the
  detected face (to be connected to a viewer).
 
- \e /faceTracker/get3D:rpc sends a request to the \ref 
  icub_iKinHead module in order to retrieve the 3d location of
  the face, knowing the its 2d location in the image plane.
 
- \e /faceTracker/topdown:i receives as input the couple u-v 
  identifying a point where to direct the gaze that is selected
  through a click in the viewer
 
- \e /faceTracker/setFace:rpc commands the facial expressions. 

\section in_files_sec Input Data Files
The description file for the Haar detector.

\section out_data_sec Output Data Files 
None. 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <time.h>

#include "facedet.h"
                              
#define STATE_SEARCH                0
#define STATE_TRACK                 1
#define STATE_TOPDOWN               2
                               
#define GAZE_X_MIN                  -0.50
#define GAZE_X_MAX                  -0.20
#define GAZE_Y_MIN                  -0.40
#define GAZE_Y_MAX                  0.40
#define GAZE_Z_MIN                  0.45
#define GAZE_Z_MAX                  0.50
#define GAZE_DELTA_MAX              0.05
                               
#define GAZE_TIMER_MAX              1.50
#define GAZE_TIMER_MIN              0.80

#define GAZE_REST_X                 -0.5
#define GAZE_REST_Y                 0.0
#define GAZE_REST_Z                 0.35

#define REACH_X_MAX                 -0.1
#define REACH_REST_X                -0.28
#define REACH_REST_Y                0.1     // given for the right arm
#define REACH_REST_Z                0.1
#define REACH_OFFS_Z                -0.1

#define FACE_EXPR_SEARCH            "shy"
#define FACE_EXPR_FOUND             "sur"
#define FACE_EXPR_TRACK             "hap"
#define FACE_EXPR_TOPDOWN           "cun"
#define FACE_EXPR_TIMEOUT           2.0

#define PICKTIMER(x)                (((double)rand()/(double)RAND_MAX)*(x))
#define PICKRAND(x)                 ((2.0*((double)rand()/(double)RAND_MAX)-1.0)*(x))
#define SAT(x,_min,_max)            ((x)>(_max)?(_max):((x)<(_min)?(_min):(x)))
#define ARMISTRACKING(state)        ((state)>=1)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class trackThread : public RateThread
{
protected:
    int state;
    string name;
    string eye;
    string arm;
    unsigned int period;
    double eyeDist;
    double holdoff;
    double timer, timerThres, t0;

    BufferedPort<ImageOf<PixelBgr> > *portImgIn,  *portImgOut;
    BufferedPort<Vector>             *portGazeAt, *portReachPoint;
    BufferedPort<Bottle>             *portTopDown;
    Port                             *portGet3D;
    Port                             *portSetFace;

    ResourceFinder &rf;

    faceDetector fd;
    CvPoint faceCentroid;

    int centroidUpdate;
    int armCmdState;

    bool enableFaceExprTrack;
    bool queuedFaceExprFlag;
    string queuedFaceExprState;

    Vector fp;

public:
    trackThread(ResourceFinder &_rf) : RateThread(50), rf(_rf)
    { }

    virtual bool threadInit()
    {
        name=rf.check("name",Value("faceTracker")).asString().c_str();
        period=rf.check("period",Value(50)).asInt();
        eye=rf.check("eye",Value("left")).asString().c_str();
        arm=rf.check("arm",Value("right")).asString().c_str();
        eyeDist=rf.check("eyeDist",Value(1.0)).asDouble();
        holdoff=rf.check("holdoff",Value(3.0)).asDouble();

        portImgIn=new BufferedPort<ImageOf<PixelBgr> >;
        string portInName="/"+name+"/img:i";
        portImgIn->open(portInName.c_str());

        portImgOut=new BufferedPort<ImageOf<PixelBgr> >;
        string portOutName="/"+name+"/img:o";
        portImgOut->open(portOutName.c_str());

        portReachPoint=new BufferedPort<Vector>;
        string portReachPointName="/"+name+"/reach:o";
        portReachPoint->open(portReachPointName.c_str());

        portGazeAt=new BufferedPort<Vector>;
        string portGazeAtName="/"+name+"/gazeat:o";
        portGazeAt->open(portGazeAtName.c_str());

        portGet3D=new Port;
        string portGet3DName="/"+name+"/get3D:rpc";
        portGet3D->open(portGet3DName.c_str());

        portTopDown=new BufferedPort<Bottle>;
        string portTopDownName="/"+name+"/topdown:i";
        portTopDown->open(portTopDownName.c_str());

        portSetFace=new Port;
        string portSetFaceName="/"+name+"/setFace:rpc";
        portSetFace->open(portSetFaceName.c_str());

        ConstString descPath=rf.findFile("descriptor");

        if (!fd.init(descPath.c_str()))
        {
            cout << "Cannot load descriptor" << endl;        
            return false;
        }

        srand((unsigned int)time(NULL));

        resetState();
        armCmdState=0;

        queuedFaceExprFlag=false;

        fp.resize(3);
        fp[0]=GAZE_X_MAX;
        fp[1]=0.0;
        fp[2]=GAZE_Z_MAX;

        setRate(period);

        t0=Time::now();

        return true;
    }

    virtual void run()
    {
        // top-down input: handled at the highest priority
        if (Bottle *topDownInput=portTopDown->read(false))
        {
            Bottle cmd, reply;
            cmd.addString("get");
            cmd.addString("3dpoint");
            cmd.addString(eye.c_str());
            cmd.addDouble(topDownInput->get(0).asInt());
            cmd.addDouble(topDownInput->get(1).asInt());
            cmd.addDouble(eyeDist);

            if (portGet3D->write(cmd,reply))
            {
                fp[0]=reply.get(0).asDouble();
                fp[1]=reply.get(1).asDouble();
                fp[2]=reply.get(2).asDouble();

                portGazeAt->prepare()=fp;
                portGazeAt->write();
            }

            armRest();
            setFace(FACE_EXPR_TOPDOWN);

            state=STATE_TOPDOWN;
            centroidUpdate=0;
            t0=Time::now();
        }

        if (state==STATE_TOPDOWN)
            if (timer>=holdoff)
                resetState();

        // get inputs
        ImageOf<PixelBgr> *pImgIn=portImgIn->read(false);        

        // seek for faces
        if (state==STATE_SEARCH && timer>timerThres)
        {            
            fp[0]=SAT(fp[0]+PICKRAND(GAZE_DELTA_MAX),GAZE_X_MIN,GAZE_X_MAX);
            fp[1]=SAT(fp[1]+PICKRAND(GAZE_DELTA_MAX),GAZE_Y_MIN,GAZE_Y_MAX);
            fp[2]=SAT(fp[2]+PICKRAND(GAZE_DELTA_MAX),GAZE_Z_MIN,GAZE_Z_MAX);

            portGazeAt->prepare()=fp;
            portGazeAt->write();

            timerThres=SAT(PICKTIMER(GAZE_TIMER_MAX),GAZE_TIMER_MIN,GAZE_TIMER_MAX);
            t0=Time::now();

            cout << "Gazing at: " << fp.toString() << endl;
        }

        // process camera image
        if (pImgIn)
        {            
            ImageOf<PixelBgr> &imgOut=portImgOut->prepare();
            imgOut=*pImgIn;
            IplImage *pVideoFrame=(IplImage*)imgOut.getIplImage();

            CvSeq *pFaceRectSeq=fd.detect(pVideoFrame);

            if (state==STATE_SEARCH)
            {
                if (pFaceRectSeq)
                    if (pFaceRectSeq->total)
                    {
                        CvRect *pFaceRect=(CvRect*)cvGetSeqElem(pFaceRectSeq,0);
                        drawFaceBox(pVideoFrame,pFaceRect);
                        faceCentroid=getCentroid(pFaceRect);

                        setFace(FACE_EXPR_FOUND);
                        enableFaceExprTrack=true;
                        t0=Time::now();

                        state=STATE_TRACK;
                    }
            }
            else if (state==STATE_TRACK)
            {
                if (pFaceRectSeq)
                    if (pFaceRectSeq->total)
                    {
                        CvRect *pFaceRect=getMostLikely(pFaceRectSeq,&faceCentroid);
                        drawFaceBox(pVideoFrame,pFaceRect);
                        faceCentroid=getCentroid(pFaceRect);

                        if (enableFaceExprTrack)
                        {
                            setFace(FACE_EXPR_TRACK);
                            enableFaceExprTrack=false;
                        }
                    }
                    else
                        resetState(true);
                else
                    resetState(true);
            }

            portImgOut->write();
        }

        // send centroid and command arm
        if (centroidUpdate)
        {
            Bottle cmd, reply;
            cmd.addString("get");
            cmd.addString("3dpoint");
            cmd.addString(eye.c_str());
            cmd.addDouble(faceCentroid.x);
            cmd.addDouble(faceCentroid.y);
            cmd.addDouble(eyeDist);

            if (portGet3D->write(cmd,reply))
            {
                fp[0]=reply.get(0).asDouble();
                fp[1]=reply.get(1).asDouble();
                fp[2]=reply.get(2).asDouble();

                portGazeAt->prepare()=fp;
                portGazeAt->write();

                armTrack();
            }

            centroidUpdate=0;
        }

        faceExprHandling();

        timer=Time::now()-t0;
    }

    void resetState(const bool dumpLost=false)
    {
        if (dumpLost)
        {
            armRest();    
            cout << "face lost" << endl;
        }

        setFace(FACE_EXPR_SEARCH);

        timerThres=SAT(PICKTIMER(GAZE_TIMER_MAX),GAZE_TIMER_MIN,GAZE_TIMER_MAX);
        t0=Time::now();
        centroidUpdate=0;
        state=STATE_SEARCH;        
    }

    void armTrack()
    {
        if (ARMISTRACKING(armCmdState))
        {
            Vector fpSat=fp;
            if (fpSat[0]>REACH_X_MAX)
                fpSat[0]=REACH_X_MAX;

            fpSat[2]+=REACH_OFFS_Z;

            portReachPoint->prepare()=fpSat;
            portReachPoint->write();
        }
        else
            armCmdState++;
    }

    void armRest()
    {
        Vector &x=portReachPoint->prepare();
        x.resize(3);

        x[0]=REACH_REST_X;
        x[1]=REACH_REST_Y;
        x[2]=REACH_REST_Z;

        if (arm=="left")
            x[1]=-x[1];

        portReachPoint->write();

        armCmdState=0;
    }

    void gazeRest()
    {
        fp[0]=GAZE_REST_X;
        fp[1]=GAZE_REST_Y;
        fp[2]=GAZE_REST_Z;

        portGazeAt->prepare()=fp;
        portGazeAt->write();
    }

    CvPoint getCentroid(CvRect *pRect, const bool centroidUpdateFlag=true)
    {
        if (centroidUpdateFlag)
            centroidUpdate++;

        return cvPoint(pRect->x+pRect->width/2,
                       pRect->y+pRect->height/2);
    }

    CvRect *getMostLikely(CvSeq *pSeq, CvPoint *pCentroid)
    {
        unsigned int minDist=0xffffffff;
        CvRect *pRet=NULL;

        for (int i=0; i<pSeq->total; i++)
        {
            CvRect *pIterRect=(CvRect*)cvGetSeqElem(pSeq,i);
            CvPoint centroid=getCentroid(pIterRect,false);

            unsigned int iterDist=(centroid.x-pCentroid->x)*(centroid.x-pCentroid->x)+
                                  (centroid.y-pCentroid->y)*(centroid.y-pCentroid->y);

            if (iterDist<minDist)
            {
                minDist=iterDist;
                pRet=pIterRect;
            }
        }

        return pRet;
    }

    void drawFaceBox(IplImage *pImg, CvRect *pRect)
    {
        cvRectangle(pImg,
                    cvPoint(pRect->x,pRect->y),
                    cvPoint(pRect->x+pRect->width,pRect->y+pRect->height),
                    CV_RGB(0,0,255),1,CV_AA,0);
    }

    void setFace(const string &state)
    {
        if (timer>=FACE_EXPR_TIMEOUT)
            faceExprCmd(state);
        else
        {    
            queuedFaceExprState=state;
            queuedFaceExprFlag=true;
        }

        t0=Time::now();
    }

    void faceExprHandling()
    {
        if (queuedFaceExprFlag && timer>=FACE_EXPR_TIMEOUT)
        {    
            faceExprCmd(queuedFaceExprState);
            queuedFaceExprFlag=false;
        }
    }

    void faceExprCmd(const string &state)
    {
        Bottle cmd, reply;

        cmd.addVocab(Vocab::encode("set"));
        cmd.addVocab(Vocab::encode("all"));
        cmd.addVocab(Vocab::encode(state.c_str()));

        portSetFace->write(cmd,reply);
    }

    virtual void threadRelease()
    {        
        armRest();
        gazeRest();

        portImgIn->interrupt();
        portImgOut->interrupt();
        portReachPoint->interrupt();
        portGazeAt->interrupt();
        portGet3D->interrupt();
        portTopDown->interrupt();
        portSetFace->interrupt();

        portImgIn->close();
        portImgOut->close();
        portReachPoint->close();
        portGazeAt->close();
        portGet3D->close();
        portTopDown->close();
        portSetFace->close();

        delete portImgIn;
        delete portImgOut;
        delete portReachPoint;
        delete portGazeAt;
        delete portGet3D;
        delete portTopDown;
        delete portSetFace;
    }
};


class trackModule: public RFModule
{
protected:
    trackThread *thr;

public:
    trackModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new trackThread(rf);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("faceTrackerDemo/conf");
    rf.setDefault("descriptor","haarcascade_frontalface_default.xml");
    rf.configure("ICUB_ROOT",argc,argv);

    trackModule mod;

    return mod.runModule(rf);
}


