/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/** 
\defgroup faceTracker faceTracker 
 
@ingroup icub_module  
 
A face tracker based on the Haar detector.

Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module uses the OpenCV Haar detector in order to find a 
face in front of the robot. Once detected it controls the head
motion for tracking the face and also to move the hand to point 
at the gazing location. 
 
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
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified \e icub is assumed.
 
--period \e T 
- The integer \e T specifies the module period in [ms].
 
--eye \e sel 
- The parameter \e sel selects the used eye: it can be "left" or
  "right".
 
--arm \e sel 
- The parameter \e sel selects the arm used for pointing: it can
  be "left", "right" or even "none".
 
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
/icub/cartesianController/left_arm/* 
/icub/cartesianController/right_arm/* 
/iKinGazeCtrl/* 
/icub/face/emotions/in 

\section portsc_sec Ports Created 
 
- \e /faceTracker/img:i receives the image.
 
- \e /faceTracker/img:o sends out the image acquired from the 
  left camera with a superimposed rectangle which identifies the
  detected face (to be connected to a viewer).
 
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
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Rand.h>

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
                               
#define GAZE_REST_AZI               0.0
#define GAZE_REST_ELE               0.0
#define GAZE_REST_VER               5.0

#define GAZE_AZI_MIN                -20.0
#define GAZE_AZI_MAX                20.0
#define GAZE_ELE_MIN                0.0
#define GAZE_ELE_MAX                20.0
#define GAZE_VER_MIN                2.0
#define GAZE_VER_MAX                20.0
                               
#define GAZE_TIMER_MAX              1.50
#define GAZE_TIMER_MIN              0.80

#define REACH_X_MAX                 -0.1
#define REACH_REST_X                -0.28
#define REACH_REST_Y                0.1     // given for the right arm
#define REACH_REST_Z                0.1
#define REACH_OFFS_Z                -0.1

#define FACE_EXPR_SEARCH            "shy"
#define FACE_EXPR_FOUND             "hap"
#define FACE_EXPR_TRACK             "hap"
#define FACE_EXPR_TOPDOWN           "shy"
#define FACE_EXPR_TIMEOUT           2.0

#define SAT(x,_min,_max)            ((x)>(_max)?(_max):((x)<(_min)?(_min):(x)))
#define ARMISTRACKING(state)        ((state)>=1)

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class trackThread : public RateThread
{
protected:
    int state;
    string eye;
    string arm;
    double eyeDist;
    double holdoff;
    double timer, timerThres, t0;

    BufferedPort<ImageOf<PixelBgr> > portImgIn, portImgOut;
    BufferedPort<Bottle>             portTopDown;
    Port                             portSetFace;

    PolyDriver clientGaze, clientArm;
    IGazeControl      *igaze;
    ICartesianControl *iarm;

    ResourceFinder &rf;

    faceDetector fd;
    CvPoint faceCentroid;

    int centroidUpdate;
    int armCmdState;

    int startup_gazeContext_id;
    int startup_armContext_id;

    bool enableFaceExprTrack;
    bool queuedFaceExprFlag;
    string queuedFaceExprState;

public:
    trackThread(ResourceFinder &_rf) : RateThread(50), rf(_rf) { }

    virtual bool threadInit()
    {
        string name=rf.check("name",Value("faceTracker")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        int period=rf.check("period",Value(50)).asInt();
        eye=rf.check("eye",Value("left")).asString().c_str();
        arm=rf.check("arm",Value("right")).asString().c_str();
        eyeDist=rf.check("eyeDist",Value(1.0)).asDouble();
        holdoff=rf.check("holdoff",Value(3.0)).asDouble();

        Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local",("/"+name+"/gaze_client").c_str());

        if (!clientGaze.open(optGaze))
            return false;

        clientGaze.view(igaze);
        igaze->storeContext(&startup_gazeContext_id);
        igaze->blockNeckRoll(0.0);

        if (arm!="none")
        {
            Property optArm("(device cartesiancontrollerclient)");
            optArm.put("remote",("/"+robot+"/cartesianController/"+arm+"_arm").c_str());
            optArm.put("local",("/"+name+"/arm_client").c_str());
    
            if (!clientArm.open(optArm))
                return false;
    
            clientArm.view(iarm);
            iarm->storeContext(&startup_armContext_id);
        }

        portImgIn.open(("/"+name+"/img:i").c_str());
        portImgOut.open(("/"+name+"/img:o").c_str());
        portTopDown.open(("/"+name+"/topdown:i").c_str());
        portSetFace.open(("/"+name+"/setFace:rpc").c_str());

        if (!fd.init(rf.findFile("descriptor").c_str()))
        {
            fprintf(stdout,"Cannot load descriptor!\n");
            return false;
        }

        Rand::init();

        resetState();
        armCmdState=0;
        queuedFaceExprFlag=false;

        setRate(period);
        cvSetNumThreads(1);

        t0=Time::now();

        return true;
    }

    virtual void run()
    {
        // top-down input: handled at the highest priority
        if (Bottle *topDownInput=portTopDown.read(false))
        {
            Vector px(2);
            px[0]=topDownInput->get(0).asInt();
            px[1]=topDownInput->get(1).asInt();
            igaze->lookAtMonoPixel(eye=="left"?0:1,px,eyeDist);

            armRest();
            setFace(FACE_EXPR_TOPDOWN);

            state=STATE_TOPDOWN;
            centroidUpdate=0;
            t0=Time::now();

            fprintf(stdout,"Top-Down gazing at: (%d,%d) pixel\n",(int)px[0],(int)px[1]);
        }

        if (state==STATE_TOPDOWN)
            if (timer>=holdoff)
                resetState();

        // get inputs
        ImageOf<PixelBgr> *pImgIn=portImgIn.read(false);        

        // seek for faces
        if ((state==STATE_SEARCH) && (timer>timerThres))
        {            
            Vector ang(3);
            ang[0]=Rand::scalar(GAZE_AZI_MIN,GAZE_AZI_MAX);
            ang[1]=Rand::scalar(GAZE_ELE_MIN,GAZE_ELE_MAX);
            ang[2]=Rand::scalar(GAZE_VER_MIN,GAZE_VER_MAX);

            igaze->lookAtAbsAngles(ang);

            timerThres=Rand::scalar(GAZE_TIMER_MIN,GAZE_TIMER_MAX);
            t0=Time::now();

            fprintf(stdout,"Gazing at: (%.1f,%.1f,%.1f) deg\n",ang[0],ang[1],ang[2]);
        }

        // process camera image
        if (pImgIn)
        {            
            ImageOf<PixelBgr> &imgOut=portImgOut.prepare();
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

            portImgOut.write();
        }

        // send centroid and command arm
        if (centroidUpdate)
        {
            Vector px(2);
            px[0]=faceCentroid.x;
            px[1]=faceCentroid.y;
            igaze->lookAtMonoPixel(eye=="left"?0:1,px,eyeDist);

            armTrack();

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
            fprintf(stdout,"Face lost!\n");
        }

        setFace(FACE_EXPR_SEARCH);

        timerThres=Rand::scalar(GAZE_TIMER_MIN,GAZE_TIMER_MAX);
        t0=Time::now();
        centroidUpdate=0;
        state=STATE_SEARCH;        
    }

    void armTrack()
    {
        if (arm!="none")
        {
            if (ARMISTRACKING(armCmdState))
            {
                Vector fp;
                igaze->getFixationPoint(fp);
    
                if (fp[0]>REACH_X_MAX)
                    fp[0]=REACH_X_MAX;
    
                fp[2]+=REACH_OFFS_Z;
    
                iarm->goToPosition(fp);
    
                fprintf(stdout,"Reaching for: (%.1f,%.1f,%.1f) m\n",fp[0],fp[1],fp[2]);
            }
            else
                armCmdState++;
        }
    }

    void armRest()
    {
        if (arm!="none")
        {
            Vector x(3);
            x[0]=REACH_REST_X;
            x[1]=REACH_REST_Y;
            x[2]=REACH_REST_Z;
    
            if (arm=="left")
                x[1]=-x[1];
    
            iarm->goToPosition(x);
    
            armCmdState=0;
        }
    }

    void gazeRest()
    {
        Vector ang(3);
        ang[0]=GAZE_REST_AZI;
        ang[1]=GAZE_REST_ELE;
        ang[2]=GAZE_REST_VER;

        igaze->lookAtAbsAngles(ang);
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

        portSetFace.write(cmd,reply);
    }

    virtual void threadRelease()
    {        
        armRest();
        gazeRest();

        igaze->restoreContext(startup_gazeContext_id);
        clientGaze.close();

        if (arm!="none")
        {
            iarm->restoreContext(startup_armContext_id);
            clientArm.close();
        }

        portImgIn.interrupt();
        portImgOut.interrupt();
        portTopDown.interrupt();
        portSetFace.interrupt();

        portImgIn.close();
        portImgOut.close();
        portTopDown.close();
        portSetFace.close();
    }
};


class trackModule: public RFModule
{
protected:
    trackThread *thr;

public:
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
    YARP_REGISTER_DEVICES(icubmod)

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


