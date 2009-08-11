
#include <cv.h>
#include <highgui.h>

#include <ace/OS.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "facedet.h"
                              
#define STATE_SEARCH_L          0
#define STATE_SEARCH_R          1
#define STATE_TRACK             2
                               
#define GAZE_X_MIN             -0.50
#define GAZE_X_MAX             -0.20
#define GAZE_Y_MIN             -0.40
#define GAZE_Y_MAX              0.40
#define GAZE_Z_MIN              0.45
#define GAZE_Z_MAX              0.50
                               
#define GAZE_TIMER_MAX          1.20
#define GAZE_TIMER_MIN          0.90
#define GAZE_DELTA              0.2

#define PICKTIMER(x)            (((double)rand()/(double)RAND_MAX)*(x))
#define PICKRAND(x)             ((2.0*((double)rand()/(double)RAND_MAX)-1.0)*(x))
#define SAT(x,_min,_max)        ((x)>(_max)?(_max):((x)<(_min)?(_min):(x)))
#define ARMISTRACKING(state)    ((state)>=5)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class trackThread : public RateThread
{
protected:
    int state;
    string name;
    unsigned int period;
    double timer, timerThres;

    BufferedPort<ImageOf<PixelBgr> > *portImgInL,   *portImgInR;
    BufferedPort<ImageOf<PixelBgr> > *portImgOutL,  *portImgOutR;
    BufferedPort<Vector>             *portFixPoint, *portReachPoint;
    BufferedPort<Vector>             *portPixel,    *portGazeAt;

    ResourceFinder &rf;

    faceDetector fdL, fdR;

    CvPoint faceCentroidL;
    CvPoint faceCentroidR;

    int centroidUpdate;
    int armCmdState;

    Vector fp;

public:
    trackThread(const string &_name, ResourceFinder &_rf, const unsigned int _period) : 
                RateThread(_period), name(_name), rf(_rf), period(_period) { }

    virtual bool threadInit()
    {
        portImgInL=new BufferedPort<ImageOf<PixelBgr> >;
        string portInNameL=name+"/left:i";
        portImgInL->open(portInNameL.c_str());

        portImgInR=new BufferedPort<ImageOf<PixelBgr> >;
        string portInNameR=name+"/right:i";
        portImgInR->open(portInNameR.c_str());

        portImgOutL=new BufferedPort<ImageOf<PixelBgr> >;
        string portOutNameL=name+"/left:o";
        portImgOutL->open(portOutNameL.c_str());

        portImgOutR=new BufferedPort<ImageOf<PixelBgr> >;
        string portOutNameR=name+"/right:o";
        portImgOutR->open(portOutNameR.c_str());

        portFixPoint=new BufferedPort<Vector>;
        string portFixPointName=name+"/lookingat:i";
        portFixPoint->open(portFixPointName.c_str());

        portReachPoint=new BufferedPort<Vector>;
        string portReachPointName=name+"/reach:o";
        portReachPoint->open(portReachPointName.c_str());

        portPixel=new BufferedPort<Vector>;
        string portPixelName=name+"/pixel:o";
        portPixel->open(portPixelName.c_str());

        portGazeAt=new BufferedPort<Vector>;
        string portGazeAtName=name+"/gazeat:o";
        portGazeAt->open(portGazeAtName.c_str());

        ConstString descPath=rf.findFile("descriptor");

        if (!fdL.init(descPath.c_str()) || !fdR.init(descPath.c_str()))
        {
            cout << "Cannot load descriptor" << endl;        
            return false;
        }

        srand((unsigned int)time(NULL));

        resetState();
        armCmdState=0;

        fp.resize(3);
        fp[0]=(GAZE_X_MIN+GAZE_X_MAX)/2.0;
        fp[1]=(GAZE_Y_MIN+GAZE_Y_MAX)/2.0;
        fp[2]=(GAZE_Z_MIN+GAZE_Z_MAX)/2.0;

        return true;
    }

    virtual void run()
    {
        double t0=Time::now();

        // get inputs
        ImageOf<PixelBgr> *pImgInL=portImgInL->read(false);
        ImageOf<PixelBgr> *pImgInR=portImgInR->read(false);

        if (Vector *x=portFixPoint->read(false))
            fp=*x;

        // seek for faces
        if (state==STATE_SEARCH_L && timer>timerThres)
        {            
            fp[0]=SAT(fp[0]+PICKRAND(GAZE_DELTA),GAZE_X_MIN,GAZE_X_MAX);
            fp[1]=SAT(fp[1]+PICKRAND(GAZE_DELTA),GAZE_Y_MIN,GAZE_Y_MAX);
            fp[2]=SAT(fp[2]+PICKRAND(GAZE_DELTA),GAZE_Z_MIN,GAZE_Z_MAX);

            Vector &x=portGazeAt->prepare();
            x=fp;
            
            portGazeAt->write();
            timerThres=SAT(PICKTIMER(GAZE_TIMER_MAX),GAZE_TIMER_MIN,GAZE_TIMER_MAX);
            timer=0;

            cout << "Gazing at: " << fp.toString() << endl;
        }

        // process left cam image
        if (pImgInL)
        {            
            ImageOf<PixelBgr> &imgOutL=portImgOutL->prepare();
            imgOutL=*pImgInL;
            IplImage *pVideoFrameL=(IplImage*)imgOutL.getIplImage();

            CvSeq *pFaceRectSeqL=fdL.detect(pVideoFrameL);

            if (state==STATE_SEARCH_L)
            {
                if (pFaceRectSeqL)
                    if (pFaceRectSeqL->total)
                    {
                        CvRect *pFaceRectL=(CvRect*)cvGetSeqElem(pFaceRectSeqL,0);
                        drawFaceBox(pVideoFrameL,pFaceRectL);
                        faceCentroidL=getCentroid(pFaceRectL);
                        state=STATE_SEARCH_R;
                    }
            }
            else if (state==STATE_TRACK)
            {
                if (pFaceRectSeqL)
                    if (pFaceRectSeqL->total)
                    {
                        CvRect *pFaceRectL=getMostLikely(pFaceRectSeqL,&faceCentroidL);
                        drawFaceBox(pVideoFrameL,pFaceRectL);
                        faceCentroidL=getCentroid(pFaceRectL);
                    }
                    else
                        resetState(true);
                else
                    resetState(true);
            }

            portImgOutL->write();
        }

        // process right cam image
        if (pImgInR)
        {            
            ImageOf<PixelBgr> &imgOutR=portImgOutR->prepare();
            imgOutR=*pImgInR;
            IplImage *pVideoFrameR=(IplImage*)imgOutR.getIplImage();

            CvSeq *pFaceRectSeqR=fdR.detect(pVideoFrameR);

            if (state==STATE_SEARCH_R)
            {
                if (pFaceRectSeqR)
                    if (pFaceRectSeqR->total)
                    {
                        CvRect *pFaceRectR=getMostLikely(pFaceRectSeqR,&faceCentroidL);
                        drawFaceBox(pVideoFrameR,pFaceRectR);
                        faceCentroidR=getCentroid(pFaceRectR);
                        state=STATE_TRACK;

                        cout << "face detected" << endl;
                    }
                    else
                        resetState();
                else
                    resetState();
            }
            else if (state==STATE_TRACK)
            {
                if (pFaceRectSeqR)
                    if (pFaceRectSeqR->total)
                    {
                        CvRect *pFaceRectR=getMostLikely(pFaceRectSeqR,&faceCentroidR);
                        drawFaceBox(pVideoFrameR,pFaceRectR);
                        faceCentroidR=getCentroid(pFaceRectR);
                    }
                    else
                        resetState(true);
                else
                    resetState(true);
            }

            portImgOutR->write();
        }

        // send centroids and command arm
        if (centroidUpdate>1)
        {
            Vector &faceCentroids=portPixel->prepare();
            faceCentroids.resize(4);

            faceCentroids[0]=faceCentroidL.x;
            faceCentroids[1]=faceCentroidL.y;
            faceCentroids[2]=faceCentroidR.x;
            faceCentroids[3]=faceCentroidR.y;

            portPixel->write();

            armTrack();

            centroidUpdate=0;
        }

        double t1=Time::now();
        timer+=t1-t0;
    }

    void resetState(const bool dumpLost=false)
    {
        if (dumpLost)
        {
            armRest();
            cout << "face lost" << endl;
        }

        timerThres=SAT(PICKTIMER(GAZE_TIMER_MAX),GAZE_TIMER_MIN,GAZE_TIMER_MAX);
        timer=0;
        centroidUpdate=0;        
        state=STATE_SEARCH_L;
    }

    void armTrack()
    {
        if (ARMISTRACKING(armCmdState))
        {
            Vector &x=portReachPoint->prepare();
            x=fp;
            portReachPoint->write();
        }
        else
            armCmdState++;
    }

    void armRest()
    {
//      if (ARMISTRACKING(armCmdState))
//      {
//          Vector &x=portReachPoint->prepare();
//          x[0]=-0.27;
//          x[1]=+0.00;
//          x[2]=+0.10;
//
//          portReachPoint->write();
//      }

        armCmdState=0;
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

    virtual void threadRelease()
    {        
        portImgInL->interrupt();
        portImgInR->interrupt();
        portImgOutL->interrupt();
        portImgOutR->interrupt();
        portFixPoint->interrupt();
        portReachPoint->interrupt();
        portPixel->interrupt();
        portGazeAt->interrupt();

        portImgInL->close();
        portImgInR->close();
        portImgOutL->close();
        portImgOutR->close();
        portFixPoint->close();
        portReachPoint->close();
        portPixel->close();
        portGazeAt->close();

        delete portImgInL;
        delete portImgInR;
        delete portImgOutL;
        delete portImgOutR;
        delete portFixPoint;
        delete portReachPoint;
        delete portPixel;
        delete portGazeAt;
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

        thr=new trackThread("/faceTracker",rf,20);
        thr->start();

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


