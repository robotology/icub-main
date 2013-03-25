/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Carlo Ciliberto, Ugo Pattacini
 * email:   carlo.ciliberto@iit.it, ugo.pattacini@iit.it
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

#include "main.h"
#include <cv.h>

#ifdef CV_MAJOR_VERSION
    // check if OpenCV supports OpenMP multi-threading
    #if (CV_MAJOR_VERSION>0) && (CV_MAJOR_VERSION<3)
        #define _MOTIONCUT_MULTITHREADING_OPENMP
    #endif
    #if (CV_MAJOR_VERSION==2) && (CV_MINOR_VERSION>0)
        #undef _MOTIONCUT_MULTITHREADING_OPENMP
    #endif
#endif

// in BGR format
#define NODE_OFF    cvScalar(0,0,255)
#define NODE_ON     cvScalar(0,255,0)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/************************************************************************/
class Blob
{
public:
    CvPoint centroid;
    int     size;

    /************************************************************************/
    Blob()
    {
        centroid.x=0;
        centroid.y=0;
        size=0;
    }
};


/************************************************************************/
class ProcessThread : public Thread
{
protected:
    ResourceFinder &rf;

    string name;
    bool firstConsistencyCheck;
    double coverXratio;
    double coverYratio;
    int nodesStep;
    int winSize;
    double recogThres;
    double recogThresAbs;
    int adjNodesThres;
    int blobMinSizeThres;
    int framesPersistence;
    int cropRadius;
    bool fixedRadius;
    bool verbosity;
    bool inhibition;
    int nodesNum;
    int nodesX;
    int nodesY;    

#ifdef _MOTIONCUT_MULTITHREADING_OPENMP
    int numThreads;
#endif

    ImageOf<PixelMono>  imgMonoIn;
    ImageOf<PixelMono>  imgMonoPrev;
    ImageOf<PixelFloat> imgPyrPrev;
    ImageOf<PixelFloat> imgPyrCurr;

    CvPoint2D32f        *nodesPrev;
    CvPoint2D32f        *nodesCurr;
    int                 *nodesPersistence;
    char                *featuresFound;
    float               *featuresErrors;

    set<int>             activeNodesIndexSet;
    deque<Blob>          blobSortedList;

    BufferedPort<ImageOf<PixelBgr> >  inPort;
    BufferedPort<ImageOf<PixelBgr> >  outPort;
    BufferedPort<ImageOf<PixelMono> > optPort;
    Port nodesPort;
    Port blobsPort;
    Port cropPort;

    /************************************************************************/
    void disposeMem()
    {
        if (nodesPrev!=NULL)
            delete nodesPrev;

        if (nodesCurr!=NULL)
            delete nodesCurr;

        if (nodesPersistence!=NULL)
            delete nodesPersistence;

        if (featuresFound!=NULL)
            delete featuresFound;

        if (featuresErrors!=NULL)
            delete featuresErrors;
    }

#ifdef _MOTIONCUT_MULTITHREADING_OPENMP
    /************************************************************************/
    int setNumThreads(const int n)
    {
        if (n>=0)
        {
            cvSetNumThreads(n);
            return cvGetNumThreads();
        }
        else
        {
            cvSetNumThreads(0);
            int m=cvGetNumThreads()+n;

            if (m>0)
                cvSetNumThreads(m);
            else
                cvSetNumThreads(1);

            return cvGetNumThreads();
        }
    }
#endif

public:
    /************************************************************************/
    ProcessThread(ResourceFinder &_rf) : rf(_rf) { }

    /************************************************************************/
    bool threadInit()
    {
        name=rf.check("name",Value("motionCUT")).asString().c_str();
        coverXratio=rf.check("coverXratio",Value(0.75)).asDouble();
        coverYratio=rf.check("coverYratio",Value(0.75)).asDouble();
        nodesStep=rf.check("nodesStep",Value(6)).asInt();
        winSize=rf.check("winSize",Value(15)).asInt();
        recogThres=rf.check("recogThres",Value(0.5)).asDouble();
        adjNodesThres=rf.check("adjNodesThres",Value(4)).asInt();
        blobMinSizeThres=rf.check("blobMinSizeThres",Value(10)).asInt();
        framesPersistence=rf.check("framesPersistence",Value(3)).asInt();
        cropRadius=rf.check("cropRadius",Value(40)).asInt();
        fixedRadius=rf.check("fixedRadius",Value("true")).asString()=="true";
        verbosity=rf.check("verbosity");
        inhibition=false;

        recogThresAbs=recogThres*((256*winSize*winSize)/100.0);

        // thresholding
        coverXratio=std::min(coverXratio,1.0);
        coverYratio=std::min(coverYratio,1.0);

        // if the OpenCV version supports OpenMP multi-threading,
        // set the maximum number of threads available to OpenCV
    #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
        numThreads=setNumThreads(rf.check("numThreads",Value(-1)).asInt());
    #endif

        nodesPrev=NULL;
        nodesCurr=NULL;
        nodesPersistence=NULL;
        featuresFound=NULL;
        featuresErrors=NULL;
        
        inPort.open(("/"+name+"/img:i").c_str());
        outPort.open(("/"+name+"/img:o").c_str());
        optPort.open(("/"+name+"/opt:o").c_str());
        nodesPort.open(("/"+name+"/nodes:o").c_str());
        blobsPort.open(("/"+name+"/blobs:o").c_str());
        cropPort.open(("/"+name+"/crop:o").c_str());

        firstConsistencyCheck=true;

        return true;
    }

    /************************************************************************/
    void afterStart(bool s)
    {
        if (s)
        {
            fprintf(stdout,"Process started successfully\n");
            fprintf(stdout,"\n");
            fprintf(stdout,"Using ...\n");
            fprintf(stdout,"name               = %s\n",name.c_str());
            fprintf(stdout,"coverXratio        = %g\n",coverXratio);
            fprintf(stdout,"coverYratio        = %g\n",coverYratio);
            fprintf(stdout,"nodesStep          = %d\n",nodesStep);
            fprintf(stdout,"winSize            = %d\n",winSize);
            fprintf(stdout,"recogThres         = %g\n",recogThres);
            fprintf(stdout,"recogThresAbs      = %g\n",recogThresAbs);
            fprintf(stdout,"adjNodesThres      = %d\n",adjNodesThres);
            fprintf(stdout,"blobMinSizeThres   = %d\n",blobMinSizeThres);
            fprintf(stdout,"framesPersistence  = %d\n",framesPersistence);
            if (fixedRadius)
                fprintf(stdout,"cropRadius (fixed) = %d\n",cropRadius);
            else               
                fprintf(stdout,"cropRadius (var)   = %d\n",cropRadius);
            
        #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
            fprintf(stdout,"numThreads        = %d\n",numThreads);
        #else
            fprintf(stdout,"numThreads        = OpenCV version does not support OpenMP multi-threading\n");
        #endif
            
            fprintf(stdout,"verbosity         = %s\n",verbosity?"on":"off");            
            fprintf(stdout,"\n");
        }
        else
            fprintf(stdout,"Process did not start\n");
    }

    /************************************************************************/
    void run()
    {
        double latch_t, dt0, dt1, dt2;

        while (!isStopping())
        {
            // acquire new image
            ImageOf<PixelBgr> *pImgBgrIn=inPort.read(true);

            // get the envelope from the image
            Stamp stamp;
            inPort.getEnvelope(stamp);

            if (isStopping() || (pImgBgrIn==NULL))
                break;

            double t0=Time::now();
             
            // consistency check
            if (firstConsistencyCheck || (pImgBgrIn->width()!=imgMonoIn.width()) ||
                (pImgBgrIn->height()!=imgMonoIn.height()))
            {
                firstConsistencyCheck=false;

                imgMonoIn.resize(*pImgBgrIn);
                imgMonoPrev.resize(*pImgBgrIn);

                imgPyrPrev.resize(pImgBgrIn->width()+8,pImgBgrIn->height()/3);
                imgPyrCurr.resize(pImgBgrIn->width()+8,pImgBgrIn->height()/3);

                // dispose previously allocated memory
                disposeMem();
                
                int min_x=(int)(((1.0-coverXratio)/2.0)*imgMonoIn.width());
                int min_y=(int)(((1.0-coverYratio)/2.0)*imgMonoIn.height());

                nodesX=(imgMonoIn.width()-2*min_x)/nodesStep+1;
                nodesY=(imgMonoIn.height()-2*min_y)/nodesStep+1;

                nodesNum=nodesX*nodesY;

                nodesPrev=new CvPoint2D32f[nodesNum];
                nodesCurr=new CvPoint2D32f[nodesNum];
                nodesPersistence=new int[nodesNum];

                featuresFound=new char[nodesNum];
                featuresErrors=new float[nodesNum];

                memset(nodesPersistence,0,nodesNum*sizeof(int));
                
                // populate grid
                int cnt=0;
                for (int y=min_y; y<=(imgMonoIn.height()-min_y); y+=nodesStep)
                    for (int x=min_x; x<=(imgMonoIn.width()-min_x); x+=nodesStep)
                        nodesPrev[cnt++]=cvPoint2D32f(x,y);

                // convert to gray-scale
                cvCvtColor(pImgBgrIn->getIplImage(),imgMonoPrev.getIplImage(),CV_BGR2GRAY);

                if (verbosity)
                {
                    // log message
                    fprintf(stdout,"Detected image of size %dx%d;\nusing %dx%d=%d nodes;\npopulated %d nodes\n",
                            imgMonoIn.width(),imgMonoIn.height(),nodesX,nodesY,nodesNum,cnt);
                }

                // skip to the next cycle
                continue;
            }

            // convert the input image to gray-scale
            cvCvtColor(pImgBgrIn->getIplImage(),imgMonoIn.getIplImage(),CV_BGR2GRAY);

            // copy input image into output image
            ImageOf<PixelBgr> &imgBgrOut=outPort.prepare();
            imgBgrOut=*pImgBgrIn;

            // get optFlow image
            ImageOf<PixelMono> &imgMonoOpt=optPort.prepare();
            imgMonoOpt.resize(imgBgrOut);
            imgMonoOpt.zero();

            // declare output bottles
            Bottle nodesBottle;
            Bottle blobsBottle;

            Bottle &nodesStepBottle=nodesBottle.addList();
            nodesStepBottle.addString("nodesStep");
            nodesStepBottle.addInt(nodesStep);

            // purge the content of variables
            activeNodesIndexSet.clear();
            blobSortedList.clear();

            // compute optical flow
            latch_t=Time::now();
            cvCalcOpticalFlowPyrLK(imgMonoPrev.getIplImage(),imgMonoIn.getIplImage(),
                                   imgPyrPrev.getIplImage(),imgPyrCurr.getIplImage(),
                                   nodesPrev,nodesCurr,nodesNum,
                                   cvSize(winSize,winSize),5,featuresFound,featuresErrors,
                                   cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.3),0);
            dt0=Time::now()-latch_t;

            // assign status to the grid nodes
            latch_t=Time::now();
            for (int i=0; i<nodesNum; i++)
            {
                bool persistentNode=false;

                CvPoint node=cvPoint((int)nodesPrev[i].x,(int)nodesPrev[i].y);
                
                // handle the node persistence
                if (!inhibition && (nodesPersistence[i]!=0))
                {
                    cvCircle(imgBgrOut.getIplImage(),node,1,NODE_ON,2);
                    cvCircle(imgMonoOpt.getIplImage(),node,1,cvScalar(255),2);

                    Bottle &nodeBottle=nodesBottle.addList();
                    nodeBottle.addInt((int)nodesPrev[i].x);
                    nodeBottle.addInt((int)nodesPrev[i].y);

                    // update the active nodes set
                    activeNodesIndexSet.insert(i);

                    nodesPersistence[i]--;

                    persistentNode=true;
                }
                else
                    cvCircle(imgBgrOut.getIplImage(),node,1,NODE_OFF,1);

                // do not consider the border nodes and skip if inhibition is on
                int row=i%nodesX;
                bool skip=inhibition || (i<nodesX) || (i>=(nodesNum-nodesX)) || (row==0) || (row==(nodesX-1));

                if (!skip && (featuresFound[i]!=0) && (featuresErrors[i]>recogThresAbs))
                {
                    // count the neighbour nodes that are ON
                    // start from -1 to avoid counting the current node
                    int cntAdjNodesOn=-1;

                    // scroll per lines
                    for (int j=i-nodesX; j<=(i+nodesX); j+=nodesX)
                        for (int k=j-1; k<=(j+1); k++)
                            cntAdjNodesOn+=(int)((featuresFound[k]!=0)&&(featuresErrors[k]>recogThresAbs));

                    // highlight independent moving node if over threhold
                    if (cntAdjNodesOn>=adjNodesThres)
                    {
                        // init the node persistence timeout
                        nodesPersistence[i]=framesPersistence;

                        // update only if the node was not persistent
                        if (!persistentNode)
                        {
                            cvCircle(imgBgrOut.getIplImage(),node,1,NODE_ON,2);
                            cvCircle(imgMonoOpt.getIplImage(),node,1,cvScalar(255),2);

                            Bottle &nodeBottle=nodesBottle.addList();
                            nodeBottle.addInt((int)nodesPrev[i].x);
                            nodeBottle.addInt((int)nodesPrev[i].y);

                            // update the active nodes set
                            activeNodesIndexSet.insert(i);
                        }
                    }
                }
            }
            dt1=Time::now()-latch_t;

            latch_t=Time::now();
            findBlobs();

            // prepare the blobs output list and draw their
            // centroids location
            for (int i=0; i<(int)blobSortedList.size(); i++)
            {
                Blob &blob=blobSortedList[i];
                int blueLev=255-((100*i)%255);
                int redLev=(100*i)%255;

                CvPoint centroid=cvPoint(blob.centroid.x,blob.centroid.y);

                Bottle &blobBottle=blobsBottle.addList();
                blobBottle.addInt(centroid.x);
                blobBottle.addInt(centroid.y);
                blobBottle.addInt(blob.size);

                cvCircle(imgBgrOut.getIplImage(),centroid,4,cvScalar(blueLev,0,redLev),3);
            }
            dt2=Time::now()-latch_t;

            // send out images, propagating the time-stamp
            if (outPort.getOutputCount()>0)
            {
                outPort.setEnvelope(stamp);
                outPort.write();
            }
            else
                outPort.unprepare();

            if (optPort.getOutputCount()>0)
            {
                optPort.setEnvelope(stamp);
                optPort.write();
            }
            else
                optPort.unprepare();

            // send out data bottles, propagating the time-stamp
            if ((nodesPort.getOutputCount()>0) && (nodesBottle.size()>1))
            {
                nodesPort.setEnvelope(stamp);
                nodesPort.write(nodesBottle);
            }

            if ((blobsPort.getOutputCount()>0) && (blobsBottle.size()>0))
            {
                blobsPort.setEnvelope(stamp);
                blobsPort.write(blobsBottle);
            }
            
            if ((cropPort.getOutputCount()>0) && (blobsBottle.size()>0))
            {
                int x=cvRound(blobsBottle.get(0).asList()->get(0).asDouble());
                int y=cvRound(blobsBottle.get(0).asList()->get(1).asDouble());
                
                int radius=std::min(cropRadius,x);
                radius=std::min(radius,y);
                radius=std::min(radius,pImgBgrIn->width()-x-1);
                radius=std::min(radius,pImgBgrIn->height()-y-1);

                if (fixedRadius && (radius<cropRadius))
                {
                    radius=cropRadius;
                    x=std::min(x,cropRadius);
                    x=std::min(x,pImgBgrIn->width()-cropRadius-1);
                    y=std::min(y,cropRadius);
                    y=std::min(y,pImgBgrIn->height()-cropRadius-1);
                }

                int radius2=radius<<1;

                ImageOf<PixelBgr> cropImg;
                cropImg.resize(radius2,radius2);
                
                cvSetImageROI((IplImage*)pImgBgrIn->getIplImage(),cvRect(x-radius,y-radius,radius2,radius2));
                cvCopy((IplImage*)pImgBgrIn->getIplImage(),(IplImage*)cropImg.getIplImage());
                cvResetImageROI((IplImage*)pImgBgrIn->getIplImage());
                            
                cropPort.setEnvelope(stamp);
                cropPort.write(cropImg);
            }

            // save data for next cycle
            imgMonoPrev=imgMonoIn;
            
            double t1=Time::now();
            if (verbosity)
            {
                // dump statistics
                fprintf(stdout,"cycle timing [ms]: optflow(%g), colorgrid(%g), blobdetection(%g), overall(%g)\n",
                        1000.0*dt0,1000.0*dt1,1000.0*dt2,1000.0*(t1-t0));
            }
        }
    }

    /************************************************************************/
    void onStop()
    {
        inPort.interrupt();
        outPort.interrupt();
        optPort.interrupt();
        nodesPort.interrupt();
        blobsPort.interrupt();
    }

    /************************************************************************/
    void threadRelease()
    {
        disposeMem();

        inPort.close();
        outPort.close();
        optPort.close();
        nodesPort.close();
        blobsPort.close();
        cropPort.close();
    }

    /************************************************************************/
    string getName()
    {
        return name;
    }

    /************************************************************************/
    void findBlobs()
    {
        // iterate until the set is empty
        while (activeNodesIndexSet.size())
        {
            Blob blob;

            // the nodes connected to the current one
            // will be removed from the list            
            floodFill(*(activeNodesIndexSet.begin()),&blob);

            // update centroid
            blob.centroid.x/=blob.size;
            blob.centroid.y/=blob.size;

            // insert iff the blob is big enough
            if (blob.size>blobMinSizeThres)
                insertBlob(blob);
        }
    }

    /************************************************************************/
    void floodFill(const int i, Blob *pBlob)
    {
        set<int>::iterator el=activeNodesIndexSet.find(i);
        if ((el!=activeNodesIndexSet.end()) && (pBlob!=NULL))
        {
            // update blob
            pBlob->centroid.x+=(int)nodesPrev[i].x;
            pBlob->centroid.y+=(int)nodesPrev[i].y;
            pBlob->size++;

            // remove element from the set            
            activeNodesIndexSet.erase(el);

            // perform recursive exploration
            for (int j=i-nodesX; j<=(i+nodesX); j+=nodesX)
                for (int k=j-1; k<=(j+1); k++)
                    if (k!=i)
                        floodFill(k,pBlob);
        }
    }

    /************************************************************************/
    void insertBlob(const Blob &blob)
    {
        // insert the blob keeping the decreasing order of the list wrt the size attribute
        for (deque<Blob>::iterator el=blobSortedList.begin(); el!=blobSortedList.end(); el++)
        {
            if (el->size<blob.size)
            {
                blobSortedList.insert(el,blob);
                return;
            }
        }

        // reaching this point means that 
        // we have to append the blob
        blobSortedList.push_back(blob);
    }

    /************************************************************************/
    bool execReq(const Bottle &req, Bottle &reply)
    {
        if (req.size())
        {
            string cmd=req.get(0).asString().c_str();

            if (cmd=="set")
            {
                if (req.size()<3)
                    return false;

                string subcmd=req.get(1).asString().c_str();

                if (subcmd=="winSize")
                {
                    winSize=req.get(2).asInt();
                    reply.addString("ack");
                }
                else if (subcmd=="recogThres")
                {
                    recogThres=req.get(2).asDouble();
                    recogThresAbs=recogThres*((256*winSize*winSize)/100.0);
                    reply.addString("ack");
                }
                else if (subcmd=="adjNodesThres")
                {
                    adjNodesThres=req.get(2).asInt();
                    reply.addString("ack");
                }
                else if (subcmd=="blobMinSizeThres")
                {
                    blobMinSizeThres=req.get(2).asInt();
                    reply.addString("ack");
                }
                else if (subcmd=="framesPersistence")
                {
                    framesPersistence=req.get(2).asInt();
                    reply.addString("ack");
                }
                else if (subcmd=="numThreads")
                {
                #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
                    numThreads=setNumThreads(req.get(2).asInt());
                    reply.addString("ack");
                #else
                    reply.addString("OpenMP multi-threading not supported");
                #endif
                }
                else if (subcmd=="verbosity")
                {
                    verbosity=req.get(2).asString()=="on";
                    reply.addString("ack");
                }
                else if (subcmd=="inhibition")
                {
                    inhibition=req.get(2).asString()=="on";
                    reply.addString("ack");
                }
                else if (subcmd=="cropRadius")
                {
                    cropRadius=req.get(2).asInt();
                    reply.addString("ack");
                }
                else
                    return false;
            }
            else if (cmd=="get")
            {
                if (req.size()<2)
                    return false;

                string subcmd=req.get(1).asString().c_str();

                if (subcmd=="winSize")
                    reply.addInt(winSize);
                else if (subcmd=="recogThres")
                    reply.addDouble(recogThres);
                else if (subcmd=="adjNodesThres")
                    reply.addInt(adjNodesThres);
                else if (subcmd=="blobMinSizeThres")
                    reply.addInt(blobMinSizeThres);
                else if (subcmd=="framesPersistence")
                    reply.addInt(framesPersistence);
                else if (subcmd=="numThreads")
                #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
                    reply.addInt(numThreads);
                #else
                    reply.addString("OpenMP multi-threading not supported");
                #endif
                else if (subcmd=="verbosity")
                    reply.addString(verbosity?"on":"off");
                else if (subcmd=="inhibition")
                    reply.addString(inhibition?"on":"off");
                else if (subcmd=="cropRadius")
                    reply.addInt(cropRadius);
                else
                    return false;
            }
            else
                return false;

            return true;
        }
        else
            return false;
    }
};


/************************************************************************/
class ProcessModule: public RFModule
{
private:
    ProcessThread *thr;
    Port           rpcPort;

public:
    /************************************************************************/
    ProcessModule() : thr(NULL) { }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new ProcessThread(rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(("/"+thr->getName()+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        if (thr->execReq(command,reply))
            return true;
        else
            return RFModule::respond(command,reply);
    }

    /************************************************************************/
    bool close()
    {
        if (thr!=NULL)
        {
            thr->stop();
            delete thr;
        }

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        fprintf(stdout,"\n");
    #ifdef CV_MAJOR_VERSION
        fprintf(stdout,"This module has been compiled with OpenCV %d.%d\n",CV_MAJOR_VERSION,CV_MINOR_VERSION);
    #else
        fprintf(stdout,"This module has been compiled with an unknown version of OpenCV (probably < 1.0)\n");
    #endif        
        fprintf(stdout,"\n");
        fprintf(stdout,"Available options:\n");
        fprintf(stdout,"\t--name              <string>\n");
        fprintf(stdout,"\t--coverXratio       <double>\n");
        fprintf(stdout,"\t--coverYratio       <double>\n");
        fprintf(stdout,"\t--nodesStep         <int>\n");
        fprintf(stdout,"\t--winSize           <int>\n");
        fprintf(stdout,"\t--recogThres        <double>\n");
        fprintf(stdout,"\t--adjNodesThres     <int>\n");
        fprintf(stdout,"\t--blobMinSizeThres  <int>\n");
        fprintf(stdout,"\t--framesPersistence <int>\n");
        fprintf(stdout,"\t--cropRadius        <int>\n");
    #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
        fprintf(stdout,"\t--numThreads        <int>\n");
    #endif
        fprintf(stdout,"\t--verbosity           -\n");
        fprintf(stdout,"\n");
        
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    ProcessModule mod;
    return mod.runModule(rf);
}


