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

/**
@ingroup icub_module

\defgroup motionCUT motionCUT
 
Detects independent moving points of a grid used to sample the 
input images. The algorithm works also with moving cameras.

Copyright (C) 2010 RobotCub Consortium
 
Authors: Carlo Ciliberto and Ugo Pattacini 
 
Date: first release on the night of 03/05/2010 :)

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module exploits the pyramidal Lucas-Kanade algorithm to
detect independent moving points over a selectable grid of 
nodes. The algorithm is designed in such a way that it works 
also - and especially - with moving cameras. 
 
\note the name motionCUT stands for <i>motion Cover/Uncover 
      Trick</i> and refers to its working principle that is
      detailed in the paper: Ciliberto C., Pattacini U., Natale
      L., Nori F. and Metta G., "Reexamining Lucas-Kanade Method
      for Real-Time Independent Motion Detection:
      Application to the iCub Humanoid Robot", <i>IEEE/RSJ
      International Conference on Intelligent Robots and
      Systems</i>, San Francisco, CA, USA, 2011.
 
\note <b>If you're going to use the motionCUT for your work, 
      please quote it within any resulting publication</b>.
 
\note We warmly suggest to use OpenCV in conjunction with 
      multi-threading layers such as OpenMP and TBB in order to
      achieve high performances for motion detection. Refer to
      the OpenCV documentation for the details.
 
\note A video on iCub employing \e motionCUT can be seen <a
      href="http://www.youtube.com/watch?v=Ql8Qe0oxHaY">here</a>.
 
\section lib_sec Libraries 
YARP libraries and OpenCV

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module.
 
--coverXratio \e ratioX
- The parameter \e ratioX identifies the portion of the x-axis 
  of the image covered by the grid nodes. Example: if
  ratioX=0.75, then the central 3/4 of the x-axis will be
  covered with points.
 
--coverYratio \e ratioY
- The analogous for the y-axis image.
 
--nodesStep \e step
- The parameter \e step selects the step in pixels between two 
  consecutive grid nodes.
 
--winSize \e size
- The parameter \e size selects window size used by the 
  algorithm.
 
--recogThres \e thres
- The parameter \e thres, given in percentage, specifies the 
  error threshold that allows discriminating between background
  and independent moving nodes as result of a matching carried
  out on the windows whose size is determined by \e winSize
  parameter. Usually very small values, such as 0.5%, have to be
  used. Indicatively, a value of 0.5% means that the two
  templates for the matching must have a similarity measure of
  99.5% to prevent the relative node from being recognized as
  independent moving node. Importantly, this parameter is also
  significantly affected by how well the matching is performed;
  as result, for increasing OpenCV versions, the threshold needs
  to be reduced (e.g. 0.01%).
 
--adjNodesThres \e min 
- This parameter allows filtering out the \e salt-and-pepper 
  noise over the output image, by specifying the minimum number
  of adjacent nodes that must be active (i.e. that undergo the
  motion) in the neighbourhood of any single node to keep it
  active.
 
--blobMinSizeThres \e min 
- This parameter allows filtering out blobs whose nodes number 
  is lower than <min>.

--framesPersistence \e frames
- This parameter allows increasing the node persistence over 
  consecutive frames implementing a sort of low-pass filter. The
  value \e frames specifies the number of consecutive frames for
  which if a node gets active it is kept on.
 
--cropSize \e d 
- If \e d is a positive integer, it specifies the side of a 
  squared cropping window placed on the center of the largest
  blob detected. By default, \e d is "auto", meaning that the
  cropping window will adapt to the size of the blob.

--numThreads \e threads
- This parameter allows controlling the maximum number of 
  threads allocated by parallelized OpenCV functions. This
  option is available only if the OpenMP layer is supported. By
  contrast, the TBB layer automatically determines the number of
  threads.\n
  \e #  > 0 : assign # threads to OpenCV; \n
  \e # == 0 : assign all threads to OpenCV; \n
  \e #  < 0 : assign all threads but # to OpenCV; \n
  The default value is -1 meaning that all threads equal to the
  number of available cores BUT ONE will be used.
 
--verbosity 
- Enable the dump of log messages.
 
\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
- <i> /<stemName>/img:i </i> accepts the incoming images. 
 
- <i> /<stemName>/img:o </i> outputs the input images with the 
  grid layer on top. This port propagates the time-stamp carried
  by the input image.
 
- <i> /<stemName>/nodes:o </i> outputs the x-y location of the 
  currently active nodes in this format: (nodesStep <val>)
  (<n0.x> <n0.y>) (<n1.x> <n1.y>) ... . This port propagates the
  time-stamp carried by the input image.
 
- <i> /<stemName>/blobs:o </i> outputs the x-y location of blobs
  centroids along with their size in this format: (<b0.cx>
  <b0.cy> <b0.size>) (<b1.cx> <b1.cy> <b1.size>) ... The output
  blobs list is sorted according to their size (decreasing
  order). This port propagates the time-stamp carried
  by the input image.

- <i> /<stemName>/crop:o </i> outputs a window containing a ROI 
  around the center of mass of the largest blob detected.
 
- <i> /<stemName>/opt:o </i> outputs monochrome images 
  containing just the grid nodes signalling independent
  movements. This port propagates the time-stamp carried
  by the input image.
 
- <i> /<stemName>/rpc </i> for RPC communication. 
 
\section rpcProto_sec RPC protocol 
The parameters <i> winSize, recogThres, adjNodesThres, 
blobMinSizeThres, framesPersistence, cropSize, numThreads, 
verbosity </i> can be changed/retrieved through the commands 
set/get. Moreover the further switch \e inhibition can be 
accessed in order to enable/disable the motion detection at 
run-time. 
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux and Windows.

\author Carlo Ciliberto and Ugo Pattacini
*/ 

#include <stdio.h>
#include <string>
#include <set>
#include <vector>
#include <deque>
#include <algorithm>

#include <cv.h>
#include <math.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

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
    int cropSize;
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
    BufferedPort<ImageOf<PixelBgr> >  cropPort;
    BufferedPort<Bottle>              nodesPort;
    BufferedPort<Bottle>              blobsPort;

    /************************************************************************/
    void disposeMem()
    {
        delete nodesPrev;
        delete nodesCurr;
        delete nodesPersistence;
        delete featuresFound;
        delete featuresErrors;

        nodesPrev=NULL;
        nodesCurr=NULL;
        nodesPersistence=NULL;
        featuresFound=NULL;
        featuresErrors=NULL;
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
        recogThres=rf.check("recogThres",Value(0.01)).asDouble();
        adjNodesThres=rf.check("adjNodesThres",Value(4)).asInt();
        blobMinSizeThres=rf.check("blobMinSizeThres",Value(10)).asInt();
        framesPersistence=rf.check("framesPersistence",Value(3)).asInt();        
        verbosity=rf.check("verbosity");

        cropSize=0;
        if (rf.check("cropSize"))
        {
            Value &vCropSize=rf.find("cropSize");
            if (!vCropSize.isString())
                cropSize=vCropSize.asInt();
        }

        recogThresAbs=recogThres*((256*winSize*winSize)/100.0);
        inhibition=false;

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
            printf("Process started successfully\n");
            printf("\n");
            printf("Using ...\n");
            printf("name              = %s\n",name.c_str());
            printf("coverXratio       = %g\n",coverXratio);
            printf("coverYratio       = %g\n",coverYratio);
            printf("nodesStep         = %d\n",nodesStep);
            printf("winSize           = %d\n",winSize);
            printf("recogThres        = %g\n",recogThres);
            printf("recogThresAbs     = %g\n",recogThresAbs);
            printf("adjNodesThres     = %d\n",adjNodesThres);
            printf("blobMinSizeThres  = %d\n",blobMinSizeThres);
            printf("framesPersistence = %d\n",framesPersistence);
            if (cropSize>0)
                printf("cropSize          = %d\n",cropSize);
            else
                printf("cropSize          = auto\n");
            
        #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
            printf("numThreads        = %d\n",numThreads);
        #else
            printf("numThreads        = OpenCV version does not support OpenMP multi-threading\n");
        #endif
            
            printf("verbosity         = %s\n",verbosity?"on":"off");            
            printf("\n");
        }
        else
            printf("Process did not start\n");
    }

    /************************************************************************/
    void run()
    {
        double latch_t, dt0, dt1, dt2;

        while (!isStopping())
        {
            // acquire new image
            ImageOf<PixelBgr> *pImgBgrIn=inPort.read(true);
            if (isStopping() || (pImgBgrIn==NULL))
                break;

            // get the envelope from the image
            Stamp stamp;
            inPort.getEnvelope(stamp);

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
                    printf("Detected image of size %dx%d;\nusing %dx%d=%d nodes;\npopulated %d nodes\n",
                           imgMonoIn.width(),imgMonoIn.height(),nodesX,nodesY,nodesNum,cnt);
                }

                // skip to the next cycle
                continue;
            }

            // convert the input image to gray-scale
            cvCvtColor(pImgBgrIn->getIplImage(),imgMonoIn.getIplImage(),CV_BGR2GRAY);

            // copy input image into output image
            ImageOf<PixelBgr> imgBgrOut=*pImgBgrIn;

            // get optFlow image
            ImageOf<PixelMono> imgMonoOpt;
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
                outPort.prepare()=imgBgrOut;
                outPort.setEnvelope(stamp);
                outPort.write();
            }

            if (optPort.getOutputCount()>0)
            {
                optPort.prepare()=imgMonoOpt;
                optPort.setEnvelope(stamp);
                optPort.write();
            }

            // send out data bottles, propagating the time-stamp
            if ((nodesPort.getOutputCount()>0) && (nodesBottle.size()>1))
            {
                nodesPort.prepare()=nodesBottle;
                nodesPort.setEnvelope(stamp);
                nodesPort.write();
            }

            if ((blobsPort.getOutputCount()>0) && (blobsBottle.size()>0))
            {
                blobsPort.prepare()=blobsBottle;
                blobsPort.setEnvelope(stamp);
                blobsPort.write();
            }
            
            if ((cropPort.getOutputCount()>0) && (blobsBottle.size()>0))
            {
                Bottle &blob=*blobsBottle.get(0).asList();
                int x=blob.get(0).asInt();
                int y=blob.get(1).asInt();
                int d=(cropSize>0)?cropSize:(int)(nodesStep*sqrt((double)blob.get(2).asInt()));
                int d2=d>>1;

                CvPoint tl=cvPoint(std::max(x-d2,0),std::max(y-d2,0));
                CvPoint br=cvPoint(std::min(x+d2,pImgBgrIn->width()-1),std::min(y+d2,pImgBgrIn->height()-1));
                CvPoint cropSize=cvPoint(br.x-tl.x,br.y-tl.y);

                ImageOf<PixelBgr> &cropImg=cropPort.prepare();
                cropImg.resize(cropSize.x,cropSize.y);
                
                cvSetImageROI((IplImage*)pImgBgrIn->getIplImage(),cvRect(tl.x,tl.y,cropSize.x,cropSize.y));
                cvCopy((IplImage*)pImgBgrIn->getIplImage(),(IplImage*)cropImg.getIplImage());
                cvResetImageROI((IplImage*)pImgBgrIn->getIplImage());
                            
                cropPort.setEnvelope(stamp);
                cropPort.write();
            }

            // save data for next cycle
            imgMonoPrev=imgMonoIn;
            
            double t1=Time::now();
            if (verbosity)
            {
                // dump statistics
                printf("cycle timing [ms]: optflow(%g), colorgrid(%g), blobdetection(%g), overall(%g)\n",
                       1000.0*dt0,1000.0*dt1,1000.0*dt2,1000.0*(t1-t0));
            }
        }
    }

    /************************************************************************/
    void onStop()
    {
        inPort.interrupt();
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
                else if (subcmd=="cropSize")
                {
                    Value &vCropSize=req.get(2);
                    if (!vCropSize.isString())
                        cropSize=vCropSize.asInt();
                    else
                        cropSize=0;

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
                else if (subcmd=="cropSize")
                {
                    if (cropSize>0)
                        reply.addInt(cropSize);
                    else
                        reply.addString("auto");
                }
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
        // request high resolution scheduling
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
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        printf("\n");
    #ifdef CV_MAJOR_VERSION
        printf("This module has been compiled with OpenCV %d.%d\n",CV_MAJOR_VERSION,CV_MINOR_VERSION);
    #else
        printf("This module has been compiled with an unknown version of OpenCV (probably < 1.0)\n");
    #endif        
        printf("\n");
        printf("Available options:\n");
        printf("\t--name              <string>\n");
        printf("\t--coverXratio       <double>\n");
        printf("\t--coverYratio       <double>\n");
        printf("\t--nodesStep         <int>\n");
        printf("\t--winSize           <int>\n");
        printf("\t--recogThres        <double>\n");
        printf("\t--adjNodesThres     <int>\n");
        printf("\t--blobMinSizeThres  <int>\n");
        printf("\t--framesPersistence <int>\n");
        printf("\t--cropSize          \"auto\" or <int>\n");
    #ifdef _MOTIONCUT_MULTITHREADING_OPENMP
        printf("\t--numThreads        <int>\n");
    #endif
        printf("\t--verbosity           -\n");
        printf("\n");
        
        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ProcessModule mod;
    return mod.runModule(rf);
}


