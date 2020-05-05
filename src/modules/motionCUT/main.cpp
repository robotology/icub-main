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

#include <cmath>
#include <string>
#include <set>
#include <vector>
#include <deque>
#include <algorithm>
#include <utility>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/cv/Cv.h>

// in BGR format
#define NODE_OFF    Scalar(0,0,255)
#define NODE_ON     Scalar(0,255,0)

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::cv;


/************************************************************************/
class Blob
{
public:
    Point centroid;
    int   size;

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
    int nodesX;
    int nodesY;

    ImageOf<PixelMono> imgMonoIn;
    ImageOf<PixelMono> imgMonoPrev;
    vector<Mat>        pyrPrev;
    vector<Mat>        pyrCurr;

    vector<Point2f>    nodesPrev;
    vector<Point2f>    nodesCurr;
    vector<uchar>      featuresFound;
    vector<float>      featuresErrors;
    vector<int>        nodesPersistence;

    set<int>           activeNodesIndexSet;
    deque<Blob>        blobSortedList;

    BufferedPort<ImageOf<PixelBgr>>  inPort;
    BufferedPort<ImageOf<PixelBgr>>  outPort;
    BufferedPort<ImageOf<PixelMono>> optPort;
    BufferedPort<ImageOf<PixelBgr>>  cropPort;
    BufferedPort<Bottle>             nodesPort;
    BufferedPort<Bottle>             blobsPort;

public:
    /************************************************************************/
    ProcessThread(ResourceFinder &_rf) : rf(_rf) { }

    /************************************************************************/
    bool threadInit()
    {
        name=rf.check("name",Value("motionCUT")).asString();
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

        inPort.open("/"+name+"/img:i");
        outPort.open("/"+name+"/img:o");
        optPort.open("/"+name+"/opt:o");
        nodesPort.open("/"+name+"/nodes:o");
        blobsPort.open("/"+name+"/blobs:o");
        cropPort.open("/"+name+"/crop:o");

        firstConsistencyCheck=true;

        return true;
    }

    /************************************************************************/
    void afterStart(bool s)
    {
        if (s)
        {
            yInfo("Process started successfully");
            yInfo("Using ...");
            yInfo("name              = %s",name.c_str());
            yInfo("coverXratio       = %g",coverXratio);
            yInfo("coverYratio       = %g",coverYratio);
            yInfo("nodesStep         = %d",nodesStep);
            yInfo("winSize           = %d",winSize);
            yInfo("recogThres        = %g",recogThres);
            yInfo("recogThresAbs     = %g",recogThresAbs);
            yInfo("adjNodesThres     = %d",adjNodesThres);
            yInfo("blobMinSizeThres  = %d",blobMinSizeThres);
            yInfo("framesPersistence = %d",framesPersistence);
            if (cropSize>0)
                yInfo("cropSize          = %d",cropSize);
            else
                yInfo("cropSize          = auto");
            yInfo("verbosity         = %s",verbosity?"on":"off");
        }
        else
            yError("Process did not start");
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

                int min_x=(int)(((1.0-coverXratio)/2.0)*imgMonoIn.width());
                int min_y=(int)(((1.0-coverYratio)/2.0)*imgMonoIn.height());

                nodesX=((int)imgMonoIn.width()-2*min_x)/nodesStep+1;
                nodesY=((int)imgMonoIn.height()-2*min_y)/nodesStep+1;

                int nodesNum=nodesX*nodesY;
                nodesPrev.assign(nodesNum,Point2f(0.0f,0.0f));
                nodesCurr.assign(nodesNum,Point2f(0.0f,0.0f));
                featuresFound.assign(nodesNum,0);
                featuresErrors.assign(nodesNum,0.0f);
                nodesPersistence.assign(nodesNum,0);

                // populate grid
                size_t cnt=0;
                for (int y=min_y; y<=(imgMonoIn.height()-min_y); y+=nodesStep)
                    for (int x=min_x; x<=(imgMonoIn.width()-min_x); x+=nodesStep)
                        nodesPrev[cnt++]=Point2f((float)x,(float)y);

                // convert to gray-scale
                cvtColor(toCvMat(*pImgBgrIn),toCvMat(imgMonoPrev),CV_BGR2GRAY);

                if (verbosity)
                {
                    // log message
                    yInfo("Detected image of size %zdx%zd; using %dx%d=%d nodes; populated %zd nodes",
                          imgMonoIn.width(),imgMonoIn.height(),nodesX,nodesY,nodesNum,cnt);
                }

                // skip to the next cycle
                continue;
            }

            // convert the input image to gray-scale
            cvtColor(toCvMat(*pImgBgrIn),toCvMat(imgMonoIn),CV_BGR2GRAY);

            // copy input image into output image
            ImageOf<PixelBgr> imgBgrOut=*pImgBgrIn;
            Mat imgBgrOutMat=toCvMat(imgBgrOut);

            // get optFlow image
            ImageOf<PixelMono> imgMonoOpt;
            imgMonoOpt.resize(imgBgrOut);
            imgMonoOpt.zero();
            Mat imgMonoOptMat=toCvMat(imgMonoOpt);

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
            constexpr int maxLevel=5;
            Size ws(winSize,winSize);
            buildOpticalFlowPyramid(toCvMat(imgMonoPrev),pyrPrev,ws,maxLevel);
            buildOpticalFlowPyramid(toCvMat(imgMonoIn),pyrCurr,ws,maxLevel);
            calcOpticalFlowPyrLK(pyrPrev,pyrCurr,nodesPrev,nodesCurr,
                                 featuresFound,featuresErrors,ws,maxLevel,
                                 TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,30,0.3));
            dt0=Time::now()-latch_t;

            // assign status to the grid nodes
            latch_t=Time::now();
            for (size_t i=0; i<nodesPrev.size(); i++)
            {
                bool persistentNode=false;
                Point node=Point((int)nodesPrev[i].x,(int)nodesPrev[i].y);

                // handle the node persistence
                if (!inhibition && (nodesPersistence[i]!=0))
                {
                    circle(imgBgrOutMat,node,1,NODE_ON,2);
                    circle(imgMonoOptMat,node,1,Scalar(255),2);

                    Bottle &nodeBottle=nodesBottle.addList();
                    nodeBottle.addInt((int)nodesPrev[i].x);
                    nodeBottle.addInt((int)nodesPrev[i].y);

                    // update the active nodes set
                    activeNodesIndexSet.insert((int)i);

                    nodesPersistence[i]--;
                    persistentNode=true;
                }
                else
                    circle(imgBgrOutMat,node,1,NODE_OFF,1);

                // do not consider the border nodes and skip if inhibition is on
                int row=i%nodesX;
                bool skip=inhibition || (i<nodesX) || (i>=(nodesPrev.size()-nodesX)) || (row==0) || (row==(nodesX-1));

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
                            circle(imgBgrOutMat,node,1,NODE_ON,2);
                            circle(imgMonoOptMat,node,1,Scalar(255),2);

                            Bottle &nodeBottle=nodesBottle.addList();
                            nodeBottle.addInt((int)nodesPrev[i].x);
                            nodeBottle.addInt((int)nodesPrev[i].y);

                            // update the active nodes set
                            activeNodesIndexSet.insert((int)i);
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

                Point centroid=Point(blob.centroid.x,blob.centroid.y);

                Bottle &blobBottle=blobsBottle.addList();
                blobBottle.addInt(centroid.x);
                blobBottle.addInt(centroid.y);
                blobBottle.addInt(blob.size);

                circle(imgBgrOutMat,centroid,4,Scalar(blueLev,0,redLev),3);
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

                Point tl=Point(std::max(x-d2,0),std::max(y-d2,0));
                Point br=Point(std::min(x+d2,(int)pImgBgrIn->width()-1),std::min(y+d2,(int)pImgBgrIn->height()-1));
                Point cropSize=Point(br.x-tl.x,br.y-tl.y);

                ImageOf<PixelBgr> &cropImg=cropPort.prepare();
                cropImg.resize(cropSize.x,cropSize.y);
                toCvMat(*pImgBgrIn)(Rect(tl.x,tl.y,cropSize.x,cropSize.y)).copyTo(toCvMat(cropImg));

                cropPort.setEnvelope(stamp);
                cropPort.write();
            }

            // save data for next cycle
            imgMonoPrev=imgMonoIn;

            double t1=Time::now();
            if (verbosity)
            {
                // dump statistics
                yInfo("cycle timing [ms]: optflow(%g), colorgrid(%g), blobdetection(%g), overall(%g)",
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
        auto el=activeNodesIndexSet.find(i);
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
            string cmd=req.get(0).asString();

            if (cmd=="set")
            {
                if (req.size()<3)
                    return false;

                string subcmd=req.get(1).asString();

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

                string subcmd=req.get(1).asString();

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
        thr=new ProcessThread(rf);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        rpcPort.open("/"+thr->getName()+"/rpc");
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
    Network yarp;

    ResourceFinder rf;
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<<"Available options:"<<endl;
        cout<<"\t--name              <string>"<<endl;
        cout<<"\t--coverXratio       <double>"<<endl;
        cout<<"\t--coverYratio       <double>"<<endl;
        cout<<"\t--nodesStep         <int>"<<endl;
        cout<<"\t--winSize           <int>"<<endl;
        cout<<"\t--recogThres        <double>"<<endl;
        cout<<"\t--adjNodesThres     <int>"<<endl;
        cout<<"\t--blobMinSizeThres  <int>"<<endl;
        cout<<"\t--framesPersistence <int>"<<endl;
        cout<<"\t--cropSize          \"auto\" or <int>"<<endl;
        cout<<"\t--verbosity"<<endl;
        cout<<endl;
        return 0;
    }

    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    ProcessModule mod;
    return mod.runModule(rf);
}
