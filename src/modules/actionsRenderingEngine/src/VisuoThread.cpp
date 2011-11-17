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
#include <yarp/os/Time.h>

#include <iCub/VisuoThread.h>


void VisuoThread::close()
{
    if (closed)
        return;

    interrupt();

    outPort[LEFT].close();
    outPort[RIGHT].close();

    imgPort[LEFT].close();
    imgPort[RIGHT].close();

    mCUTPort[LEFT].close();
    mCUTPort[RIGHT].close();

    pftInPort.close();
    pftOutPort.close();

    boundMILPort.close();
    cmdMILPort.close();
    recMILPort.close();

    cmdMSRPort.close();
    recMSRPort.close();

    if(img[LEFT]!=NULL)
    {
        delete img[LEFT];
        img[LEFT]=NULL;
    }

    if(img[RIGHT]!=NULL)
    {
        delete img[RIGHT];
        img[RIGHT]=NULL;
    }

    closed=true;
}


bool VisuoThread::checkTracker(Vector *vec)
{
    bool track=false;

    trackBuffer.push_back(*vec);

    if(isTracking() && trackBuffer.size()>minTrackBufSize)
    {
        double speed_avg[2];
        double speed_std[2];
        speed_avg[LEFT]=speed_avg[RIGHT]=0.0;
        speed_std[LEFT]=speed_std[RIGHT]=0.0;

        for(unsigned int i=1; i<trackBuffer.size(); i++)
        {
            double speed[2];
            speed[LEFT]=(trackBuffer[i-1][0]-trackBuffer[i][0])*(trackBuffer[i-1][0]-trackBuffer[i][0])+
                        (trackBuffer[i-1][1]-trackBuffer[i][1])*(trackBuffer[i-1][1]-trackBuffer[i][1]);
            speed_avg[LEFT]+=sqrt(speed[LEFT]);
            speed_std[LEFT]+=speed[LEFT];

            speed[RIGHT]=(trackBuffer[i-1][0]-trackBuffer[i][0])*(trackBuffer[i-1][0]-trackBuffer[i][0])+
                        (trackBuffer[i-1][1]-trackBuffer[i][1])*(trackBuffer[i-1][1]-trackBuffer[i][1]);
            speed_avg[RIGHT]+=sqrt(speed[RIGHT]);
            speed_std[RIGHT]+=speed[RIGHT];
        }

        double n=1.0/(trackBuffer.size()-1);
        speed_avg[LEFT]*=n;
        speed_std[LEFT]=n*speed_std[LEFT]-speed_avg[LEFT]*speed_avg[LEFT];
        speed_avg[RIGHT]*=n;
        speed_std[RIGHT]=n*speed_std[RIGHT]-speed_avg[RIGHT]*speed_avg[RIGHT];

        track=true;

        // check that the speeds are not varying too much
        if(speed_avg[LEFT]>speedStdThresh || speed_avg[RIGHT]>speedStdThresh)
        {
            track=false;
        }

        double dist=(trackBuffer.back()[0]-trackBuffer.back()[6])*(trackBuffer.back()[0]-trackBuffer.back()[6])+
                    (trackBuffer.back()[1]-trackBuffer.back()[7])*(trackBuffer.back()[1]-trackBuffer.back()[7]);

        // avoid strabicity
        if(sqrt(dist)>stereoDistThresh)
        {
            track=false;
        }
        // check that both images are tracking similar things


        while(trackBuffer.size()>maxTrackBufSize)
            trackBuffer.pop_front();
    }

    return track;
}


void VisuoThread::startTracker(const Vector &stereo, const int &side)
{
    IplImage *ipl_img, *ipl_tpl;

    int eye_in_use;

    if(stereo[2*dominant_eye]==0.0 && stereo[2*dominant_eye+1]==0.0)
    {
        if(stereo[2*(1-dominant_eye)]==0.0 && stereo[2*(1-dominant_eye)+1]==0.0)
            return;

        eye_in_use=1-dominant_eye;
    }
    else
        eye_in_use=dominant_eye;

    imgMutex.wait();
    ipl_img=img[eye_in_use]!=NULL? cvCloneImage((IplImage *) img[eye_in_use]->getIplImage()):NULL;
    imgMutex.post();

    if(ipl_img!=NULL)
    {
        int x=stereo[2*eye_in_use]-0.5*side<0?0:cvRound(stereo[2*eye_in_use]-0.5*side);
        int y=stereo[2*eye_in_use+1]-0.5*side<0?0:cvRound(stereo[2*eye_in_use+1]-0.5*side);
        int width=stereo[2*eye_in_use]+0.5*side>=ipl_img->width?ipl_img->width-x:side;
        int height=stereo[2*eye_in_use+1]+0.5*side>=ipl_img->height?ipl_img->height-y:side;

        ipl_tpl=cvCreateImage(cvSize(side,side),ipl_img->depth,ipl_img->nChannels);
        cvSetImageROI(ipl_img,cvRect(x,y,width,height));
        cvCopyImage(ipl_img,ipl_tpl);
        cvCvtColor(ipl_tpl,ipl_tpl,CV_BGR2RGB);
        //ImageOf<PixelBgr> tpl;
        if(tpl.width()>0)
        {
            IplImage *tmp_ipl=(IplImage*) tpl.getIplImage();
            //cvReleaseImage(&tmp_ipl);
        }        
        tpl.wrapIplImage(ipl_tpl);
        pftOutPort.write(tpl);

        cvReleaseImage(&ipl_img);
        cvReleaseImage(&ipl_tpl);
    }

    trackMutex.wait();
    stereoTracker.side=side;
    tracking=true;
    trackMutex.post();

    trackMode=MODE_TRACK_TEMPLATE;
}


void VisuoThread::restartTracker()
{
    pftOutPort.write(tpl);

    trackMutex.wait();
    tracking=true;
    trackMutex.post();
}

void VisuoThread::updateImages()
{
    ImageOf<PixelRgb> *iL=imgPort[LEFT].read(false);
    ImageOf<PixelRgb> *iR=imgPort[RIGHT].read(false);

    imgMutex.wait();
    if(iL!=NULL) 
    {
        if(img[LEFT]!=NULL)
            delete img[LEFT];

        img[LEFT]=new ImageOf<PixelRgb>(*iL);
        
        newImage[LEFT]=true;
    }

    if(iR!=NULL)
    {
        if(img[RIGHT]!=NULL)
            delete img[RIGHT];
        
        img[RIGHT]=new ImageOf<PixelRgb>(*iR);
        
        newImage[RIGHT]=true;
    }
    imgMutex.post();
}

void VisuoThread::updateLocationsMIL()
{
    Bottle *bLocations=recMILPort.read(false);

    if(bLocations!=NULL)// && bLocations->size()==1)
    {
        MILMutex.wait();
        /*        
        for(int cam=0; cam<2; cam++)
        {
            locations[cam].clear();
            Bottle *bCam=bLocations->get(cam).asList();
            for(int i=0; i<bCam->size(); i++)
            {
                Bottle *b=bCam->get(i).asList();
                locations[cam][b->get(0).asString().c_str()]=cvPoint(b->get(1).asInt(),b->get(2).asInt());
            }
        }
        */
        locations.clear();
        
        for(int i=0; i<bLocations->get(0).asList()->size(); i++)
        {
            Bottle *b=bLocations->get(0).asList()->get(i).asList();
            locations[b->get(0).asString().c_str()]=cvPoint(b->get(1).asInt(),b->get(2).asInt());
        }
        MILMutex.post();
    }
}


void VisuoThread::updateMotionCUT()
{
    // Detect motion
    Bottle *bMotion[2];
    bMotion[LEFT]=mCUTPort[LEFT].read(false);
    bMotion[RIGHT]=mCUTPort[RIGHT].read(false);

    double tNow=Time::now();

    bool detected=true;

    motMutex.wait();

    Vector stereo(4); stereo=0.0;
    for(int cam=0; cam<2; cam++)
    {
        // if even the first element of the buffer is far (in time)
        // clear the whole buffer
        if(buffer[cam].size()>0 && tNow-buffer[cam].back().t>timeTol)
            buffer[cam].clear();

        // If only a single moving blob has been detected
        if (bMotion[cam] && bMotion[cam]->size()==1)
        {
            Item item;
            item.t=Time::now();
            item.size=bMotion[cam]->get(0).asList()->get(2).asDouble();
            item.p=cvPoint(bMotion[cam]->get(0).asList()->get(0).asInt(),bMotion[cam]->get(0).asList()->get(1).asInt());
            buffer[cam].push_back(item);

            stereo[2*cam]=bMotion[cam]->get(0).asList()->get(0).asDouble();
            stereo[2*cam+1]=bMotion[cam]->get(0).asList()->get(1).asDouble();
        }

        // Avoid time unconsistencies
        while (buffer[cam].size() && (tNow-buffer[cam].front().t)>timeTol)
            buffer[cam].pop_front();
    }


    if(trackMode==MODE_TRACK_MOTION)
        stereo_target.set(stereo);


    motMutex.post();
}



void VisuoThread::updatePFTracker()
{
    Vector *trackVec=pftInPort.read(false);

    Vector stereo;
    if(trackVec!=NULL && trackVec->size()==12)
    {
        //must check if the tracker has gone mad.
        if(checkTracker(trackVec))
        {
            trackMutex.wait();
            stereoTracker.vec=*trackVec;
            trackMutex.post();

            stereo.resize(4);
            stereo[0]=stereoTracker.vec[0];
            stereo[1]=stereoTracker.vec[1];
            stereo[2]=stereoTracker.vec[6];
            stereo[3]=stereoTracker.vec[7];

            if(trackMode==MODE_TRACK_TEMPLATE)
                stereo_target.set(stereo);
        }
        else
        {
            trackMutex.wait();
            stereoTracker.vec.clear();
            stereoTracker.side=0;
            trackMutex.post();
        }
    }



    imgMutex.wait();
    if(img[LEFT]!=NULL && img[RIGHT]!=NULL)
    {
        Image drawImg[2];
        drawImg[LEFT]=*img[LEFT];
        drawImg[RIGHT]=*img[RIGHT];

        if(stereoTracker.vec.size()==12)
        {

            cvCircle(drawImg[LEFT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[0]),cvRound(stereoTracker.vec[1])),3,cvScalar(0,255),3);
            cvRectangle(drawImg[LEFT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[2]),cvRound(stereoTracker.vec[3])),
                                                 cvPoint(cvRound(stereoTracker.vec[4]),cvRound(stereoTracker.vec[5])),cvScalar(0,255),3);

            cvCircle(drawImg[RIGHT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[6]),cvRound(stereoTracker.vec[7])),3,cvScalar(0,255),3);
            cvRectangle(drawImg[RIGHT].getIplImage(),cvPoint(cvRound(stereoTracker.vec[8]),cvRound(stereoTracker.vec[9])),
                                                 cvPoint(cvRound(stereoTracker.vec[10]),cvRound(stereoTracker.vec[11])),cvScalar(0,255),3);

            Bottle v;
            v.clear();
            Bottle &vl=v.addList();
            vl.addInt(cvRound(stereoTracker.vec[0]));
            vl.addInt(cvRound(stereoTracker.vec[1]));
            vl.addInt(stereoTracker.side);
            Bottle &vr=v.addList();
            vr.addInt(cvRound(stereoTracker.vec[6]));
            vr.addInt(cvRound(stereoTracker.vec[7]));
            vr.addInt(stereoTracker.side);

            boundMILPort.write(v);
        }


        if(newImage[LEFT])
            outPort[LEFT].write(drawImg[LEFT]);
        
        if(newImage[RIGHT])
            outPort[RIGHT].write(drawImg[RIGHT]);

        //avoid writing multiple times the same image
        newImage[LEFT]=false;
        newImage[RIGHT]=false;
    }
    imgMutex.post();
}



VisuoThread::VisuoThread(ResourceFinder &_rf, Initializer *initializer)
    :RateThread(20),rf(_rf),stereo_target(initializer->stereo_target),opcPort(initializer->port_opc)
{
    buffer[LEFT].clear();
    buffer[RIGHT].clear();

    locations.clear();

    img[LEFT]=NULL;
    img[RIGHT]=NULL;

    stereoTracker.vec.clear();
    stereoTracker.side=0;

    trackMode=MODE_TRACK_TEMPLATE;
    closed=false;
}

bool VisuoThread::threadInit()
{
    string name=rf.find("name").asString().c_str();

    Bottle bVision=rf.findGroup("vision");

    setRate(bVision.check("period",Value(20)).asInt());

    minMotionBufSize=bVision.check("minMotionBufSize",Value(10)).asInt();
    minTrackBufSize=bVision.check("minTrackBufSize",Value(1)).asInt();
    maxTrackBufSize=bVision.check("maxTrackBufSize",Value(2)).asInt();
    timeTol=bVision.check("timeTol",Value(0.5)).asDouble();
    motionStdThresh=bVision.check("motionStdThresh",Value(5.0)).asDouble();
    speedStdThresh=bVision.check("speedStdThresh",Value(700.0)).asDouble();
    stereoDistThresh=bVision.check("stereoDistThresh",Value(300.0)).asDouble();
    dominant_eye=bVision.check("dominant_eye",Value("left")).asString()=="left"?LEFT:RIGHT;

    rawWaitThresh=bVision.check("raw_detection_wait_thresh",Value(15.0)).asDouble();
    motionWaitThresh=bVision.check("motion_detection_wait_thresh",Value(5.0)).asDouble();
    objectWaitThresh=bVision.check("object_detection_wait_thresh",Value(5.0)).asDouble();

    // open ports
    outPort[LEFT].open(("/"+name+"/left/img:o").c_str());
    outPort[RIGHT].open(("/"+name+"/right/img:o").c_str());

    imgPort[LEFT].open(("/"+name+"/left/img:i").c_str());
    imgPort[RIGHT].open(("/"+name+"/right/img:i").c_str());

    mCUTPort[LEFT].open(("/"+name+"/left/blobs:i").c_str());
    mCUTPort[RIGHT].open(("/"+name+"/right/blobs:i").c_str());

    rawInPort[LEFT].open(("/"+name+"/left/raw:i").c_str());
    rawInPort[RIGHT].open(("/"+name+"/right/raw:i").c_str());

    boundMILPort.open(("/"+name+"/MIL/window:o").c_str());
    cmdMILPort.open(("/"+name+"/MIL/cmd:o").c_str());
    recMILPort.open(("/"+name+"/MIL/rec:i").c_str());

    cmdMSRPort.open(("/"+name+"/MSR/cmd:o").c_str());
    recMSRPort.open(("/"+name+"/MSR/rec:i").c_str());


    pftInPort.open(("/"+name+"/tracker:i").c_str());
    pftOutPort.open(("/"+name+"/tracker:o").c_str());

    segPort.open(("/"+name+"/seg:o").c_str());

    newImage[LEFT]=false;
    newImage[RIGHT]=false;

    show=false;

    interrupted=false;

    trackMode=MODE_TRACK_TEMPLATE;

    closed=false;
    return true;
}

void VisuoThread::run()
{
    updateImages();
    updatePFTracker();
    updateLocationsMIL();
    updateMotionCUT();
}


void VisuoThread::threadRelease()
{
    close();
}



//get the action target point in the images reference frame
bool VisuoThread::getTarget(Value &type, Bottle &options)
{
    bool ok=false;

    Bottle &bNewTarget=options.addList();
    bNewTarget.addString("target");

    Bottle &bTarget=bNewTarget.addList();
    Bottle &bNewStereo=bTarget.addList();
    bNewStereo.addString("stereo");

    Bottle &bStereo=bNewStereo.addList();


    if(type.isList())
    {
        Bottle *list=type.asList();
        if(list->size()>2)
        {
            if(list->get(0).asString()=="right")
            {
                bStereo.addDouble(0.0);
                bStereo.addDouble(0.0);
                bStereo.addDouble(list->get(1).asDouble());
                bStereo.addDouble(list->get(2).asDouble());
                
            }
            else if(list->get(0).asString()=="left")
            {
                bStereo.addDouble(list->get(1).asDouble());
                bStereo.addDouble(list->get(2).asDouble());
                bStereo.addDouble(0.0);
                bStereo.addDouble(0.0);
            }
            else if(list->get(0).asString()=="cartesian")
            {
                bTarget.addList()=*list;
            }
            else
            {
                Bottle &bNewCartesian=bTarget.addList();
                bNewCartesian.addString("cartesian");

                Bottle &bCartesian=bNewCartesian.addList();

                for(int i=0; i<list->size(); i++)
                    bCartesian.addDouble(list->get(i).asDouble());
            }

            ok=true;
        }
        else if(list->size()==2)
        {
            bStereo.addDouble(list->get(0).asDouble());
            bStereo.addDouble(list->get(1).asDouble());
            bStereo.addDouble(0.0);
            bStereo.addDouble(0.0);

            ok=true;
        }
    }
    else switch(type.asVocab())
    {
        case PARAM_FIXATION:
        {
            ok=getFixation(bStereo);
            break;
        }
        case PARAM_MOTION:
        {
            ok=getMotion(bStereo);
            break;
        }
        case PARAM_TRACK:
        {
            ok=getTrack(bStereo);
            break;
        }
        case PARAM_RAW:
        {
            ok=getRaw(bStereo);
            break;
        }

        default:
        {
            Bottle &bName=bTarget.addList();
            bName.addString("name");
            bName.addString(type.asString().c_str());

            getObject(type.asString().c_str(),bStereo);
            break;
        }
    }


    return ok;
}


bool VisuoThread::getFixation(Bottle &bStereo)
{
    Vector stereo(4);

    imgMutex.wait();
    if(img[LEFT]!=NULL)
    {
        stereo[0]=stereo[2]=0.5*img[LEFT]->width();
        stereo[1]=stereo[3]=0.5*img[LEFT]->height();
    }
    else
    {
        stereo[0]=stereo[2]=160;
        stereo[1]=stereo[3]=120;
    }
    imgMutex.post();

    for(size_t i=0; i<stereo.size(); i++)
        bStereo.addDouble(stereo[i]);

    int side=40;
    startTracker(stereo,side);

    return true;
}

bool VisuoThread::getMotion(Bottle &bStereo)
{
    Vector stereo;

    bool ok=false;

    double t=Time::now();

    while(Time::now()-t<motionWaitThresh && !interrupted)
    {
        motMutex.wait();
        // If the buffers are sufficently dense and not so small, return true.
        double size=0.0;
        if (buffer[LEFT].size()>minMotionBufSize && buffer[RIGHT].size()>minMotionBufSize)
        {
            Vector p[2];
            for (int cam=0; cam<2; cam++)
            {
                double size_cam,u,v,n;
                double u_std,v_std;
                size_cam=u=v=0.0;
                u_std=v_std=0.0;
                n=1.0/buffer[cam].size();

                for (unsigned int i=0; i<buffer[cam].size(); i++)
                {
                    size_cam+=buffer[cam][i].size;
                    u+=buffer[cam][i].p.x;
                    v+=buffer[cam][i].p.y;
                    u_std+=buffer[cam][i].p.x*buffer[cam][i].p.x;
                    v_std+=buffer[cam][i].p.y*buffer[cam][i].p.y;
                }

                size_cam*=n;
                u*=n;
                v*=n;
                u_std=sqrt(n*u_std-u*u);
                v_std=sqrt(n*v_std-v*v);

                //check if the motion detected point is not wildly moving
                if (u_std<motionStdThresh && v_std<motionStdThresh)
                {
                    p[cam].resize(2);
                    p[cam][0]=u;
                    p[cam][1]=v;
                }
                else
                    break;

                size+=size_cam;
            }

            int side=cvRound(2*sqrt(size/3.1415)*2);

            if (p[LEFT].size()==2 && p[RIGHT].size()==2)
            {
                stereo.resize(4);
                stereo[0]=p[LEFT][0];
                stereo[1]=p[LEFT][1];
                stereo[2]=p[RIGHT][0];
                stereo[3]=p[RIGHT][1];
                
                startTracker(stereo,cvRound(side));

                ok=true;
            }
        }
        motMutex.post();
    }

    for(size_t i=0; i<stereo.size(); i++)
        bStereo.addDouble(stereo[i]);

    return ok;
}


bool VisuoThread::getTrack(Bottle &bStereo)
{
    Vector stereo;

    bool ok=false;

    trackMutex.wait();
    if(stereoTracker.vec.size()==12)
    {
        stereo.resize(4);
        stereo[0]=stereoTracker.vec[0];
        stereo[1]=stereoTracker.vec[1];
        stereo[2]=stereoTracker.vec[6];
        stereo[3]=stereoTracker.vec[7];
    }
    trackMutex.post();

    for(size_t i=0; i<stereo.size(); i++)
        bStereo.addDouble(stereo[i]);

    return ok;
}



bool VisuoThread::getRaw(Bottle &bStereo)
{
    Vector stereo;

    bool ok=false;

    //empty the buffers
    Bottle *bL=rawInPort[LEFT].read(false);
    Bottle *bR=rawInPort[RIGHT].read(false);

    double t=Time::now();

    while(stereo.size()!=4 && Time::now()-t<rawWaitThresh && !interrupted)
    {
        Bottle *bL=rawInPort[LEFT].read(false);
        Bottle *bR=rawInPort[RIGHT].read(false);
        
        if(bL!=NULL || bR!=NULL)
            stereo.resize(4,0.0);

        if(bL!=NULL)
        {
            stereo[0]=bL->get(0).asInt();
            stereo[1]=bL->get(1).asInt();
        }
        
        if(bR!=NULL)
        {
            stereo[2]=bR->get(0).asInt();
            stereo[3]=bR->get(1).asInt();
        }
    }

    if(stereo.size()==4)
    {
        int side=40;
        startTracker(stereo,side);
        ok=true;
    }

    for(size_t i=0; i<stereo.size(); i++)
        bStereo.addDouble(stereo[i]);

    return ok;
}

// if the object is located in both cameras, return its stereo position
bool VisuoThread::getObject(const std::string &object_name, Bottle &bStereo)
{
    Vector stereo;

    bool ok=opcPort.getStereoPosition(object_name,stereo);

    if(!ok)
    {
        double t=Time::now();
        while(Time::now()-t<objectWaitThresh && !interrupted)
        {
            MILMutex.wait();
            if(locations.count(object_name)>0)
            {
                stereo.resize(4);
                stereo[0]=locations[object_name].x;
                stereo[1]=locations[object_name].y;
                stereo[2]=160.0;
                stereo[3]=120.0;

                ok=true;
            }
            MILMutex.post();
        }
    }

    for(size_t i=0; i<stereo.size(); i++)
        bStereo.addDouble(stereo[i]);

    return ok;
}


Bottle VisuoThread::recogMSR(string &obj_name)
{
    Bottle bDetect;
    Bottle *bMSR=recMSRPort.read(false);
    if(bMSR!=NULL)
    {
        bDetect=*bMSR;
        obj_name=bDetect.get(0).asString().c_str();
    }
    return bDetect;
}



bool VisuoThread::startLearningMIL(const std::string &obj_name)
{
    if(!isTracking())
        return false;

    Bottle command,reply;
    command.fromString(("learn " + obj_name + " template").c_str());

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}


bool VisuoThread::suspendLearningMIL()
{
    Bottle command,reply;
    command.fromString("set label 0");

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}

bool VisuoThread::resumeLearningMIL()
{
    Bottle command,reply;
    command.fromString("set label template");

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}

bool VisuoThread::trainMIL()
{
    Bottle command,reply;
    command.fromString("train");

    cmdMILPort.write(command,reply);

    return (reply.get(0).toString()=="ack");
}



void VisuoThread::interrupt()
{
    outPort[LEFT].interrupt();
    outPort[RIGHT].interrupt();

    imgPort[LEFT].interrupt();
    imgPort[RIGHT].interrupt();

    mCUTPort[LEFT].interrupt();
    mCUTPort[RIGHT].interrupt();

    rawInPort[LEFT].interrupt();
    rawInPort[RIGHT].interrupt();

    pftInPort.interrupt();
    pftOutPort.interrupt();

    boundMILPort.interrupt();
    cmdMILPort.interrupt();
    recMILPort.interrupt();

    cmdMSRPort.interrupt();
    recMSRPort.interrupt();

    opcPort.interrupt();

    interrupted=true;
}

void VisuoThread::reinstate()
{
    outPort[LEFT].resume();
    outPort[RIGHT].resume();

    imgPort[LEFT].resume();
    imgPort[RIGHT].resume();

    mCUTPort[LEFT].resume();
    mCUTPort[RIGHT].resume();

    rawInPort[LEFT].resume();
    rawInPort[RIGHT].resume();

    pftInPort.resume();
    pftOutPort.resume();

    boundMILPort.resume();
    cmdMILPort.resume();
    recMILPort.resume();

    cmdMSRPort.resume();
    recMSRPort.resume();

    opcPort.resume();

    interrupted=false;
}


