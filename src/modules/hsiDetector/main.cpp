// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
@ingroup icub_tools

\defgroup hsiDetector hsiDetector
 
Provides binary FG images detecting the HUE, SATURATION, and INTENSITY values as specified. 

Copyright (C) 2010 RobotCub Consortium
 
Authors: Stephen Hart 
 
Date: May 7, 2010

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module exploits provides binary images corresponding to specified HSI channels.  
 
\section lib_sec Libraries 
YARP libraries and OpenCV

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. By default \e hsiDetector is
  useds
 

\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created
- <i> /<stemName>/img:i </i> accepts the incoming images. 
 
- <i> /<stemName>/hue/<id>/img:o </i> outputs the HUE channel "id" image
- <i> /<stemName>/sat/<id>/img:o </i> outputs the SATURATION channel "id" image
- <i> /<stemName>/int/<id>/img:o </i> outputs the INTENSITY channel "id" image
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux.

\author Stephen Hart
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include <cv.h>

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class ProcessThread : public Thread
{
private:
    ResourceFinder &rf;
    
    string name;
    int numChannels;
    
    int width;
    int height;
   
    IplImage *hsv;
    IplImage *hue;
    IplImage *saturation;
    IplImage *intensity;

    vector<int> hueChannels;
    vector<int> satChannels;
    vector<int> intChannels;

    Bottle hueBottle;
    Bottle satBottle;
    Bottle intBottle;
	
    ImageOf<PixelMono> imgFilteredIn;
    
    BufferedPort<ImageOf<PixelBgr> >  inPort;
    BufferedPort<ImageOf<PixelMono> >  outPort;
    vector< BufferedPort<ImageOf<PixelMono> > * >  huePorts;
    vector< BufferedPort<ImageOf<PixelMono> > * >  satPorts;
    vector< BufferedPort<ImageOf<PixelMono> > * >  intPorts;
    
public:
    ProcessThread(ResourceFinder &_rf) : rf(_rf) { }
    
    virtual bool threadInit()
    {
        
        vector<int>::iterator p;
        stringstream ss;
        int v;
        
        name=rf.check("name",Value("hsiDetector")).asString().c_str();
        numChannels=rf.check("num",Value(10)).asInt();
        hueBottle=rf.findGroup("hue", "hue channels");
        satBottle=rf.findGroup("saturation", "saturation channels");
        intBottle=rf.findGroup("intensity", "intensity channels");
        
        inPort.open(("/"+name+"/img:i").c_str());
        outPort.open(("/"+name+"/sat/img:o").c_str());
        
        // go through and see which channels we are going to start up
        for(int i=1; i<hueBottle.size(); i++) {
            v = hueBottle.get(i).asInt();
            p = find(hueChannels.begin(), hueChannels.end(), v);
            if((p == hueChannels.end()) && (v>=0) && (v<numChannels)) {
                hueChannels.push_back(v);
            }
        } 
        for(int i=1; i<satBottle.size(); i++) {
            v = satBottle.get(i).asInt();
            p = find(satChannels.begin(), satChannels.end(), v);
            if((p == satChannels.end()) && (v>=0) && (v<numChannels)) {
                satChannels.push_back(v);
            }
        }
        for(int i=1; i<intBottle.size(); i++) {
            v = intBottle.get(i).asInt();
            p = find(intChannels.begin(), intChannels.end(), v);
            if((p == intChannels.end()) && (v>=0) && (v<numChannels)) {
                intChannels.push_back(v);
            }
        }    
        
        sort(hueChannels.begin(),hueChannels.end());
        sort(satChannels.begin(),satChannels.end());
        sort(intChannels.begin(),intChannels.end());
        
        width = 0;
        height = 0;
        
        for(int i=0; i<hueChannels.size(); i++) {
            //cout << "adding hue channel " << hueChannels[i] << endl; 
            huePorts.push_back(new BufferedPort<ImageOf<PixelMono> >);
            ss.str(""); ss.clear();		
            ss << hueChannels[i];
            huePorts[i]->open(("/"+name+"/hue/"+ss.str()+"/img:o").c_str());	
        }
        for(int i=0; i<satChannels.size(); i++) {
            //cout << "adding saturation channel " << satChannels[i] << endl; 
            satPorts.push_back(new BufferedPort<ImageOf<PixelMono> >);
            ss.str(""); ss.clear();		
            ss << satChannels[i];
            satPorts[i]->open(("/"+name+"/sat/"+ss.str()+"/img:o").c_str());	
        }
        for(int i=0; i<intChannels.size(); i++) {
            //cout << "adding intensity channel " << intChannels[i] << endl; 
            intPorts.push_back(new BufferedPort<ImageOf<PixelMono> >);
            ss.str(""); ss.clear();		
            ss << intChannels[i];
            intPorts[i]->open(("/"+name+"/int/"+ss.str()+"/img:o").c_str());
        }
        cout << endl;
        
        return true;
    }

    void afterStart(bool s)
    {
        if (s)
            {
                fprintf(stdout,"Process started successfully\n");
                fprintf(stdout,"\n");
                fprintf(stdout,"Using ...\n");
                fprintf(stdout,"name           = %s\n",name.c_str());
                fprintf(stdout,"num channels   = %d\n",numChannels);
                fprintf(stdout,"\n");
            }
        else
            fprintf(stdout,"Process did not start\n");
    }
    
    virtual void run()
    {
        while (!isStopping())
            {
                // acquire new image
                if (ImageOf<PixelBgr> *pImgIn=inPort.read(false))
                    {         
                        double latch_t, dt0, dt1;
                        double t0=Time::now();
                        
                        int lo, hi;
                        
                        // get the pointer to the output
                        ImageOf<PixelMono> &imgOut=outPort.prepare();
                        imgOut.resize(*pImgIn);
                        
                        // consistency check
                        if (pImgIn->width()!=width ||
                            pImgIn->height()!=height)
                            {  
                                hsv = cvCreateImage(cvGetSize(pImgIn->getIplImage()), IPL_DEPTH_8U, 3);
                                saturation = cvCreateImage(cvGetSize(pImgIn->getIplImage()), IPL_DEPTH_8U, 1);
                                intensity = cvCreateImage(cvGetSize(pImgIn->getIplImage()), IPL_DEPTH_8U, 1);
                                hue = cvCreateImage(cvGetSize(pImgIn->getIplImage()), IPL_DEPTH_8U, 1);
                                
                                width = pImgIn->width();
                                height = pImgIn->height();
                            }
                        
                        // convert image to HSV and split out 3 channels
                        cvCvtColor(pImgIn->getIplImage(),hsv,CV_BGR2HSV);
                        cvSplit(hsv,hue,saturation,intensity,0);
                        
                        // default is always high saturation channel
                        cvInRangeS(saturation, cvScalar(200.0), cvScalar(255.0), imgOut.getIplImage());                
                        
                        // blur image and discretize it
                        cvSmooth(imgOut.getIplImage(), imgOut.getIplImage(), CV_BLUR, 7, 7);
                        cvThreshold( imgOut.getIplImage(), imgOut.getIplImage(), 50, 255, CV_THRESH_BINARY );
                        
                        // send images over YARP
                        outPort.write();     	               
                        
                        // go through each channel that we are running 
                        for(int i=0; i<hueChannels.size(); i++) {
                            ImageOf<PixelMono> &channelImgOut = huePorts[i]->prepare();     
                            channelImgOut.resize(*pImgIn);
                            
                            lo = ((255.0/numChannels)*hueChannels[i])+1;
                            if(lo==1) lo=0;
                            hi = (255.0/numChannels)*(hueChannels[i]+1);
                            
                            //cout << "hue channel[" << hueChannels[i] << "]: (" << lo << "," << hi << ")" << endl;
                            cvInRangeS(hue, cvScalar(lo), cvScalar(hi), channelImgOut.getIplImage());                
                            cvSmooth(channelImgOut.getIplImage(), channelImgOut.getIplImage(), CV_BLUR, 7, 7);
                            cvThreshold( channelImgOut.getIplImage(), channelImgOut.getIplImage(), 50, 255, CV_THRESH_BINARY );
                            
                            huePorts[i]->write();     
                        }
                        for(int i=0; i<satChannels.size(); i++) {
                            ImageOf<PixelMono> &channelImgOut = satPorts[i]->prepare();     
                            channelImgOut.resize(*pImgIn);
                            
                            lo = ((255.0/numChannels)*satChannels[i])+1;
                            if(lo==1) lo=0;
                            hi = (255.0/numChannels)*(satChannels[i]+1);
                            
                            //cout << "saturation channel[" << satChannels[i] << "]: (" << lo << "," << hi << ")" << endl;
                            cvInRangeS(saturation, cvScalar(lo), cvScalar(hi), channelImgOut.getIplImage());                
                            cvSmooth(channelImgOut.getIplImage(), channelImgOut.getIplImage(), CV_BLUR, 7, 7);
                            cvThreshold( channelImgOut.getIplImage(), channelImgOut.getIplImage(), 50, 255, CV_THRESH_BINARY );
                            
                            satPorts[i]->write();     
                        }
                        for(int i=0; i<intChannels.size(); i++) {
                            ImageOf<PixelMono> &channelImgOut = intPorts[i]->prepare();     
                            channelImgOut.resize(*pImgIn);
                            
                            lo = ((255.0/numChannels)*intChannels[i])+1;
                            if(lo==1) lo=0;
                            hi = (255.0/numChannels)*(intChannels[i]+1);
                            
                            //cout << "intensity channel[" << intChannels[i] << "]: (" << lo << "," << hi << ")" << endl;
                            cvInRangeS(intensity, cvScalar(lo), cvScalar(hi), channelImgOut.getIplImage());                
                            cvSmooth(channelImgOut.getIplImage(), channelImgOut.getIplImage(), CV_BLUR, 7, 7);
                            cvThreshold( channelImgOut.getIplImage(), channelImgOut.getIplImage(), 25, 255, CV_THRESH_BINARY );
                            
                            intPorts[i]->write();     
                        }
                        
                    }
            }
    }
    
    virtual void threadRelease()
    {
        inPort.close();
        outPort.close();
        
        for(int i=0; i<huePorts.size(); i++) {
            huePorts[i]->close();
            delete huePorts[i];
        }
        huePorts.clear();
        
        for(int i=0; i<satPorts.size(); i++) {
            satPorts[i]->close();
            delete satPorts[i];
        }
        satPorts.clear();
        
        for(int i=0; i<intPorts.size(); i++) {
            intPorts[i]->close();
            delete intPorts[i];
        }
        huePorts.clear();
        
    }

    void interrupt()
    {
        inPort.interrupt();
        outPort.interrupt();
        for(int i=0; i<huePorts.size(); i++) {
            huePorts[i]->interrupt();
        }
        for(int i=0; i<satPorts.size(); i++) {
            satPorts[i]->interrupt();
        }
        for(int i=0; i<intPorts.size(); i++) {
            intPorts[i]->interrupt();
        }    
    }
};


class ProcessModule: public RFModule
{
private:
    ProcessThread *thr;
    
public:
    ProcessModule() : thr(NULL) { }
    
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();
        
        thr=new ProcessThread(rf);
        if (thr->start())
            return true;
        else
        {
            delete thr;    
            return false;
        }
    }
    
    virtual bool close()
    {
        if (thr)
        {
            thr->interrupt();
            thr->stop();
            delete thr;
        }

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
    rf.configure("ICUB_ROOT",argc,argv);

    ProcessModule mod;

    return mod.runModule(rf);
}


