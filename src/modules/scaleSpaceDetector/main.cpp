// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
@ingroup icub_module

\defgroup scaleSpaceDetector scaleSpaceDetector
 
Provides FG images detecting the BLOBS, EDGES, CORNERS, or EDGES

Copyright (C) 2010 RobotCub Consortium
 
Authors: Stephen Hart 
 
Date: June 10, 2010

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module exploits provides binary images corresponding to specified SCALESPACE channels as per Tony Lindeberg's "Scale-Space Theory"
 
\section lib_sec Libraries 
YARP libraries and OpenCV

\section parameters_sec Parameters
--name \e stemName 
- The parameter \e stemName specifies the stem name of ports 
  created by the module. By default \e scaleSpaceDetector is
  used

--type \e [blob | ridge | corner | edge] 
- The parameter \e type specifies which type of feature will be processed
   by the module. By default \e edge is used

--scales \e 
- The parameter \e scales specifies which scales will be searched over.

--threshold \e 
- The parameter \e threshold specifies whether the output image should be thresholded

\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created

- \e /<stemName>/img:i </i> accepts the incoming images (should be color). 

- \e /<stemName>/<type>/img:o </i> outputs the channel for the specified type

- \e /<stemName>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
    -'susp' suspend the module
    -'run' resume the module
    -'add <scale>' adds a scale the list to process images with 
    -'remove <scale>' removes the scale specified from the list 
    -'list' lists the current scales being processed

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
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <yarp/sig/Matrix.h>
#include <algorithm>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class ProcessThread : public RateThread 
{
private:
    ResourceFinder &rf;
    
    string name;
    string type;

    int width;
    int height;
   
    IplImage *gImg8;
    IplImage *gImg32;

    IplImage *Lx;
    IplImage *Ly;
    IplImage *Lxx;
    IplImage *Lyy;
    IplImage *Lxy;

    IplImage *Lx2;
    IplImage *Ly2;
    IplImage *LxLy;
    IplImage *Ltmp1;
    IplImage *Ltmp2;
    IplImage *Ltmp3;

    IplImage *Lblob;
    IplImage *Ledge;
    IplImage *Lridge;
    IplImage *Lcorner;
    IplImage *LMax;

    ImageOf<PixelMono> LNormalized;

    vector<int> scales;

    Bottle scaleBottle;
	
    ImageOf<PixelMono> imgFilteredIn;

    BufferedPort<ImageOf<PixelBgr> >  inPort;
    BufferedPort<ImageOf<PixelMono> >  outPort;
    vector< BufferedPort<ImageOf<PixelMono> > * >  scalePorts;

    vector<CvMat *> Gx;
    vector<CvMat *> Gy;
    vector<CvMat *> Gxx;
    vector<CvMat *> Gyy;
    vector<CvMat *> Gxy;

    vector<float *> gx;
    vector<float *> gy;
    vector<float *> gxx;
    vector<float *> gyy;
    vector<float *> gxy;

    bool thresholdOn;
    int threshold;

public:

    static const int THRESH_VAL = 100;

    ProcessThread(ResourceFinder &_rf) : 
        RateThread(10), 
        rf(_rf) { }
    
    virtual bool threadInit()
    {
        
        string str;
        
        scales.clear();
        threshold = THRESH_VAL;

        name=rf.check("name",Value("scaleSpaceDetector")).asString().c_str();
        type=rf.check("type",Value("corner")).asString().c_str();
        str=rf.check("threshold",Value("off")).asString().c_str();
        thresholdOn=rf.check("threshold",Value("on")).asString().c_str();
        threshold=rf.check("value",Value("100")).asInt();
        scaleBottle=rf.findGroup("scales", "scales");

        if(str=="off") {
            thresholdOn=false;
        } else {
            thresholdOn=true;
        }

        if(scaleBottle.size() == 1) {
            scales.push_back(4);
            scales.push_back(8);
        } else {
            scales.clear();
            for(int k=1; k<scaleBottle.size(); k++) {
               scales.push_back(scaleBottle.get(k).asInt());
            }
        }
        
        if( (type!="blob") &&
            (type!="edge") &&
            (type!="corner") &&
            (type!="ridge") ) {
            cout << "scaleSpaceDetector -- unknown type: " << type.c_str() << ", must be [edge | blob | ridge | corner]" << endl;
            return false;
        }
        
        inPort.open(("/"+name+"/img:i").c_str());
        outPort.open(("/"+name+"/"+type+"/img:o").c_str());

        sort(scales.begin(),scales.end());
        
        width = 0;   height = 0;

        cout << "scales: [ ";
        for(int n=0; n<scales.size(); n++) {
            cout << scales[n] << " ";
            createFilters(scales[n]);            
        }
        cout << "]" << endl;

        return true;
    }
    
    string getName() { return name; }

    void afterStart(bool s)
    {
        if (s) {
            fprintf(stdout,"Process started successfully\n");
            fprintf(stdout,"\n");
            fprintf(stdout,"Using ...\n");
            fprintf(stdout,"name   = %s\n",name.c_str());
            fprintf(stdout,"type   = %s\n",type.c_str());
            fprintf(stdout,"thresh = %d, on=%d\n",threshold, thresholdOn);
            fprintf(stdout,"\n");
        } else {
            fprintf(stdout,"Process did not start\n");
        }
    }
    
    virtual void run()
    {

        double latch_t, dt0, dt1, t0;
        int lo, hi;
        
        int scale_offset;
        int y_min, x_min;
        int y_max, x_max;
        int my,mx;

        CvScalar v;
        int channels;
        int step;
        double d;

        while (!isSuspended()) {

            // acquire new image
            if (ImageOf<PixelBgr> *pImgIn=inPort.read(false)) {
                
                t0=Time::now();
                        
                // get the pointer to the output
                ImageOf<PixelMono> &imgOut=outPort.prepare();
                imgOut.resize(*pImgIn);
                
                // consistency check
                if (pImgIn->width()!=width || pImgIn->height()!=height) {  

                    width = pImgIn->width();
                    height = pImgIn->height();
                    
                    gImg8 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
                    gImg32 = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    
                    Lx = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Ly = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Lxx = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Lyy = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Lxy = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    
                    Lx2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Ly2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    LxLy = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Ltmp1 = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Ltmp2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Ltmp3 = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);

                    Lblob = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Ledge = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Lridge = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    Lcorner = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    LMax = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
                    
                    LNormalized.resize(*pImgIn);
                } 
                
                LNormalized.zero();
                cvSet(LMax,cvScalar(0));
                
                // convert image to gray scale
                cvCvtColor(pImgIn->getIplImage(),gImg8,CV_BGR2GRAY);
                channels = gImg8->nChannels;
                step = gImg8->widthStep;                        
                
                // convert the image to 32F from 8U
                for(int j=0; j<height; j++) {
                    for(int i=0; i<width; i++) {
                        cvSet2D(gImg32,j,i,cvGet2D(gImg8,j,i));
                    }
                }
                
                for(int n=0; n<scales.size(); n++) {
                    
                    scale_offset = ceil(sqrt(scales[n]))*5;

                    y_min = floor(scale_offset/2);
                    x_min = floor(scale_offset/2);                            
                    
                    y_max = height-scale_offset;
                    x_max = width-scale_offset;
                    
                    if(type == "blob") {
                        
                        //Ledge = (Lxx + Lyy);
                        cvFilter2D(gImg32, Lxx, Gxx[n]);
                        cvFilter2D(gImg32, Lyy, Gyy[n]);                       
                        cvAdd(Lxx, Lyy, Lblob);

                        for(int j=0; j<y_max; j++) {
                            for(int i=0; i<x_max; i++) {

                                v = cvGet2D(Lblob,j,i);
                                d = v.val[0]*scales[n];
                                        
                                my = j+y_min;
                                mx = i+x_min;
                                        
                                v = cvGet2D(LMax,my,mx);
                                if(d > v.val[0]) {
                                    cvSet2D(LMax,my,mx,cvScalar(d));
                                } 
                            }
                        }
                                
                    } else if(type == "edge") {

                        //Ledge = (Lx.^2 + Ly.^2).^0.5;
                        cvFilter2D(gImg32, Lx, Gx[n]);
                        cvFilter2D(gImg32, Ly, Gy[n]);
                        
                        cvMul(Lx,Lx,Lx2);
                        cvMul(Ly,Ly,Ly2);                    
                        cvAdd(Lx2, Ly2, Ledge);
                        
                        for(int j=0; j<y_max; j++) {
                            for(int i=0; i<x_max; i++) {
                                
                                v = cvGet2D(Ledge,j,i);
                                d = sqrt(v.val[0])*pow(scales[n],0.25);
                                cvSet2D(Ledge,j,i,cvScalar(d));

                                my = j+y_min;
                                mx = i+x_min;

                                v = cvGet2D(LMax,my,mx);
                                if(d > v.val[0]) {
                                    cvSet2D(LMax,my,mx,cvScalar(d));
                                } 
                            }
                        }                                                                                  

                    } else if(type == "corner") {
                        
                        //Lcorner = Lx.*Lx.*Lyy + Ly.*Ly.*Lxx - 2*Lx.*Ly.*Lxy;
                        cvFilter2D(gImg32, Lx, Gx[n]);
                        cvFilter2D(gImg32, Ly, Gy[n]);
                        cvFilter2D(gImg32, Lxx, Gxx[n]);
                        cvFilter2D(gImg32, Lyy, Gyy[n]);
                        cvFilter2D(gImg32, Lxy, Gxy[n]);

                        cvMul(Lx,Lx,Lx2);
                        cvMul(Ly,Ly,Ly2);      
                        cvMul(Lx,Ly,LxLy);      
                          
                        cvMul(Lx2,  Lyy, Ltmp1);
                        cvMul(Ly2,  Lxx, Ltmp2);
                        cvMul(LxLy, Lxy, Ltmp3);
                        cvScale(Ltmp3, Ltmp3, -2.0);

                        cvAdd(Ltmp1, Ltmp2, Ltmp1);
                        cvAdd(Ltmp1, Ltmp3, Lcorner);

                        for(int j=0; j<y_max; j++) {
                            for(int i=0; i<x_max; i++) {
                                
                                v = cvGet2D(Lcorner,j,i);
                                d = v.val[0]*pow(scales[n],2.0);
                                cvSet2D(Lcorner,j,i,cvScalar(d));

                                my = j+y_min;
                                mx = i+x_min;

                                v = cvGet2D(LMax,my,mx);
                                if(d > v.val[0]) {
                                    cvSet2D(LMax,my,mx,cvScalar(d));
                                } 
                            }
                        }                                                           

                    } else if(type == "ridge") {

                        // Lridge = ((Lxx - Lyy).^2 + 4*Lxy.^2);
                        cvFilter2D(gImg32, Lxx, Gxx[n]);
                        cvFilter2D(gImg32, Lyy, Gyy[n]);
                        cvFilter2D(gImg32, Lxy, Gxy[n]);

                        cvScale(Lyy,Lyy,-1.0);
                        cvAdd(Lxx,Lyy,Ltmp1);
                        cvMul(Ltmp1,Ltmp1,Ltmp2);

                        cvMul(Lxy,Lxy,Ltmp1);
                        cvScale(Ltmp1,Ltmp1,4.0);
                        cvAdd(Ltmp2, Ltmp1, Lridge);

                        for(int j=0; j<y_max; j++) {
                            for(int i=0; i<x_max; i++) {
                                
                                v = cvGet2D(Lridge,j,i);
                                d = v.val[0]*pow(scales[n],1.5);
                                cvSet2D(Lridge,j,i,cvScalar(d));

                                my = j+y_min;
                                mx = i+x_min;

                                v = cvGet2D(LMax,my,mx);
                                if(d > v.val[0]) {
                                    cvSet2D(LMax,my,mx,cvScalar(d));
                                } 
                            }
                        }                                                           
                    }
                
                }    

                cvNormalize(LMax, LNormalized.getIplImage(), 0, 255, CV_MINMAX);       
                
                if(thresholdOn) {
                    cvThreshold(LNormalized.getIplImage(), LNormalized.getIplImage(), threshold, 255, CV_THRESH_BINARY );
                }

                // send image over YARP
                imgOut = LNormalized;
                outPort.write();
                                    
            }
        
        }
                
    }

        
    void createFilters(int scale, int pos=-1) 
    {
            
        float sigma = sqrt((float)scale);
        int s = ceil(5*sigma)+1;
        int n;

        float sigma2 = sigma*sigma;
        float sigma4 = sigma2*sigma2;
        float sigma6 = sigma2*sigma4;
        float sigma8 = sigma4*sigma4;
        
        float x[s*s];
        float y[s*s];
        float x2[s*s];
        float y2[s*s];

        float alpha;
        int c = 0;

        // add filter to the end of the list
        if(pos == -1) {

            gx.push_back(new float[s*s]);
            gy.push_back(new float[s*s]);
            gxx.push_back(new float[s*s]);
            gyy.push_back(new float[s*s]);
            gxy.push_back(new float[s*s]);

            Gx.push_back(new CvMat);
            Gy.push_back(new CvMat);
            Gxx.push_back(new CvMat);
            Gyy.push_back(new CvMat);
            Gxy.push_back(new CvMat);
            n = gx.size() - 1;        

        } else {
            
            gx.insert(gx.begin()+pos, new float[s*s]);
            gy.insert(gy.begin()+pos, new float[s*s]);
            gxx.insert(gxx.begin()+pos, new float[s*s]);
            gyy.insert(gyy.begin()+pos, new float[s*s]);
            gxy.insert(gxy.begin()+pos, new float[s*s]);

            Gx.insert(Gx.begin()+pos, new CvMat);
            Gy.insert(Gy.begin()+pos, new CvMat);
            Gxx.insert(Gxx.begin()+pos, new CvMat);
            Gyy.insert(Gyy.begin()+pos, new CvMat);
            Gxy.insert(Gxy.begin()+pos, new CvMat);

            n = pos;
        }

        for(int i=0; i<s; i++) {
            for(int j=0; j<s; j++) {
                
                x[c] = (j)-((s-1)/2);
                y[c] = (i)-((s-1)/2);
                x2[c] = x[c]*x[c];
                y2[c] = y[c]*y[c];

                alpha = exp(-(x2[c] + y2[c])/(2.0*sigma2));
                
                gx[n][c]   = (-x[c]/(2.0*M_PI*sigma4))           * alpha;
                gy[n][c]   = (-y[c]/(2.0*M_PI*sigma4))           * alpha;
                gxx[n][c]  = (1.0/(2.0*M_PI*sigma6))             * alpha*(x2[c]-sigma2);
                gyy[n][c]  = (1.0/(2.0*M_PI*sigma6))             * alpha*(y2[c]-sigma2);
                gxy[n][c]  = ((x[c]*y[c])/(2.0*M_PI*sigma6))     * alpha;

                c++;                
            }
        }

        Gx[n]   = cvCreateMatHeader( s, s, CV_32FC1 );
        Gy[n]   = cvCreateMatHeader( s, s, CV_32FC1 );
        Gxx[n]  = cvCreateMatHeader( s, s, CV_32FC1 );
        Gyy[n]  = cvCreateMatHeader( s, s, CV_32FC1 );
        Gxy[n]  = cvCreateMatHeader( s, s, CV_32FC1 );

        cvSetData(Gx[n],   gx[n],   Gx[n]->step);
        cvSetData(Gy[n],   gy[n],   Gy[n]->step);
        cvSetData(Gxx[n],  gxx[n],  Gxx[n]->step);
        cvSetData(Gyy[n],  gyy[n],  Gyy[n]->step);
        cvSetData(Gxy[n],  gxy[n],  Gxy[n]->step);

    }

    void removeFilter(int pos) {

        if(pos >= scales.size()) return;

        scales.erase(scales.begin() + pos);

        gx.erase(gx.begin() + pos);
        gy.erase(gy.begin() + pos);
        gxx.erase(gxx.begin() + pos);
        gyy.erase(gyy.begin() + pos);
        gxy.erase(gxy.begin() + pos);

        Gx.erase(Gx.begin() + pos);
        Gy.erase(Gy.begin() + pos);
        Gxx.erase(Gxx.begin() + pos);
        Gyy.erase(Gyy.begin() + pos);
        Gxy.erase(Gxy.begin() + pos);

    }

    
    bool addScale(int scale) {

        int pos;
        for(int i=0; i<scales.size(); i++) {
            if(scales[i] == scale) {
                cout << "scale " << scale << " already in list" << endl;
                return true;
            }
        }
        scales.push_back(scale);
        sort(scales.begin(),scales.end());
        for(int i=0; i<scales.size(); i++) {
            if(scales[i] == scale) {
                createFilters(scale, i);
                break;
            }

        }

        return true;
            
    }

    bool removeScale(int scale) {
        
        int pos = -1;
        for(int i=0; i<scales.size(); i++) {
            if(scales[i] == scale) {
                pos = i;
                cout << "scale " << scale << " in list at position[" << i << "]" << endl;
                break;
            }
        }

        if(pos==-1) {
            cout << "scale to remove not found" << endl;
            return true;
        }

        removeFilter(pos);

        return true;

    }
 
    void deleteFilters() {
        cout << "deleting filters..." << endl;
        for(int i=0; i < Gx.size(); i++) {
            delete Gx[i];
            delete Gy[i];
            delete Gxx[i];
            delete Gyy[i];
            delete Gxy[i];
        }
        Gx.clear();
        Gy.clear();
        Gxx.clear();
        Gyy.clear();
        Gxy.clear();
    }

    void releaseImages() {       
        cout << "releasing images store" << endl;
        cvReleaseImage(&gImg8);
        cvReleaseImage(&gImg32);
        cvReleaseImage(&Lx);
        cvReleaseImage(&Ly);
        cvReleaseImage(&Lxx);
        cvReleaseImage(&Lyy);
        cvReleaseImage(&Lxy);
        cvReleaseImage(&Lblob);
        cvReleaseImage(&Ledge);
        cvReleaseImage(&Lridge);
        cvReleaseImage(&Lcorner);
        cvReleaseImage(&LMax);
    }

    virtual void threadRelease() {
        inPort.close();
        outPort.close();
        scales.clear();        
        deleteFilters();
        releaseImages();
    }

    void interrupt()
    {
        inPort.interrupt();
        outPort.interrupt();
    }

    void listScales() {
        cout << "( ";
        for(int i=0; i<scales.size(); i++) {
            cout << scales[i] << " ";
        }
        cout << ")" << endl;;
    }

    void setThreshold(bool b, int v=100) {
        thresholdOn = b;
        threshold = v;        
        cout << "setting threshold: " << b << ", val=" << v << endl;
    }

};


class ProcessModule: public RFModule
{
private:
    ProcessThread *thr;
    Port rpcPort;

public:
    ProcessModule() : thr(NULL) { }
    
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();
        string rpcPortName;

        thr=new ProcessThread(rf);

        if (thr->start()) {
            rpcPortName = "/" + thr->getName() + "/rpc";
            rpcPort.open(rpcPortName.c_str());
            attach(rpcPort);
            return true;
        } else {
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

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        cout << "Receiving command from rpc port" << endl;
        int new_scale, scale_to_delete;
        int v;
        bool b;

        if (command.size())
        {
            switch (command.get(0).asVocab())
            {                
                case VOCAB4('s','u','s','p'):
                {                    
                    thr->suspend();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
                case VOCAB3('r','u','n'):
                {                    
                    thr->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
                case VOCAB3('a','d','d'):
                {
                    new_scale = command.get(1).asInt();
                    thr->suspend();
                    thr->addScale(new_scale);
                    thr->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
            case VOCAB4('r','e','m','o'):
                {
                    scale_to_delete = command.get(1).asInt();
                    thr->suspend();
                    thr->removeScale(scale_to_delete);
                    thr->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
            case VOCAB4('l','i','s','t'):
                {
                    thr->suspend();
                    thr->listScales();
                    thr->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
            case VOCAB4('t','h','r','e'):
                {
                    if(command.get(1).asString()=="on") {
                        b = true;
                    } else {
                        b = false;
                    }
                    if(command.size() == 3) {
                        v = command.get(2).asInt();
                    } else {
                        v = thr->THRESH_VAL;
                    }
                    thr->setThreshold(b,v);
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
        {
            reply.add("command size==0");
            return false;
        }
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


