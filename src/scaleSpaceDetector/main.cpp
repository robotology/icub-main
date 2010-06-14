// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
@ingroup icub_tools

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
- <i> /<stemName>/img:i </i> accepts the incoming images (should be color). 
- <i> /<stemName>/<type>/img:o </i> outputs the channel for the specified type
 
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


class ProcessThread : public Thread
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

    bool threshold;

public:
    ProcessThread(ResourceFinder &_rf) : rf(_rf) { }
    
    virtual bool threadInit()
    {
        
        //vector<int>::iterator p;
        string str;
        //        stringstream ss;
        //int v;
        
        scales.clear();

        name=rf.check("name",Value("scaleSpaceDetector")).asString().c_str();
        type=rf.check("type",Value("corner")).asString().c_str();
        str=rf.check("threshold",Value("on")).asString().c_str();
        scaleBottle=rf.findGroup("scales", "scales");

        if(str=="off") {
            threshold=false;
        } else {
            threshold=true;
        }

        if(scaleBottle.size() == 1) {
            scales.push_back(4);
            scales.push_back(8);
        } else {
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
            addFilters(scales[n]);            
        }
        cout << "]" << endl;

        return true;
    }
    
    void afterStart(bool s)
    {
        if (s) {
            fprintf(stdout,"Process started successfully\n");
            fprintf(stdout,"\n");
            fprintf(stdout,"Using ...\n");
            fprintf(stdout,"name   = %s\n",name.c_str());
            fprintf(stdout,"type   = %s\n",type.c_str());
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

        while (!isStopping()) {
            
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
                
                if(threshold) {
                    cvThreshold(LNormalized.getIplImage(), LNormalized.getIplImage(), 100, 255, CV_THRESH_BINARY );
                }

                // send image over YARP
                imgOut = LNormalized;
                outPort.write();
                                    
            }
        
        }
        
        // clear the images
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

        
    void addFilters(int scale) 
    {
            
        float sigma = sqrt((float)scale);
        int s = ceil(5*sigma)+1;
        
        float sigma2 = sigma*sigma;
        float sigma4 = sigma2*sigma2;
        float sigma6 = sigma2*sigma4;
        float sigma8 = sigma4*sigma4;
        
        float x[s*s];
        float y[s*s];
        float x2[s*s];
        float y2[s*s];

        //g.push_back(new float[s*s]);
        gx.push_back(new float[s*s]);
        gy.push_back(new float[s*s]);
        gxx.push_back(new float[s*s]);
        gyy.push_back(new float[s*s]);
        gxy.push_back(new float[s*s]);
        /*
        gxxx.push_back(new float[s*s]);
        gyyy.push_back(new float[s*s]);
        gxyy.push_back(new float[s*s]);
        gxxy.push_back(new float[s*s]);
        */

        //G.push_back(new CvMat);
        Gx.push_back(new CvMat);
        Gy.push_back(new CvMat);
        Gxx.push_back(new CvMat);
        Gyy.push_back(new CvMat);
        Gxy.push_back(new CvMat);
        /*
        Gxxx.push_back(new CvMat);
        Gyyy.push_back(new CvMat);
        Gxxy.push_back(new CvMat);
        Gxyy.push_back(new CvMat);
        */
        int n = gx.size() - 1;        
        float alpha;
        int c = 0;

        for(int i=0; i<s; i++) {
            for(int j=0; j<s; j++) {
                
                x[c] = (j)-((s-1)/2);
                y[c] = (i)-((s-1)/2);
                x2[c] = x[c]*x[c];
                y2[c] = y[c]*y[c];

                alpha = exp(-(x2[c] + y2[c])/(2.0*sigma2));
                
                //g[n][c]    = (1.0/(2.0*M_PI*sigma2))             * alpha;
                gx[n][c]   = (-x[c]/(2.0*M_PI*sigma4))           * alpha;
                gy[n][c]   = (-y[c]/(2.0*M_PI*sigma4))           * alpha;
                gxx[n][c]  = (1.0/(2.0*M_PI*sigma6))             * alpha*(x2[c]-sigma2);
                gyy[n][c]  = (1.0/(2.0*M_PI*sigma6))             * alpha*(y2[c]-sigma2);
                gxy[n][c]  = ((x[c]*y[c])/(2.0*M_PI*sigma6))     * alpha;
                //gxxx[n][c] = (-x[c]/(2.0*M_PI*sigma8))           * alpha*(x2[c]-3.0*sigma2);
                //gyyy[n][c] = (-y[c]/(2.0*M_PI*sigma8))           * alpha*(y2[c]-3.0*sigma2);
                //gxxy[n][c] = (-y[c]/(2.0*M_PI*sigma8))           * alpha*(x2[c]-sigma2);
                //gxyy[n][c] = (-x[c]/(2.0*M_PI*sigma8))           * alpha*(y2[c]-sigma2);
                
                c++;
                
            }
        }


        //G[n]    = cvCreateMatHeader( s, s, CV_32FC1 );
        Gx[n]   = cvCreateMatHeader( s, s, CV_32FC1 );
        Gy[n]   = cvCreateMatHeader( s, s, CV_32FC1 );
        Gxx[n]  = cvCreateMatHeader( s, s, CV_32FC1 );
        Gyy[n]  = cvCreateMatHeader( s, s, CV_32FC1 );
        Gxy[n]  = cvCreateMatHeader( s, s, CV_32FC1 );
        //Gxxx[n] = cvCreateMatHeader( s, s, CV_32FC1 );
        //Gyyy[n] = cvCreateMatHeader( s, s, CV_32FC1 );
        //Gxxy[n] = cvCreateMatHeader( s, s, CV_32FC1 );
        //Gxyy[n] = cvCreateMatHeader( s, s, CV_32FC1 );

        //cvSetData(G[n],    g[n],    G[n]->step);
        cvSetData(Gx[n],   gx[n],   Gx[n]->step);
        cvSetData(Gy[n],   gy[n],   Gy[n]->step);
        cvSetData(Gxx[n],  gxx[n],  Gxx[n]->step);
        cvSetData(Gyy[n],  gyy[n],  Gyy[n]->step);
        cvSetData(Gxy[n],  gxy[n],  Gxy[n]->step);
        //cvSetData(Gxxx[n], gxxx[n], Gxxx[n]->step);
        //cvSetData(Gyyy[n], gyyy[n], Gyyy[n]->step);
        //cvSetData(Gxxy[n], gxxy[n], Gxxy[n]->step);
        //cvSetData(Gxyy[n], gxyy[n], Gxyy[n]->step);

    }
    
    
    virtual void threadRelease() {

        inPort.close();
        outPort.close();
        scales.clear();        

        for(int i=0; i < Gx.size(); i++) {
            //delete G[i];
            delete Gx[i];
            delete Gy[i];
            delete Gxx[i];
            delete Gyy[i];
            delete Gxy[i];
            //delete Gxxx[i];
            //delete Gyyy[i];
            //delete Gxxy[i];
            // delete Gxyy[i];
        }
        //G.clear();
        Gx.clear();
        Gy.clear();
        Gxx.clear();
        Gyy.clear();
        Gxy.clear();
        //Gxxx.clear();
        //Gyyy.clear();
        //Gxxy.clear();
        //Gxyy.clear();
    }

    void interrupt()
    {
        inPort.interrupt();
        outPort.interrupt();
        scales.clear();
        for(int i=0; i < Gx.size(); i++) {
            //delete G[i];
            delete Gx[i];
            delete Gy[i];
            delete Gxx[i];
            delete Gyy[i];
            delete Gxy[i];
            //delete Gxxx[i];
            //delete Gyyy[i];
            //delete Gxxy[i];
            //delete Gxyy[i];
        }
        //G.clear();
        Gx.clear();
        Gy.clear();
        Gxx.clear();
        Gyy.clear();
        Gxy.clear();
        //Gxxx.clear();
        // Gyyy.clear();
        //Gxxy.clear();
        //Gxyy.clear();        
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


