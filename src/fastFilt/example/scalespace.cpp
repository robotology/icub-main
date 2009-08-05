// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>

//OpenCV
#include <cv.h>

//
#include "iCub/ScaleSpace.h"


using namespace yarp::os;
using namespace yarp::sig;

class scalespace : public Module {

private : 
    BufferedPort<ImageOf<PixelRgb> > portIn;
    BufferedPort<ImageOf<PixelMono> > *portOut;
    
    int lines;
    int cols;
    int levels;
    double *scales;
    ScaleSpace ss;
    IplImage *ingray;
    IplImage *infloat;
    IplImage *outfloat;
    IplImage *outgray;
    
public:

    scalespace()
    {
        infloat = 0; 
        ingray = 0;
        outgray = 0;
        outfloat = 0;
        scales = 0;
    };
    ~scalespace(){};
    
    virtual bool open(Searchable& config)
    {
        int i; 
        char portname[10];
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }

        cols  = config.check("width", 640, "Number of image columns").asInt();
        lines = config.check("height", 480, "Number of image lines").asInt();
        levels = config.check("levels", 4, "Number of scales in the scale space").asInt();

        scales = new double[levels];
        Bottle &xtmp = config.findGroup("scales");
	    if(xtmp.size() != levels+1)
            for (i = 0; i < levels; i++) 
                scales[i] = pow(2.0,i+1);
            
        for (i = 1; i < xtmp.size(); i++) 
            scales[i-1] = xtmp.get(i).asDouble();

        ss.AllocateResources(lines, cols, levels, scales);
        infloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        ingray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        outgray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        outfloat = cvCreateImageHeader(cvSize(cols,lines), IPL_DEPTH_32F, 1);
                
        portIn.open(getName("in"));

        portOut = new BufferedPort<ImageOf<PixelMono> >[levels];
        for( i = 1; i <= levels; i++ )
        {
            sprintf(portname, "out:%d", i);
            portOut[i-1].open(getName(portname));
        }
        return true;
    };

    virtual bool close()
    {
       portIn.close();
       delete scales;
       cvReleaseImage(&infloat);
       cvReleaseImage(&ingray);
       cvReleaseImageHeader(&outfloat);
       cvReleaseImage(&outgray);

       for(int i = 0; i < levels; i++)
          portOut[i].close();
       ss.FreeResources();
       return true;
    };
 
    virtual bool interruptModule()
    {

       portIn.interrupt();
       for(int i = 0; i < levels; i++)
          portOut[i].interrupt();
       
       return true;
    };

    virtual bool updateModule()
    {
        yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
        yrpImgIn = portIn.read();
        if (yrpImgIn == NULL)   
        {
            printf("No image received!");
            return true;
        }
        if (yrpImgIn->width() != cols)
        {
            printf("Mismatch on input width!");            
            return true;
        }
        if (yrpImgIn->height() != lines)
        {
            printf("Mismatch on input cols!");            
            return true;
        }            
   
        IplImage *inptr = (IplImage*)yrpImgIn->getIplImage();
        cvCvtColor( inptr, ingray, CV_BGR2GRAY );
        cvConvert(ingray, infloat);

        ss.BuildAll((float*)infloat->imageData);

        
        for(int i = 0; i < levels; i++ )
        {
            yarp::sig::ImageOf<PixelMono>& yrpout = portOut[i].prepare();
            yrpout.resize(cols,lines);
            IplImage *ptr = (IplImage*)yrpout.getIplImage();
            outfloat->imageData = (char*)ss.GetLevel(i);
            cvConvert(outfloat, ptr);
            portOut[i].write();
        }
        
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    scalespace module;
    module.setName("/scalespace"); // set default name of module
    return module.runModule(argc,argv);
}

