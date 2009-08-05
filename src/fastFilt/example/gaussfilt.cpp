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
#include "iCub/FastGauss.h"


using namespace yarp::os;
using namespace yarp::sig;

class gaussfilt : public Module {

private : 
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelMono> > portImgOut;

    int lines;
    int cols;
    double scale;
    FastGauss FG;
    IplImage *infloat, *ingray, *outfloat, *outgray;
    
public:

    gaussfilt(){infloat = 0; outfloat = 0; ingray = 0; outgray = 0;};
    ~gaussfilt(){};
    
    virtual bool open(Searchable& config)
    {
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }

        // Defaults will correspond to a view field of 90 deg.
        scale = config.check("scale", 4, "Filter scale (gaussian standard deviation)").asInt();
        cols  = config.check("w", 640, "Number of image columns").asInt();
        lines = config.check("h", 480, "Number of image lines").asInt();

        FG.AllocateResources(lines, cols, scale);
        infloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        outfloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        ingray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        outgray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        
        portImgIn.open(getName("in"));
        portImgOut.open(getName("out"));

        return true;
    };

    virtual bool close()
    {
       portImgIn.close();
       portImgOut.close();
       FG.FreeResources();
       if(infloat != 0)
           cvReleaseImage(&infloat);
       if(outfloat != 0)
           cvReleaseImage(&outfloat);
       if(ingray != 0)
           cvReleaseImage(&ingray);
       if(outgray != 0)
           cvReleaseImage(&outgray);
       return true;
    };
 
    virtual bool interruptModule()
    {
       portImgIn.interrupt();
       portImgOut.interrupt();
       return true;
    };

    virtual bool updateModule()
    {
        yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
        yrpImgIn = portImgIn.read();
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
        FG.GaussFilt((float*)infloat->imageData, (float*)outfloat->imageData);
        yarp::sig::ImageOf<PixelMono>& yrpOut = portImgOut.prepare();
        yrpOut.resize(cols, lines);
	    IplImage *outptr = (IplImage*)yrpOut.getIplImage();
        cvConvert(outfloat,outptr);
        portImgOut.write();
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    gaussfilt module;
    module.setName("/gaussfilt"); // set default name of module
    return module.runModule(argc,argv);
}

