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
#include "iCub/FastGabor.h"


using namespace yarp::os;
using namespace yarp::sig;

class gaborfilt : public Module {

private : 
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelMono> > portRealOut;
    BufferedPort<ImageOf<PixelMono> > portImagOut;
    BufferedPort<ImageOf<PixelMono> > portAbsOut;

    int lines;
    int cols;
    double scale;
    double wavelength;
    double orientation;
    FastGabor FG;
    IplImage *infloat, *ingray, *realfloat, *realgray, 
        *imagfloat, *imaggray, *absfloat, *absgray, *gaussfloat;
    
public:

    gaborfilt()
    {
        infloat = 0; 
        ingray = 0;
        realfloat = 0;
        realgray = 0;
        imagfloat = 0;
        imaggray = 0;
        absfloat = 0;
        absgray = 0;
        gaussfloat = 0;
    };
    ~gaborfilt(){};
    
    virtual bool open(Searchable& config)
    {
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }

        // Defaults will correspond to a view field of 90 deg.
        scale = config.check("scale", 2, "Filter scale (gaussian standard deviation)").asInt();
        orientation = config.check("orientation", 0, "Filter orientation (degrees)").asInt();
        wavelength = config.check("wavelength", 4, "Filter wavelength (pixels)").asInt();
        cols  = config.check("w", 640, "Number of image columns").asInt();
        lines = config.check("h", 480, "Number of image lines").asInt();

        FG.AllocateResources(lines, cols, scale, orientation, wavelength);
        infloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        ingray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        realfloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        realgray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        imagfloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        imaggray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        absfloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);
        absgray = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_8U, 1);
        gaussfloat = cvCreateImage(cvSize(cols,lines), IPL_DEPTH_32F, 1);

        
        portImgIn.open(getName("in"));
        portRealOut.open(getName("out:real"));
        portImagOut.open(getName("out:imag"));
        portAbsOut.open(getName("out:abs"));

        return true;
    };

    virtual bool close()
    {
       portImgIn.close();
       portRealOut.close();
       portImagOut.close();
       portAbsOut.close();
       FG.FreeResources();
       if(infloat != 0)
           cvReleaseImage(&infloat);
       if(ingray != 0)
           cvReleaseImage(&ingray);
       if(realfloat != 0)
           cvReleaseImage(&realfloat);
       if(realgray != 0)
           cvReleaseImage(&realgray);
       if(imagfloat != 0)
           cvReleaseImage(&imagfloat);
       if(imaggray != 0)
           cvReleaseImage(&imaggray);
       if(absfloat != 0)
           cvReleaseImage(&absfloat);
       if(absgray != 0)
           cvReleaseImage(&absgray);
       if(gaussfloat != 0)
           cvReleaseImage(&gaussfloat);
       
       return true;
    };
 
    virtual bool interruptModule()
    {
       portImgIn.interrupt();
       portRealOut.interrupt();
       portImagOut.interrupt();
       portAbsOut.interrupt();
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

        FG.GaborFilt((float*)infloat->imageData, 
                     (float*)realfloat->imageData, 
                     (float*)imagfloat->imageData);

        FG.GaussFilt((float*)infloat->imageData, ((float*)gaussfloat->imageData));

        FG.RemoveDC((float*)realfloat->imageData, (float*)gaussfloat->imageData, (float*)realfloat->imageData);

        FG.ComputeMagnitude((float*)realfloat->imageData, (float*)imagfloat->imageData, (float*)absfloat->imageData);

        yarp::sig::ImageOf<PixelMono>& yrpReal = portRealOut.prepare();
        yarp::sig::ImageOf<PixelMono>& yrpImag = portImagOut.prepare();
        yarp::sig::ImageOf<PixelMono>& yrpAbs = portAbsOut.prepare();
        
        yrpReal.resize(cols, lines);
        yrpImag.resize(cols, lines);
        yrpAbs.resize(cols, lines);

	    IplImage *realptr = (IplImage*)yrpReal.getIplImage();
        IplImage *imagptr = (IplImage*)yrpImag.getIplImage();
        IplImage *absptr = (IplImage*)yrpAbs.getIplImage();
        
        //
        
        cvConvertScale(realfloat,realptr,4,128);
        cvConvertScale(imagfloat,imagptr,4,128);
        cvConvertScale(absfloat,absptr,16,0);

        portRealOut.write();
        portImagOut.write();
        portAbsOut.write();
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    gaborfilt module;
    module.setName("/gaborfilt"); // set default name of module
    return module.runModule(argc,argv);
}

