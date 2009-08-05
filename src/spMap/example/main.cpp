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
#include "iCub/spherical_projection.h"


using namespace yarp::os;
using namespace yarp::sig;

class sp_mod : public Module {

private : 
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;

    int _input_lines;
    int _input_cols;
    int _output_lines;
    int _output_cols;
    double _fx;            
    double _fy;           
    double _cx;
    double _cy;
    double _k1;
    double _k2;
    double _p1;
    double _p2;

    IplImage *_mapx;
    IplImage *_mapy;

public:

    sp_mod(){};
    ~sp_mod(){};
    
    virtual bool open(Searchable& config)
    {
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }

        // Defaults will correspond to a view field of 90 deg.
        _input_lines = config.check("h", 480, "Input image lines").asInt();
        _input_cols = config.check("w", 640, "Input image columns").asInt();
        _fx = config.check("fx", 320, "Focal distance (on horizontal pixel size units)").asDouble();
        _fy = config.check("fy", 240, "Focal distance (on vertical pixel size units)").asDouble();
        _cx = config.check("cx", 320, "Image center (on horizontal pixel size units)").asDouble();
        _cy = config.check("cy", 240, "Image center (on vertical pixel size units)").asDouble();
        _k1 = config.check("k1", 0, "Radial distortion (first parameter)").asDouble();
        _k2 = config.check("k2", 0, "Radial distortion (second parameter)").asDouble();
        _p1 = config.check("p1", 0, "Tangential distortion (first parameter)").asDouble();
        _p2 = config.check("p2", 0, "Tangential distortion (second parameter)").asDouble();

        _output_lines = config.check("oh", 480, "Output image lines").asInt();
        _output_cols = config.check("ow", 640, "Output image columns").asInt();

        _mapx = cvCreateImage(cvSize(_output_cols, _output_lines), IPL_DEPTH_32F, 1);
        _mapy = cvCreateImage(cvSize(_output_cols, _output_lines), IPL_DEPTH_32F, 1);

        if(!compute_sp_map(_input_lines, _input_cols, _output_lines, _output_cols,
                            _fx, _fy, _cx, _cy, _k1, _k2, _p1, _p2, 
                            (float*)_mapx->imageData, (float*)_mapy->imageData))
            return false;


        portImgIn.open(getName("in"));
        portImgOut.open(getName("out"));

        return true;
    };

    virtual bool close()
    {
       portImgIn.close();
       portImgOut.close();
       cvReleaseImage(&_mapx);
       cvReleaseImage(&_mapy);
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
            return true;
        yarp::sig::ImageOf<PixelRgb> yrpImgOut; 
        yrpImgOut.resize(_output_cols, _output_lines);
        IplImage *inptr = (IplImage*)yrpImgIn->getIplImage();
	    IplImage *outptr = (IplImage*)yrpImgOut.getIplImage();
        //cvRemap(inptr,outptr,_mapx,_mapy,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        cvRemap(inptr,outptr,_mapx,_mapy,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        yarp::sig::ImageOf<PixelRgb>& yrpOut = portImgOut.prepare();
        yrpOut = yrpImgOut;
        portImgOut.write();
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    sp_mod module;
    module.setName("/sphericalmap"); // set default name of module
    return module.runModule(argc,argv);
}

