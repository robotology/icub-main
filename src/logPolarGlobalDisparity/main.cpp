// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

//OpenCV
#include <cv.h>
#include <iostream>
#include <math.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

#include "CLogPolarSensor.h"
#include "LogPolarGlobalDisparity.h"

/**
 * @ingroup icub_module
 *
 * \defgroup icub_logpolarglobaldosparity logpolarglobaldisparity
 *
 * Calculates the globaldisparity from a pair of images by first converting them to logpolar.
 * The logpolar map has the effect of providing more weigth to the image regions in the
 * center of the visual field.
 *
 * \author Alex Bernardino
 *
 */



class LogPolarGlobalDisparity : public Module
{
private:
    int _input_lines;
    int _input_cols;
    
    int _lp_lines; 
    int _lp_cols;
    double _rho_min;
	double _log_shift;
	double _log_fact;
    double _sec_rad;

    CLogPolarSensor _lps;
    CLogPolarGlobalDisparity _lgd;
    int _ndisparities;
    int *_disparities; 
    IplImage *_grayleft, *_grayright, *_logleft, *_logright;


	BufferedPort< ImageOf<PixelRgb> > _leftImagePort;
    BufferedPort< ImageOf<PixelRgb> > _rightImagePort;
    BufferedPort< Vector > _disparityPort;
    BufferedPort< ImageOf<PixelRgb> > _blendedImagePort;
	
public:

    LogPolarGlobalDisparity()
    {
        _disparities = NULL;
        _grayleft = NULL;
        _grayright = NULL;
        _logleft = NULL;
        _logright = NULL;
    };
    ~LogPolarGlobalDisparity(){};
    
    virtual bool open(Searchable& config)
    {
         if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }

        // Defaults will correspond to a view field of 90 deg.
        _input_lines = config.check("h", 480, "Input image lines").asInt();
        _input_cols = config.check("w", 640, "Input image columns").asInt();
        _lp_lines = config.check("angles", 64, "Number of lines in the log-polar image (angles)").asInt();
        _lp_cols = config.check("eccentr", 32, "Number of columns in the log-polar image (eccentricities)").asInt();
        _rho_min = config.check("rho_min", 3, "Blind radius on the log-polar image)").asDouble();
        _log_shift = config.check("log_shift", 0, "Shift in the radial coordinates for the log-polar mapping").asDouble();
        Bottle &xtmp = config.findGroup("disparities","Disparity Hypothesis Values");
	    _ndisparities = xtmp.size()-1;
        if(_ndisparities <= 1)
            return false;
        _disparities = new int[_ndisparities];
        for (int i = 1; i < xtmp.size(); i++) 
            _disparities[i-1] = xtmp.get(i).asInt();

        _lps.compute_params_fixed_design(_lp_lines,_lp_cols,(double)_input_lines,(double)_input_cols,_log_shift,_rho_min,&_log_fact,&_sec_rad);
	    _lps.set_image_plane_params(_input_lines, _input_cols);
	    _lps.set_logpolar_params(_lp_lines, _lp_cols, _rho_min, _sec_rad, _log_fact, _log_shift);
	    _lps.create_logpolar_lut();


        _lgd.CreateLogPolarGlobalDisparity(_ndisparities, _disparities, &_lps);

        
        _leftImagePort.open(getName("left:i"));
        _rightImagePort.open(getName("right:i"));
        _disparityPort.open(getName("disp:o"));
        _blendedImagePort.open(getName("img:o"));
  
        _grayleft = cvCreateImage( cvSize(_input_cols,_input_lines), 8, 1 );
        _grayright = cvCreateImage(cvSize(_input_cols,_input_lines), 8, 1 );
        _logleft = cvCreateImage( cvSize(_lp_cols,_lp_lines), 8, 1 );
        _logright = cvCreateImage(cvSize(_lp_cols,_lp_lines), 8, 1 );


        
        return true;
    };

    virtual bool close()
    {
        _leftImagePort.close();
        _rightImagePort.close();
        _disparityPort.close();
        _blendedImagePort.close();

        _lgd.CloseLogPolarGlobalDisparity();


        if(_disparities != NULL)
            delete _disparities;
        _disparities = NULL;
       
        if(_grayleft != NULL)
            cvReleaseImage(&_grayleft);
        _grayleft = NULL;

        if(_grayright != NULL)
            cvReleaseImage(&_grayright);
        _grayright = NULL;

        if(_logleft != NULL)
            cvReleaseImage(&_logleft);
        _logleft = NULL;


        if(_logright != NULL)
            cvReleaseImage(&_logright);
        _logright = NULL;



       return true;
    };
 
    virtual bool interruptModule()
    {
       _leftImagePort.interrupt();
       _rightImagePort.interrupt();
       _disparityPort.interrupt();
       _blendedImagePort.interrupt();
       return true;
    };

    virtual bool updateModule()
    {
        yarp::sig::ImageOf<PixelRgb> *_leftImgIn, *_rightImgIn;
		_leftImgIn = _leftImagePort.read(true);       
        _rightImgIn = _rightImagePort.read(true); 

        if( _leftImgIn == NULL)  // this is the case if module is requested to quit while waiting for image
            return true;

        if( _rightImgIn == NULL)
            return true;

        // convert to GRAY
        cvCvtColor( _leftImgIn->getIplImage(), _grayleft, CV_BGR2GRAY );
        cvCvtColor( _rightImgIn->getIplImage(), _grayright, CV_BGR2GRAY );
       
        _lps._logmap(_grayleft->imageData, _logleft->imageData, false);
		_lps._logmap(_grayright->imageData, _logright->imageData, false);
		
	    int index;
		double disp = (double)_lgd.ComputeDisparity((unsigned char*)_logleft->imageData, 
                                            (unsigned char*)_logright->imageData,  
                                            _lp_cols*_lp_lines, 1, &index );

        Vector &d = _disparityPort.prepare();
		d.size(1);
        d[0] = disp/_input_cols;  
		_disparityPort.write();

        ImageOf<PixelRgb> &out = _blendedImagePort.prepare();
        out.resize(_input_cols, _input_lines);
        //blend images for output 
        cvMerge(_grayleft, _grayright, _grayright, NULL, out.getIplImage());
		_blendedImagePort.write();
		
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    yarp::os::Time::turboBoost();

    LogPolarGlobalDisparity module;
    module.setName("/disparity"); // set default name of module
    return module.runModule(argc,argv);
}
