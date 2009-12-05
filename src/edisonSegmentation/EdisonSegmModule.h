// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Alex Bernardino, Vislab, IST/ISR
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __EDISONSEGMMODULE__
#define __EDISONSEGMMODULE__

 // std
#include <stdio.h>


// OpenCV
#include  <cv.h>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Module.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


/**
 *
 * Edison Segmentation Module
 *
 *
 */
#include <msImageProcessor.h>

class EdisonSegmModule : public Module {

private:
	double _timestart;
	int orig_height_, height_, orig_width_, width_, dim_;		//input image dimensions
	
	//internal storage for the input image - must be RGB
	ImageOf<PixelRgb> inputImage;
	unsigned char * inputImage_;

	//internal storage for hsv image, only needed for dim = 1.
	ImageOf<PixelRgb> inputHsv;
	unsigned char * inputHsv_;

	//internal storage for hue channel, only needed for dim = 1.
	ImageOf<PixelMono> inputHue;
	unsigned char * inputHue_;

	//store the weight maps
	ImageOf<PixelFloat> gradMap;
	float * gradMap_;
	
	ImageOf<PixelFloat> confMap;
	float * confMap_;
	
	ImageOf<PixelFloat> weightMap;
	float * weightMap_;

	//float * custMap_; //????? Is this really needed

	//storage for the output images
	ImageOf<PixelRgb> filtImage;
	unsigned char * filtImage_;

	ImageOf<PixelRgb> segmImage;
	unsigned char * segmImage_;

	ImageOf<PixelInt> labelImage;

	ImageOf<PixelMono> labelView; // for visualizing the labels - debug stuff

	//store the output edges and boundaries
	int *edges_, numEdges_;
	int *boundaries_, numBoundaries_;

	//parameters for mean shift
	int sigmaS;		//spatial bandwidth
	float sigmaR;		//range bandwidth
	int minRegion;  //area of the smallest objects to consider
	
	//parameters for synergistic segmentation
	int gradWindRad; //Gradient Window Radius
	float threshold; //Edge Strength Threshold [0,1]
	float mixture;   //Mixture Parameter [0,1]

	SpeedUpLevel speedup; //{NO_SPEEDUP, MED_SPEEDUP, HIGH_SPEEDUP} 
	
    BufferedPort<ImageOf<PixelRgb> >       _imgPort;      //input image
	BufferedPort<ImageOf<PixelRgb> >       _rawPort;      //raw image 
	BufferedPort<ImageOf<PixelRgb> >       _viewPort;     //output image - segmentation (modes of each detected object), edges, etc (configurable)
	BufferedPort<ImageOf<PixelInt> >       _labelPort;    //output image with labels 
    BufferedPort<ImageOf<PixelRgb> >       _filtPort;     //output the mean shift filtered image
	BufferedPort<Bottle>				   _configPort;   //to configure the module
	BufferedPort<ImageOf<PixelMono> >      _labelViewPort; //to visualize the labels

public:
    EdisonSegmModule();
    ~EdisonSegmModule();
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
};


#endif

