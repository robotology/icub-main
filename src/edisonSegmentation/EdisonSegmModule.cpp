// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Alexandre Bernardino, Vislab, IST/ISR.
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <string>
#include <iostream>
using namespace std;


#include "EdisonSegmModule.h"


#include <BgImage.h>
#include <BgEdge.h>
#include <BgEdgeList.h>
#include <BgEdgeDetect.h>
#include <BgDefaults.h>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
using namespace yarp::os;


EdisonSegmModule::EdisonSegmModule()
{
  orig_height_       = -1;
  orig_width_		 = -1;
  height_            = -1;
  width_             = -1;
  dim_				 = -1;   //defaults to color image
  numEdges_          = -1;
  numBoundaries_     = -1;
  inputImage_        = (unsigned char *) NULL;
  filtImage_         = (unsigned char *) NULL;
  segmImage_         = (unsigned char *) NULL;
  gradMap_           = (float *) NULL;
  confMap_           = (float *) NULL;
  weightMap_         = (float *) NULL;
  edges_             = (int *)   NULL;
  boundaries_        = (int *)   NULL;

  //defaults for the parameters - as the ones in the EDISON GUI application
  sigmaS = 7;		
  sigmaR = 6.5;		
  minRegion = 20;  
  gradWindRad = 2; 
  threshold = 0.3; 
  mixture = 0.2;  
  speedup = MED_SPEEDUP; 
}

EdisonSegmModule::~EdisonSegmModule()
{
  if(edges_)       delete [] edges_;
  if(boundaries_)  delete [] boundaries_;
}


bool EdisonSegmModule::open(Searchable& config)
{
    if (config.check("help","if present, display usage message")) {
        printf("Call with --from configFile.ini\n");
        return false;
    }
    //need to initialize _xsize, _ysize, _thresh, _camerafile, _objectfile, _objectnum, _object

    ResourceFinder rf;
    
    if (config.check("context"))
        rf.setDefaultContext(config.find("context").asString());
    else
        rf.setDefaultContext(getName());

    if (config.check("from"))
        rf.setDefaultConfigFile(config.find("from").asString());
    else
        rf.setDefaultConfigFile("configFile.ini");

    rf.configure("ICUB_ROOT",0,NULL);

	//defaults for the parameters - as the ones in the EDISON GUI application
	height_ = 240;
	width_ = 320;
	dim_ = 3;
    sigmaS = 7;		
    sigmaR = 6.5;		
    minRegion = 20;  
    gradWindRad = 2; 
    threshold = 0.3; 
    mixture = 0.2;  
    speedup = MED_SPEEDUP; 
	//override defaults if specified
	if(rf.check("height")) height_ = rf.find("height").asInt();
    if(rf.check("width")) width_ = rf.find("width").asInt();
    if(rf.check("dim")) dim_ = rf.find("dim").asInt();
	if(rf.check("sigmaS")) sigmaS = rf.find("sigmaS").asInt();		
	if(rf.check("sigmaR")) sigmaR = rf.find("sigmaR").asDouble();		
	if(rf.check("minRegion")) minRegion = rf.find("minRegion").asInt();  
	if(rf.check("gradWinRad")) gradWindRad = rf.find("gradWinRad").asDouble(); 
	if(rf.check("threshold")) threshold = rf.find("threshold").asDouble(); 
	if(rf.check("mixture")) mixture = rf.find("mixture").asDouble();  
	if(rf.check("speedup")) 
	{
		int spdp = rf.find("speedup").asInt(); 
		switch(spdp) {
			case(0): speedup = NO_SPEEDUP; break;
			case(1): speedup = MED_SPEEDUP; break;
			case(2): speedup = HIGH_SPEEDUP; break;
			default: speedup = NO_SPEEDUP;
		}
	}
	// name of the camera calibration file
    // ConstString strCamConfigPath=rf.findFile("camera");

	_imgPort.open(getName("rawimg:i"));
	_configPort.open(getName("conf:i"));
	_filtPort.open(getName("filtimg:o"));
	_labelPort.open(getName("labelimg:o"));
	_viewPort.open(getName("viewimg:o"));
	_rawPort.open(getName("rawimg:o"));
	_labelViewPort.open(getName("debugimg:o"));
	attach(_configPort, true);

	//read an image to get the dimensions
	ImageOf<PixelRgb> *yrpImgIn;	
    yrpImgIn = _imgPort.read();
    if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;
	orig_height_ = yrpImgIn->height();
	orig_width_ = yrpImgIn->width();

	//THIS IS NOT REQUIRED - INPUT IMAGES ARE ALWAYS RGB
	//IF DIM == 3 THEN THE PROCESSING CLASS CONVERTS TO LUV
	//IF DIM == 1 THEN THE PROCESSING CLASS ONLY USES THE FIRST CHANNEL
	//WE USE HUE CHANNEL, SO MUST CONVERT FROM RGB TO HSV
	//check compatibility of image depth 
	/*if (yrpImgIn->getPixelSize() != dim_) 
	{
		cout << endl << "Incompatible image depth" << endl;
		return false;
	}*/ 

	//check image dimensions
	if( orig_width_ < width_ || orig_height_ < height_)
	{
		cout << endl << "Input image smaller than required" << endl;
		return false;
	}
	 
	//allocate memory for image buffers and get the pointers
 
	inputImage.resize(width_, height_); inputImage_ = inputImage.getRawImage();
	inputHsv.resize(width_, height_); inputHsv_ = inputHsv.getRawImage();
	inputHue.resize(width_, height_); inputHue_ = inputHue.getRawImage();
	filtImage.resize(width_, height_);  filtImage_ = filtImage.getRawImage();
	segmImage.resize(width_, height_);  segmImage_ = segmImage.getRawImage();
	gradMap.resize(width_, height_);    gradMap_ = (float*)gradMap.getRawImage();
	confMap.resize(width_, height_);    confMap_ = (float*)confMap.getRawImage();
	weightMap.resize(width_, height_);  weightMap_ = (float*)weightMap.getRawImage();

	labelImage.resize(width_, height_);
	labelView.resize(width_, height_);
    
	return true;
}

bool EdisonSegmModule::close()
{
    _imgPort.close();
    _labelPort.close();
	_labelViewPort.close();
	_filtPort.close();
    _viewPort.close();
	_configPort.close();
	_rawPort.close();

	// also deallocate image _frame
    return true;
}

bool EdisonSegmModule::interruptModule(){
    
    _imgPort.interrupt();
    _configPort.interrupt();
    _labelPort.interrupt();
	_filtPort.interrupt();
	_labelViewPort.interrupt();
	_viewPort.interrupt();
	_rawPort.interrupt();
    return true;
}

bool EdisonSegmModule::updateModule()
{
    ImageOf<PixelRgb> *yrpImgIn;
	static int cycles = 0;

    yrpImgIn = _imgPort.read();
    if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;

	if(cycles == 0)
		_timestart = yarp::os::Time::now();
	cycles++;

    IplImage *iplimg = (IplImage*)yrpImgIn->getIplImage();

    //computing the ROI to crop the image
	struct _IplROI roi;
	roi.coi = 0; // all channels are selected
	roi.height = height_;
	roi.width = width_;
	roi.xOffset = ( orig_width_ - width_ ) / 2;
	roi.yOffset = ( orig_height_ - height_ ) / 2;
	
	//copying roi data to buffer
	iplimg->roi = &roi;
	cvCopy( iplimg, inputImage.getIplImage());

	double edgetime = yarp::os::Time::now();
	//compute gradient and confidence maps
	BgEdgeDetect edgeDetector(gradWindRad);
	BgImage bgImage;
	bgImage.SetImage(inputImage_, width_, height_, true);
	edgeDetector.ComputeEdgeInfo(&bgImage, confMap_, gradMap_);
	//compute the weigth map
    for(int i = 0; i < width_*height_; i++) {
      if(gradMap_[i] > 0.02) {
		weightMap_[i] = mixture*gradMap_[i] + (1 - mixture)*confMap_[i];
      } else {
		weightMap_[i] = 0;
      }
    }
	///////////////////////////// This block can be parallelized
	cout << "Edge computation Time (ms): " << (yarp::os::Time::now() - edgetime)*1000.0 << endl;

	msImageProcessor iProc;
	if( dim_ == 3 )
		iProc.DefineImage(inputImage_, COLOR, height_, width_);
	else
	{	
		cvCvtColor(inputImage.getIplImage(), inputHsv.getIplImage(), CV_RGB2HSV);
		cvSplit(inputHsv.getIplImage(), inputHue.getIplImage(), 0, 0, 0);
		iProc.DefineImage(inputHue_, GRAYSCALE, height_, width_);
	}
	if(iProc.ErrorStatus) {
		cout << "MeanShift Error" << endl;
		return false;
	}
	iProc.SetWeightMap(weightMap_, threshold);
	if(iProc.ErrorStatus) {
		cout << "MeanShift Error" << endl;
		return false;
	}


	double filtertime = yarp::os::Time::now();
	iProc.Filter(sigmaS, sigmaR, speedup);
    if(iProc.ErrorStatus) {
		cout << "MeanShift Error" << endl;
		return false;
	}
	cout << "Mean Shift Filter Computation Time (ms): " << (yarp::os::Time::now() - filtertime)*1000.0 << endl;


    //obtain the filtered image
    iProc.GetResults(filtImage_);
	if(iProc.ErrorStatus) {
		cout << "MeanShift Error" << endl;
		return false;
	}
    
    //fuse regions
	double fusetime = yarp::os::Time::now();
    iProc.FuseRegions(sigmaR, minRegion);
    if(iProc.ErrorStatus) {
		cout << "MeanShift Error" << endl;
		return false;
	}
	cout << "Region Fusion Computation Time (ms): " << (yarp::os::Time::now() - fusetime)*1000.0 << endl;

    //obtain the segmented image
    iProc.GetResults(segmImage_);
    if(iProc.ErrorStatus) {
		cout << "MeanShift Error" << endl;
		return false;
	}

	//define the boundaries - do not need this
	/*
	RegionList *regionList        = iProc.GetBoundaries();
	int        *regionIndeces     = regionList->GetRegionIndeces(0);
	int        numRegions         = regionList->GetNumRegions();
	numBoundaries_ = 0;
	for(int i = 0; i < numRegions; i++) {
		numBoundaries_ += regionList->GetRegionCount(i);
	}
	if(boundaries_) delete [] boundaries_;
	boundaries_ = new int [numBoundaries_];
	for(int i = 0; i < numBoundaries_; i++) {
		boundaries_[i] = regionIndeces[i];
	}*/
		
	int regionCount; // how many regions have been found
	int *labels = NULL; //pointer for the labels (should this be released in the end?) 
	float *modes; //pointer for the Luv values (should this be released in the end?) 
	int *modePointCounts; //the area of each region (should this be released in the end?) 

	regionCount = iProc.GetRegions(&labels, &modes, &modePointCounts);
	int *labelp = (int*)labelImage.getRawImage();
	for(int i = 0; i < width_*height_; i++)
		labelp[i] = labels[i]*30 + 30;
	
	IplImage *labelint = (IplImage*)labelImage.getIplImage();
	IplImage *labelchar = (IplImage*)labelView.getIplImage();

	cvConvert(labelint, labelchar);

	ImageOf<PixelInt> &yrpImgLabel = _labelPort.prepare();
	yrpImgLabel = labelImage;
	_labelPort.write();

	ImageOf<PixelMono> &yrpImgDebug = _labelViewPort.prepare();
	yrpImgDebug = labelView;
	_labelViewPort.write();


	ImageOf<PixelRgb> &yrpFiltOut = _filtPort.prepare();
	yrpFiltOut = filtImage;
	_filtPort.write();

	ImageOf<PixelRgb> &yrpImgView = _viewPort.prepare();
	yrpImgView = segmImage;
	_viewPort.write();

	ImageOf<PixelRgb> &yrpImgOut = _rawPort.prepare();
	yrpImgOut = *yrpImgIn;
	_rawPort.write();


	//report the frame rate
	if(cycles % 100 == 0)
	{
		double cps = ((double)cycles)/(yarp::os::Time::now() - _timestart);
		printf("fps: %02.2f\n", cps);
	}
    return true;
}


int main(int argc, char *argv[]) {

    Network yarp;
    EdisonSegmModule module;
    module.setName("/edisonSegm"); // set default name of module
    return module.runModule(argc,argv);
}
