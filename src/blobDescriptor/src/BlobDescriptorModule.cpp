// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>, Ivana Cingovska, Alexandre Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 
 *
 */
 
/* YARP */
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>
using namespace yarp::os;

/* iCub */
#include <iCub/BlobDescriptorModule.h>
#include <iCub/BlobDescriptorSupport.h>

/* system */
#include <iostream>
//#include <math.h>
#include <stdio.h>
using namespace std;

// FIXME: make these user-specified parameters
#define H_BINS 30
#define S_BINS 30
#define V_BINS 30

/**
 * Receive a previously initialized Resource Finder object and process module parameters,
 * both from command line and .ini file.
 */
bool BlobDescriptorModule::configure(ResourceFinder &rf) // equivalent to Module::open()
{
	/* get the robot name that will form the prefix of all module port names */
	_moduleName = rf.check( "name",
							Value("/blobDescriptor"),
							"Module name (string)" ).asString();
	/* before continuing, set the module name */
	setName(_moduleName.c_str());

	/* now, get the remaining parameters */
		
	/* get the robot name which will form the prefix of robot port names,
	 * append the specific part and device required */
	_robotName = rf.check( "robot",
						   Value("/icub"),
						   "Robot name (string)" ).asString();
	_robotPortName = _robotName + "/head"; // FIXME: maybe it should be "/" + _robotName + "/head"

	_rawImgInputPortName         = getName(
                                           rf.check( "raw_image_input_port",
                                                     Value("/rawImg:i"),
                                                     "Raw image input port (string)" ).asString()
                                           );
    _labeledImgInputPortName     = getName(
                                           rf.check( "labeled_image_input_port",
                                                     Value("/labeledImg:i"),
                                                     "Labeled image input port (string)" ).asString()
                                           );
    _rawImgOutputPortName        = getName(
                                           rf.check( "raw_image_output_port",
                                                     Value("/rawImg:o"),
                                                     "Raw image output port (string)" ).asString()
                                           );
    _viewImgOutputPortName       = getName(
                                           rf.check( "view_image_output_port",
                                                     Value("/viewImg:o"),
                                                     "View image output port (string)" ).asString()
                                           );
    _affDescriptorOutputPortName = getName(
                                           rf.check( "aff_descriptor_output_port",
                                                     Value("/affDescriptor:o"),
                                                     "Affordance descriptor output port (string)" ).asString()
                                           );
    _trackerInitOutputPortName   = getName(
                                           rf.check( "tracker_init_output_port",
                                                     Value("/trackerInit:o"),
                                                     "Tracker initialization output port (string)" ).asString()
                                           );
    _minAreaThreshold = rf.check( "min_area_threshold",
                                  Value(100),
                                  "Minimum number of pixels allowed for foreground objects" ).asInt();

	_maxObjects = rf.check( "max_objects" , 
						    Value(20), 
							"Maximum number of objects to process" ).asInt();
	if( _maxObjects <= 0)
	{
		cout << "WARNING: Invalid parameter (number of objects). Will use default (20)" << endl;
		_maxObjects = 20;
	}

	//Network::init();
	
	/* open ports */
	if(! _rawImgInputPort.open(_rawImgInputPortName.c_str()) )
	{
		cout << getName() << ": unable to open port" << _rawImgInputPortName << endl;
		return false;
	}
    if(! _labeledImgInputPort.open(_labeledImgInputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _labeledImgInputPortName << endl;
        return false;
    }
    if(! _rawImgOutputPort.open(_rawImgOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _rawImgOutputPortName << endl;
        return false;
    }
    if(! _viewImgOutputPort.open(_viewImgOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _viewImgOutputPortName << endl;
        return false;
    }
    if(! _affDescriptorOutputPort.open(_affDescriptorOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _affDescriptorOutputPortName << endl;
        return false;
    }
    if(! _trackerInitOutputPort.open(_trackerInitOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _trackerInitOutputPortName << endl;
        return false;
    }

    _yarpRawInputPtr     = _rawImgInputPort.read(true);
    _yarpLabeledInputPtr = _labeledImgInputPort.read(true);
    
	/* check that raw and labeled image dimensions are the same */
	if( (_yarpRawInputPtr->width() != _yarpLabeledInputPtr->width()) || 
		(_yarpRawInputPtr->height() != _yarpLabeledInputPtr->height()))
	{
		cout << getName() << ": input image dimensions differ. Exiting..." << endl;
        return false;
	}
	
    _w   = _yarpRawInputPtr->width();
    _h   = _yarpRawInputPtr->height();
    _sz  = cvSize(_w, _h);

	/* allocate internal image buffers */
	_yarpRawImg.resize(_w,_h);
	_yarpHSVImg.resize(_w,_h);
	_yarpHueImg.resize(_w,_h);
	_yarpLabeledImg.resize(_w,_h);
	_yarpViewImg.resize(_w,_h);
	_yarpTempImg.resize(_w,_h);
    //_h_plane = cvCreateImage(_sz, IPL_DEPTH_8U, 1);

    /* IvanaModule::init() */
    _hist_size[0] = H_BINS;
    _hist_size[1] = S_BINS;
    /* hue varies from 0 (~0°red) to 180 (~360°red again) */
    _h_ranges[0]  =   0;
    _h_ranges[1]  = 360;
    /* saturation varies from 0 (black-gray-white) to 255 (pure spectrum color) */
    _s_ranges[0]  =   0;
    _s_ranges[1]  = 255;
    _v_ranges[0]  =   0;
    _v_ranges[1]  = 255;
	float *ranges[] = { _h_ranges, _s_ranges, _v_ranges };
    
	/* initialize object descriptor list */
	_objDescTable = new ObjectDescriptor[_maxObjects];
	/* must allocate and initialize masks and histograms */
	/* FIXME: in the future, this should go in the object class constructor */
	for(int i = 0; i < _maxObjects; i++)
	{
		_objDescTable[i].mask_image = cvCreateImage(_sz, IPL_DEPTH_8U, 1);
		_objDescTable[i].mask_data = (unsigned char *) _objDescTable[i].mask_image->imageData;
		_objDescTable[i].h_bins = _hist_size[0];
		_objDescTable[i].s_bins = _hist_size[1];
        _objDescTable[i].objHist = cvCreateHist(1, _hist_size, CV_HIST_ARRAY, ranges, 1);
		_objDescTable[i].storage = cvCreateMemStorage(0);
		_objDescTable[i].contours = 0;
		_objDescTable[i].convexhull = 0; 

	}

	return true; // tell RFModule that everything went well, so that it will run the module
}
	
/**
 * Try to halt operations by threads managed by the module. Called asynchronously
 * after a quit command is received.
 */
bool BlobDescriptorModule::interruptModule()
{
	cout << getName() << ": interrupting module, for port cleanup." << endl;
	_rawImgInputPort.interrupt();
    _labeledImgInputPort.interrupt();
    _rawImgOutputPort.interrupt();
    _viewImgOutputPort.interrupt();
    _affDescriptorOutputPort.interrupt();
    _trackerInitOutputPort.interrupt();
	return true;
}
	
/**
 * Close function. Called automatically when the module closes, after the last
 * updateModule call.
 */
bool BlobDescriptorModule::close()
{
	cout << getName() << ": closing module." << endl;

    _rawImgInputPort.close();
    _labeledImgInputPort.close();
    _rawImgOutputPort.close();
    _viewImgOutputPort.close();
    _affDescriptorOutputPort.close();
    _trackerInitOutputPort.close();

	for(int i = 0; i < _maxObjects; i++)
	{
		cvReleaseImage(&(_objDescTable[i].mask_image));
		cvReleaseHist(&(_objDescTable[i].objHist));
	}
	delete [] _objDescTable;

	// Network::fini();
	return true;
}
   
/**
 * Message handler function. Echo all received messages, quit if required.
 */
bool BlobDescriptorModule::respond(const Bottle &command, Bottle &reply)
{
  	cout << getName() << ": echoing received command." << endl;
  	reply = command;
  	if(command.get(0).asString() == "quit")
		return false;
  	else
  		return true;
}
   
/**
 * Main cycle, called iteratively every getPeriod() seconds.
 */
bool BlobDescriptorModule::updateModule()
{
	Stamp rawstamp, labeledstamp; 
	
    _yarpRawInputPtr = _rawImgInputPort.read(true);
	_yarpLabeledInputPtr = _labeledImgInputPort.read(true);
	
	/* check that both images have timestamps */
	if( !_rawImgInputPort.getEnvelope(rawstamp) || !_labeledImgInputPort.getEnvelope(labeledstamp) )
	{
        cout << getName() << ": this module requires ports with valid timestamp data. Stamps are missing. Exiting..." << endl;
		return false;
	}
    /* synchronize the two images, if one of them is delayed, so that they correspond */
	while( rawstamp.getCount() < labeledstamp.getCount() )
	{
		_yarpRawInputPtr = _rawImgInputPort.read(true);
		_rawImgInputPort.getEnvelope(rawstamp);
	}
	while( rawstamp.getCount() > labeledstamp.getCount() )
	{
		_yarpLabeledInputPtr = _labeledImgInputPort.read(true);
		_labeledImgInputPort.getEnvelope(labeledstamp);
	}

	_yarpRawImg = *_yarpRawInputPtr;
	_yarpViewImg = _yarpRawImg;
	_yarpLabeledImg = *_yarpLabeledInputPtr;

	/* get OpenCV pointers to images, to more easily call OpenCV functions */
    IplImage *opencvRawImg     = (IplImage *) _yarpRawImg.getIplImage();
	IplImage *opencvHSVImg     = (IplImage *) _yarpHSVImg.getIplImage();
	IplImage *opencvHueImg     = (IplImage *) _yarpHueImg.getIplImage();
    IplImage *opencvLabeledImg = (IplImage *) _yarpLabeledImg.getIplImage();
	IplImage *opencvViewImg    = (IplImage *) _yarpViewImg.getIplImage();
	IplImage *opencvTempImg    = (IplImage *) _yarpTempImg.getIplImage();

    /* convert from RGB to HSV and get the Hue plane - to compute the histograms */
	cvCvtColor(opencvRawImg, opencvHSVImg, CV_RGB2HSV);
	cvSplit(opencvHSVImg, opencvHueImg, NULL, NULL, NULL);
    IplImage *planes[] = { opencvHueImg };

    /* compute numLabels as the max value within opencvLabeledImg */
    double max_val, trash;
    cvMinMaxLoc(opencvLabeledImg, &trash, &max_val, NULL, NULL, NULL);
    int numLabels = (int) max_val;

	/* FIXME: different selection criteria should be accepted here */
    _numObjects = selectObjects( opencvLabeledImg, opencvTempImg, numLabels, _minAreaThreshold);
	if(_numObjects > _maxObjects )
	{
        cout << getName() << ": more objects than the permitted maximum. Only " << _maxObjects << " will be processed." << endl;
		_numObjects = _maxObjects;
	}

    /* extract characteristics of objects */
    extractObj(opencvLabeledImg, _numObjects, _objDescTable);

	/* here, all objects have been segmented and are stored independently. */

	/* contour extraction */
	for( int i=0; i < _numObjects; i++)
	{
		cvFindContours(_objDescTable[i].mask_image, 
		               _objDescTable[i].storage, 
					   &(_objDescTable[i].contours),
					   sizeof(CvContour),
					   CV_RETR_LIST, 
					   CV_CHAIN_APPROX_SIMPLE, 
				       cvPoint(0,0)
					   );
	}

	for( int i=0; i < _numObjects; i++)
	{
		/* contour drawing */
		cvDrawContours(
			opencvViewImg, 
			_objDescTable[i].contours, 
			CV_RGB(0,255,0), // external color
			CV_RGB(0,0,255), // hole color
			1,				 
			1, 
			CV_AA, 
			cvPoint(0, 0)	 // ROI offset
			);
	}
	
	//DEBUG - print the characteristics of the objects found
	for(int i=0; i < _numObjects; i++)
    {
		cout << "Object no " << _objDescTable[i].no;
		cout << " label " << _objDescTable[i].label;
		cout << " area " << _objDescTable[i].area;
		cout << " x " << _objDescTable[i].center.x;
		cout << " y " << _objDescTable[i].center.y << endl;
	}


    /* compute histogram of each object */
    for(int i=0; i < _numObjects; i++)
    {
        cvCalcHist(planes, _objDescTable[i].objHist, 0, _objDescTable[i].mask_image);
        float ohmax; // to normalize the object histogram
        cvGetMinMaxHistValue(_objDescTable[i].objHist, 0, &ohmax, 0, 0);
        cvConvertScale(_objDescTable[i].objHist->bins, _objDescTable[i].objHist->bins, ohmax ? 255. / ohmax : 0., 0);
    }

	/* output image to view results */
	ImageOf<PixelRgb> &yarpOutputImage = _viewImgOutputPort.prepare();
	yarpOutputImage = _yarpViewImg;
	_viewImgOutputPort.write();
  	return true;
}
