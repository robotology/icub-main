// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>, Ivana Cingovska
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 
 *
 */
 
/* YARP */
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
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
                                  Value(2000),
                                  "Minimum number of pixels allowed for foreground objects" ).asInt();

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

    _yarpRawImg     = _rawImgInputPort.read(true);
    _yarpLabeledImg = _labeledImgInputPort.read(true);
    /* the OpenCV versions of these images will be set in BlobDescriptorModule::updateModule() */

    // FIXME: check
    h_plane = cvCreateImage(labeled_sz, IPL_DEPTH_8U, 1);

    /* IvanaModule::init() */
    hist_size    = new int[2];
    hist_size[0] = H_BINS;
    hist_size[1] = S_BINS;
    h_ranges     = new float[2];
    s_ranges     = new float[2];
    v_ranges     = new float[2];
    /* hue varies from 0 (~0°red) to 180 (~360°red again) */
    h_ranges[0]  =   0;
    h_ranges[1]  = 360;
    /* saturation varies from 0 (black-gray-white) to 255 (pure spectrum color) */
    s_ranges[0]  =   0;
    s_ranges[1]  = 255;
    v_ranges[0]  =   0;
    v_ranges[1]  = 255;

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

    cvReleaseImage(&_opencvRawImg);
    cvReleaseImage(&_opencvLabeledImg32);
    cvReleaseImage(&_opencvLabeledImg8);
    cvReleaseImage(&h_plane);

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
    _yarpRawImg     = _rawImgInputPort.read(true);
    _yarpLabeledImg = _labeledImgInputPort.read(true);

    raw_w   = _yarpRawImg->width();
    raw_h   = _yarpRawImg->height();
    raw_sz  = cvSize(raw_w, raw_h);

    labeled_w  = _yarpLabeledImg->width();
    labeled_h  = _yarpLabeledImg->height();
    labeled_sz = cvSize(labeled_w, labeled_h);

    _opencvRawImg = (IplImage *) _yarpRawImg->getIplImage();
    _opencvLabeledImg32 = (IplImage *) _yarpLabeledImg->getIplImage();

    // DEBUG
    int *ptr = (int *) _yarpLabeledImg->getRawImage();

    float *ranges[] = { h_ranges, s_ranges, v_ranges }; // FIXME: deallocate this?
    IplImage *planes[] = { h_plane }; /* just hue (Ivana used saturation, too - planesW variable) */

    /* compute _numLabels as the max value within _opencvLabeledImg */
    double max_val, trash;
    cvMinMaxLoc(_opencvLabeledImg32, &trash, &max_val, NULL, NULL, NULL);
    _numLabels = (int) max_val;

    // DEBUG
    for(int i=0; i<labeled_h; i++)
    {
        for(int j=0; j<labeled_w; j++)
        {
            cout << *ptr++ << " ";
        }
        cout << endl;
    }
    //return false;

    _numObjects = selectObjects(_opencvLabeledImg32, _opencvLabeledImg8, _numLabels, _minAreaThreshold);
    cout << getName() << ": number of objects is " << _numObjects << endl;

    /* extract characteristics of objects */
    objDescTable = new ObjectDescriptor[_numObjects];
    extractObj(_opencvLabeledImg32, _numObjects, objDescTable);

    /* compute histogram of each object */
    for(int i=0; i < _numObjects; i++)
    {



        objDescTable[i].h_bins = hist_size[0];
        objDescTable[i].s_bins = hist_size[1];
        objDescTable[i].objHist = cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges, 1);
        cvCalcHist(planes, objDescTable[i].objHist, 0, objDescTable[i].mask_image);
        float ohmax; // to normalize the object histogram
        cvGetMinMaxHistValue(objDescTable[i].objHist, 0, &ohmax, 0, 0);
        cvConvertScale(objDescTable[i].objHist->bins, objDescTable[i].objHist->bins, ohmax ? 255. / ohmax : 0., 0);




    }


    
  	return true;
}
