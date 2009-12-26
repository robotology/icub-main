// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>, Ivana Cingovska, Alexandre Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 
 *
 */
 
/* YARP */
#include <yarp/os/Network.h>
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
//#define H_BINS 30
//#define S_BINS 30
//#define V_BINS 30

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
    _userSelectionInputPortName  = getName(
                                           rf.check( "user_selection_port",
                                                     Value("/userSelection:i"),
                                                     "User selection input port (string)" ).asString()
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
    _trackerInitSingleObjOutputPortName = getName(
                                                  rf.check( "tracker_init_single_obj_output_port",
                                                            Value("/trackerInitSingleObj:o"),
                                                            "Single object tracker initialization output port (string)" ).asString()
                                                  );
    _handlerPortName = getName(
                               rf.check( "conf_port",
                                         Value("/conf"),
                                         "Configuration and message handling port (string)" ).asString()
                               );
    _minAreaThreshold = rf.check( "min_area_threshold",
                                  Value(100),
                                  "Minimum number of pixels allowed for foreground objects" ).asInt();
	_maxObjects = rf.check( "max_objects" , 
						    Value(20), 
							"Maximum number of objects to process" ).asInt();
	if( _maxObjects <= 0)
	{
		cout << getName() << " WARNING: Invalid number of objects parameter. Will use default (20) instead." << endl;
		_maxObjects = 20;
	}

	//Network::init();
	
	/* open ports */
    if(! _handlerPort.open(_handlerPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _handlerPortName << endl;
        return false;
    }
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
    if(! _userSelectionInputPort.open(_userSelectionInputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _userSelectionInputPortName << endl;
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
    /* descriptor of a single object, selected by the user on yarpview - debug */
    if(! _trackerInitSingleObjOutputPort.open(_trackerInitSingleObjOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << _trackerInitSingleObjOutputPortName << endl;
        return false;
    }

    _yarpRawInputPtr     = _rawImgInputPort.read(true);
    _yarpLabeledInputPtr = _labeledImgInputPort.read(true);
    
	/* check that raw and labeled image dimensions are equal */
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
	_yarpSatImg.resize(_w,_h);
	_yarpValImg.resize(_w,_h);
	_yarpLabeledImg.resize(_w,_h);
	_yarpViewImg.resize(_w,_h);
	_yarpTempImg.resize(_w,_h);
    //_h_plane = cvCreateImage(_sz, IPL_DEPTH_8U, 1);

    /* IvanaModule::init() */
    //_hist_size[0] = H_BINS;
    //_hist_size[1] = S_BINS;
    /* hue varies from 0 (~0°red) to 180 (~360°red again) */
    //_h_ranges[0]  =   0;
    //_h_ranges[1]  = 360;
    /* saturation varies from 0 (black-gray-white) to 255 (pure spectrum color) */
	//_s_ranges[0]  =   0;
    //_s_ranges[1]  = 255;
    //_v_ranges[0]  =   0;
    //_v_ranges[1]  = 255;
	//float *ranges[] = { _h_ranges, _s_ranges, _v_ranges };
    
	/* initialize object descriptor list */
	_objDescTable = new ObjectDescriptor[_maxObjects];
	for(int i = 0; i < _maxObjects; i++)	
		_objDescTable[i].Create(_w,_h);
	return true; /* tell RFModule that everything went well, so that it will run the module */
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
    _userSelectionInputPort.interrupt();
    _rawImgOutputPort.interrupt();
    _viewImgOutputPort.interrupt();
    _affDescriptorOutputPort.interrupt();
    _trackerInitOutputPort.interrupt();
    _trackerInitSingleObjOutputPort.interrupt(); /* from user click - debug */
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
    _userSelectionInputPort.close();
    _rawImgOutputPort.close();
    _viewImgOutputPort.close();
    _affDescriptorOutputPort.close();
    _trackerInitOutputPort.close();
    _trackerInitSingleObjOutputPort.close();

	
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
	Stamp rawstamp, labeledstamp, writestamp; 
	
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

	//here both stamps are equal
	writestamp = rawstamp;

    _yarpRawImg     = *_yarpRawInputPtr;
    _yarpViewImg    =  _yarpRawImg;
    _yarpLabeledImg = *_yarpLabeledInputPtr;

	/* get OpenCV pointers to images, to more easily call OpenCV functions */
    IplImage *opencvRawImg     = (IplImage *) _yarpRawImg.getIplImage();
	IplImage *opencvHSVImg     = (IplImage *) _yarpHSVImg.getIplImage();
	IplImage *opencvHueImg     = (IplImage *) _yarpHueImg.getIplImage();
	IplImage *opencvSatImg     = (IplImage *) _yarpSatImg.getIplImage();
	IplImage *opencvValImg     = (IplImage *) _yarpValImg.getIplImage();
    IplImage *opencvLabeledImg = (IplImage *) _yarpLabeledImg.getIplImage();
	IplImage *opencvViewImg    = (IplImage *) _yarpViewImg.getIplImage();
	IplImage *opencvTempImg    = (IplImage *) _yarpTempImg.getIplImage();

    /* convert from RGB to HSV and get the Hue plane - to compute the histograms */
	cvCvtColor(opencvRawImg, opencvHSVImg, CV_RGB2HSV);
	cvSplit(opencvHSVImg, opencvHueImg, opencvSatImg, opencvValImg, NULL);
    IplImage *planes[] = { opencvHueImg }; //compute histogram of hue only

	/* generic variables for use with cvMaxMinLoc */
	double max_val, min_val;
	//CvPoint max_loc, min_loc;

    /* compute numLabels as the max value within opencvLabeledImg */
    cvMinMaxLoc(opencvLabeledImg, &min_val, &max_val, NULL, NULL, NULL);
	if(min_val != 0)
		cout << "WARNING: min_val of labeled image is different from zero !!!!" << endl;
	
	int numLabels = (int)max_val + 1;

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

		if(_objDescTable[i].contours == NULL)
			cout << "Something very wrong happened. Object without edges" << endl;

		if(_objDescTable[i].contours->h_next == NULL)
			_objDescTable[i].valid = true;
		else
			_objDescTable[i].valid = false;  //objects with holes are not allowed
	}

	
	for( int i=0; i < _numObjects; i++)
	{
		/* contour drawing - all objects */
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
    for(int i = 0; i < _numObjects; i++)
    {
        cvCalcHist(planes, _objDescTable[i].objHist, 0, _objDescTable[i].mask_image);
        float ohmax; // to normalize the object histogram
        cvGetMinMaxHistValue(_objDescTable[i].objHist, 0, &ohmax, 0, 0);
        cvConvertScale(_objDescTable[i].objHist->bins, _objDescTable[i].objHist->bins, ohmax ? 255. / ohmax : 0., 0);
    }

	/* compute saturation and intensity bounds for each object */
	for(int i = 0; i < _numObjects; i++)
    {
		cvMinMaxLoc(opencvSatImg, &min_val, &max_val, 0, 0, _objDescTable[i].mask_image);
		_objDescTable[i].s_min = (int)min_val;
		_objDescTable[i].s_max = (int)max_val;
		cvMinMaxLoc(opencvValImg, &min_val, &max_val, 0, 0, _objDescTable[i].mask_image);
		_objDescTable[i].v_min = (int)min_val;
		_objDescTable[i].v_max = (int)max_val;
    }
	/* compute the roi for each object to set the target roi */
	for(int i = 0; i < _numObjects; i++)
    {
		//MUST FIND A GOOD WAY TO DO IT
		//MAYBE USE FITELLIPSE ON THE CONTOUR
		//TRACKER COULD ALSO RECEIVE THE ANGLE
		//THE FOLLOWING INITIALIZATION IS JUST FOR EARLY TESTING
		_objDescTable[i].roi_height = 30;
		_objDescTable[i].roi_width = 30;
		_objDescTable[i].roi_x = _objDescTable[i].center.x - 15;
		_objDescTable[i].roi_y = _objDescTable[i].center.y - 15;
    }

	Bottle *userinput = _userSelectionInputPort.read(false);
	int userselection = -1;
	if(userinput != NULL)
	{
		CvPoint2D32f pt;
		pt.x = (float)(userinput->get(0).asInt());
		pt.y = (float)(userinput->get(1).asInt());
				
		cout << "Received selection (x,y) = " << pt.x << "  " << pt.y << endl;  
		//check which is the selected object
		for(int i = 0; i < _numObjects; i++)
		{
			// test only valid objects (without holes)
			if(_objDescTable[i].valid)
			{
				if( cvPointPolygonTest( _objDescTable[i].contours, pt, 0) > 0) //point inside
				{
					userselection = i;
					break;
				}
			}
		}
		//Draw the contour of the selected object with a different color
		if(userselection != -1)
			cvDrawContours(
				opencvViewImg, 
				_objDescTable[userselection].contours, 
				CV_RGB(0,0,255), // external color
				CV_RGB(0,0,0), // hole color
				1,				 
				1, 
				CV_AA, 
				cvPoint(0, 0)	 // ROI offset
		);
	}

	//Approximate the contours - only for valid objects
	for( int i=0; i < _numObjects; i++)
	{
		if(_objDescTable[i].valid)
		{
			_objDescTable[i].contours = cvApproxPoly( 
				_objDescTable[i].contours, 
				sizeof(CvContour), 
				_objDescTable[i].storage, 
				CV_POLY_APPROX_DP, 
				cvContourPerimeter(_objDescTable[i].contours)*0.02, 
				1);

			_objDescTable[i].convexhull = cvConvexHull2( _objDescTable[i].contours, _objDescTable[i].storage, CV_CLOCKWISE, 1 );

			//mesurements of the contour
			_objDescTable[i].contour_area = fabs(cvContourArea( _objDescTable[i].contours, CV_WHOLE_SEQ ));
			_objDescTable[i].contour_perimeter = cvArcLength( _objDescTable[i].contours, CV_WHOLE_SEQ, 1 );
			_objDescTable[i].convex_perimeter = cvArcLength( _objDescTable[i].convexhull, CV_WHOLE_SEQ, 1 );
			CvBox2D enclosing_rect = cvMinAreaRect2( _objDescTable[i].convexhull, _objDescTable[i].storage );
			_objDescTable[i].major_axis = (enclosing_rect.size.width > enclosing_rect.size.height ? enclosing_rect.size.width : enclosing_rect.size.height);
			_objDescTable[i].minor_axis = (enclosing_rect.size.width > enclosing_rect.size.height ? enclosing_rect.size.height : enclosing_rect.size.width);
			_objDescTable[i].rect_area = _objDescTable[i].major_axis*_objDescTable[i].minor_axis;
		
			CvPoint2D32f center;
			float radius;
			cvMinEnclosingCircle( _objDescTable[i].contours, &center, &radius );

			//shape descriptors
			if(_objDescTable[i].contour_perimeter > 0)
				_objDescTable[i].convexity = _objDescTable[i].convex_perimeter/_objDescTable[i].contour_perimeter;
			else
				_objDescTable[i].convexity = 0;

			if(_objDescTable[i].major_axis > 0)
				_objDescTable[i].eccentricity = _objDescTable[i].minor_axis/_objDescTable[i].major_axis;
			else
				_objDescTable[i].eccentricity = 0;

	
			if(_objDescTable[i].contour_perimeter > 0)
				_objDescTable[i].compactness = _objDescTable[i].contour_area/(_objDescTable[i].contour_perimeter*_objDescTable[i].contour_perimeter);

			if( radius > 0)
				_objDescTable[i].circleness = _objDescTable[i].contour_area/(3.1415*radius*radius);

			if(_objDescTable[i].rect_area > 0)
				_objDescTable[i].squareness = _objDescTable[i].contour_area/_objDescTable[i].rect_area;

			//find the approximating ellipse
			_objDescTable[i].ellipse = cvFitEllipse2(_objDescTable[i].contours);

			/*printf("\nArea: %f\n", contour_area);			// 1
			printf("Convexity: %f\n", convexity);			// 2
			printf("Eccentricity: %f\n", eccentricity);		// 3
			printf("Compactness: %f\n", compactness);		// 4
			printf("Circleness: %f\n", circleness);			// 5
			printf("Squareness: %f\n\n", squareness);		// 6
			*/
			//display contours in image

			
			//printf("\ndid cvDrawContours\n");
		}
	}

	Bottle &affbot = _affDescriptorOutputPort.prepare();
	affbot.clear();
	/* output affordance descriptors */
	for(int i = 0; i < _numObjects; i++)
	{
		if( _objDescTable[i].valid)
		{
			Bottle &objbot = affbot.addList();
			objbot.clear();

			double x = _objDescTable[i].ellipse.center.x;
			double y = _objDescTable[i].ellipse.center.y;
			double w = _objDescTable[i].ellipse.size.width;
			double h = _objDescTable[i].ellipse.size.height;

			double norm_x = (x-w/2)/(w/2); //between -1 and 1 
			double norm_y = (y-h/2)/(h/2); //between -1 and 1 
			double norm_w = (w/_w);        //between 0 and 1 
			double norm_h = (h/_h);		   //between 0 and 1 	

			/*0*/objbot.addDouble(norm_x);
			/*1*/objbot.addDouble(-norm_y);
			/*2*/objbot.addDouble(norm_w);
			/*3*/objbot.addDouble(norm_h);
			/*4*/objbot.addDouble(_objDescTable[i].ellipse.angle);
			/*5*/objbot.addDouble(0);
			/*6*/objbot.addDouble(0);


			/*7*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 0));
			/*8*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 1));
			/*9*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 2));
			/*10*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 3));
			/*11*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 4));
			/*12*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 5));
			/*13*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 6));
			/*14*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 7));
			/*15*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 8));
			/*16*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 9));
			/*17*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 10));
			/*18*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 11));
			/*19*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 12));
			/*20*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 13));
			/*21*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 14));
			/*22*/objbot.addDouble((double)cvQueryHistValue_1D(_objDescTable[i].objHist, 15));

			/*23*/objbot.addDouble((double)_objDescTable[i].contour_area);
			/*24*/objbot.addDouble((double)_objDescTable[i].convexity);
			/*25*/objbot.addDouble((double)_objDescTable[i].eccentricity);
			/*26*/objbot.addDouble((double)_objDescTable[i].compactness);
			/*27*/objbot.addDouble((double)_objDescTable[i].circleness);
			/*28*/objbot.addDouble((double)_objDescTable[i].squareness);
		}
	}
	_affDescriptorOutputPort.setEnvelope(writestamp);
	_affDescriptorOutputPort.write();

	/* output data for tracker initialization */
	Bottle &trackbot = _trackerInitOutputPort.prepare();
	trackbot.clear();
	/* output affordance descriptors */
	for(int i = 0; i < _numObjects; i++)
	{
		if( _objDescTable[i].valid)
		{
			Bottle &objbot = trackbot.addList();
			objbot.clear();
			objbot.addInt(_objDescTable[i].roi_x);
			objbot.addInt(_objDescTable[i].roi_y);
			objbot.addInt(_objDescTable[i].roi_width);
			objbot.addInt(_objDescTable[i].roi_height);
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 0));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 1));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 2));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 3));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 4));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 5));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 6));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 7));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 8));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 9));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 10));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 11));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 12));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 13));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 14));
			objbot.addInt((int)cvQueryHistValue_1D(_objDescTable[i].objHist, 15));
			objbot.addInt(_objDescTable[i].v_min);
			objbot.addInt(_objDescTable[i].v_max);
			objbot.addInt(_objDescTable[i].s_min);
		}
	}

	_trackerInitOutputPort.setEnvelope(writestamp);
	_trackerInitOutputPort.write();

	/* output data to initialize the tracker on the selected object (if any)*/
	if(userselection != -1)
	{
		Bottle &bot = _trackerInitSingleObjOutputPort.prepare();
		bot.clear();
		bot.addInt(_objDescTable[userselection].roi_x);
		bot.addInt(_objDescTable[userselection].roi_y);
		bot.addInt(_objDescTable[userselection].roi_width);
		bot.addInt(_objDescTable[userselection].roi_height);
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 0));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 1));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 2));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 3));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 4));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 5));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 6));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 7));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 8));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 9));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 10));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 11));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 12));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 13));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 14));
		bot.addInt((int)cvQueryHistValue_1D(_objDescTable[userselection].objHist, 15));
		bot.addInt(_objDescTable[userselection].v_min);
		bot.addInt(_objDescTable[userselection].v_max);
		bot.addInt(_objDescTable[userselection].s_min);
		_trackerInitSingleObjOutputPort.setEnvelope(writestamp);
		_trackerInitSingleObjOutputPort.write();
	}

	/* output the original image */
	ImageOf<PixelRgb> &yarpRawOutputImage = _rawImgOutputPort.prepare();
	yarpRawOutputImage = _yarpRawImg;
	_rawImgOutputPort.setEnvelope(writestamp);
	_rawImgOutputPort.write();
	
	/* output image to view results */
	ImageOf<PixelRgb> &yarpOutputImage = _viewImgOutputPort.prepare();
	yarpOutputImage = _yarpViewImg;
	_viewImgOutputPort.setEnvelope(writestamp);
	_viewImgOutputPort.write();
  	return true;
}
