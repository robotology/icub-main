// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Alexandre Bernardino, Vislab, IST/ISR.
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <string>
#include <iostream>
using namespace std;


#include "artkpTrackSingleMarkerModule.h"
using namespace ARToolKitPlus;

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
using namespace yarp::os;




ARTKPTrackSingleMarkerModule::ARTKPTrackSingleMarkerModule()
{
    _object_center[0] = 0.0;
    _object_center[1] = 0.0;
	_reference_found = false;
	tracker = NULL;
	useBCH = false;
}

ARTKPTrackSingleMarkerModule::~ARTKPTrackSingleMarkerModule()
{
	if(tracker!=NULL)
		delete tracker;
}


bool ARTKPTrackSingleMarkerModule::open(Searchable& config)
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

    _xsize = rf.find("xsize").asInt();
    _ysize = rf.find("ysize").asInt();
    _thresh = rf.find("thresh").asInt();
	_object_width = (ARFloat)rf.find("pattsize").asDouble();

	// create a tracker that does:
    //  - 6x6 sized marker images
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 1 pattern
    //  - can detect a maximum of 8 patterns in one image
	tracker = new ARToolKitPlus::TrackerSingleMarkerImpl< 6, 6, 6, 1, 8>(_xsize,_ysize);
	const char* description = tracker->getDescription();
	printf("ARToolKitPlus compile-time information:\n%s\n\n", description);
    // set a logger so we can output error messages
    //
    tracker->setLogger(&logger);
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
	//tracker->setLoadUndistLUT(true);

	// name of the camera calibration file
    ConstString strCamConfigPath=rf.findFile("camera");

	// load a camera file. two types of camera files are supported:
    //  - Std. ARToolKit
    //  - MATLAB Camera Calibration Toolbox
    //if(!tracker->init("data/LogitechPro4000.dat", 1.0f, 1000.0f))            // load std. ARToolKit camera file
    if(!tracker->init(strCamConfigPath.c_str(), 1.0f, 1000.0f))        // load MATLAB file
	{
		printf("ERROR: init() failed\n");
		//delete cameraBuffer;
		delete tracker;
		tracker = NULL;
		return false;
	}

    // define size of the marker
    tracker->setPatternWidth(_object_width);

	// the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(useBCH ? 0.125f : 0.250f);

    // set a threshold. alternatively we could also activate automatic thresholding
    //tracker->setThreshold(150);
	tracker->activateAutoThreshold(true);
	tracker->activateVignettingCompensation(true);
	tracker->setNumAutoThresholdRetries(3);

    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);

    // RPP is more robust than ARToolKit's standard pose estimator
    tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);



    CvSize sz = cvSize(_xsize,_ysize);
	_frame = cvCreateImage(sz, 8, 1); //gray scale image to process

    _imgPort.open(getName("image"));
    _configPort.open(getName("conf"));
    _targetrelativePort.open(getName("target"));
	_viewPort.open(getName("view"));
	_coordsPort.open(getName("coords"));
	_targetPosPort.open(getName("pos"));
    attach(_configPort, true);
    return true;
}

bool ARTKPTrackSingleMarkerModule::close()
{
    _imgPort.close();
    _configPort.close();
    _targetrelativePort.close();
	_viewPort.close();
	_coordsPort.close();
	_targetPosPort.close();

	if(tracker!=NULL)
		delete tracker;

	// also delete the allocated image...

    return true;
}

bool ARTKPTrackSingleMarkerModule::interruptModule(){
    
    _imgPort.interrupt();
    _configPort.interrupt();
    _targetrelativePort.interrupt();
	_viewPort.interrupt();
	_coordsPort.interrupt();
	_targetPosPort.interrupt();
    return true;
}

bool ARTKPTrackSingleMarkerModule::updateModule()
{
    ImageOf<PixelRgb> *yrpImgIn;
	static int cycles = 0;

		
    yrpImgIn = _imgPort.read();
    if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;

	if(cycles == 0)
		_timestart = yarp::os::Time::now();
	cycles++;

    unsigned char   *dataPtr;
	ARMarkerInfo    *marker_info;
    int             marker_num;
    

    //MUST CONVERT TO GRAYSCALE ??
    IplImage *iplimg = (IplImage*)yrpImgIn->getIplImage();
    cvCvtColor(iplimg,_frame,CV_RGB2GRAY);
    dataPtr = (unsigned char*)_frame->imageData;
  //  myFrames.currTickTimeIncr();

    /* detect the markers in the video frame */
	int markerId = tracker->calc(dataPtr, -1, true, &marker_info, &marker_num);
    float conf = (float)tracker->getConfidence();

	printf("\n\nFound %d markers. Best is marker %d  (confidence %d%%)\n  ", marker_num, markerId, (int(conf*100.0f)));

	ImageOf<PixelRgb> &yrpImgOut = _viewPort.prepare();
	yrpImgOut = *yrpImgIn;
	if(conf > 0.3) // 30 percent confidence
	{
		//get the index of the marker
		int i;
		for(i = 0; i < marker_num; i++)
			if(marker_info[i].id == markerId)
				break;
		if( i < marker_num) //got it
		{
			//process and write 2D coords
			int px;
			int py;
			px = (int)marker_info[i].pos[0];
			py = (int)marker_info[i].pos[1];
			Bottle &coords = _coordsPort.prepare();
			coords.clear();
			coords.addInt(px);
			coords.addInt(py);
			_coordsPort.write();
			
			//draw cross on detected location and stream out
			PixelRgb pix = PixelRgb(255,0,0);
			yarp::sig::draw::addCrossHair(yrpImgOut, pix, px, py, 20);
			

			//get the transformation matrix and write it to port
			ARFloat _transf[3][4]; 
			tracker->arGetTransMat(&marker_info[i], _object_center, _object_width, _transf);
			yarp::sig::Vector &objData=_targetrelativePort.prepare();
			objData.resize(16);
			double t00 = objData[0]  = tracker->getModelViewMatrix()[0]; //_transf[0][0];
			double t01 = objData[1]  = tracker->getModelViewMatrix()[1]; //_transf[0][1];
			double t02 = objData[2]  = tracker->getModelViewMatrix()[2]; //_transf[0][2];
			double t03 = objData[3]  = tracker->getModelViewMatrix()[3]; //_transf[0][3];
			double t10 = objData[4]  = tracker->getModelViewMatrix()[4]; //_transf[1][0];
			double t11 = objData[5]  = tracker->getModelViewMatrix()[5]; //_transf[1][1];
			double t12 = objData[6]  = tracker->getModelViewMatrix()[6]; //_transf[1][2];
			double t13 = objData[7]  = tracker->getModelViewMatrix()[7]; //_transf[1][3];
			double t20 = objData[8]  = tracker->getModelViewMatrix()[8]; //_transf[2][0];
			double t21 = objData[9]  = tracker->getModelViewMatrix()[9]; //_transf[2][1];
			double t22 = objData[10] = tracker->getModelViewMatrix()[10];//_transf[2][2];
			double t23 = objData[11] = tracker->getModelViewMatrix()[11];//_transf[2][3];
			double t30 = objData[12] = tracker->getModelViewMatrix()[12];//_transf[2][3];
			double t31 = objData[13] = tracker->getModelViewMatrix()[13];//_transf[2][3];
			double t32 = objData[14] = tracker->getModelViewMatrix()[14];//_transf[2][3];
			double t33 = objData[15] = tracker->getModelViewMatrix()[15];//_transf[2][3];

			_targetrelativePort.write();  

			if(t03 < -20.0)
				printf("HEEELLLPPPPPP!!!!\n");
			Bottle &targetPos = _targetPosPort.prepare();
			targetPos.clear();
            targetPos.addDouble(t00);
            targetPos.addDouble(t01);
            targetPos.addDouble(t02);
            targetPos.addDouble(t03);
            targetPos.addDouble(t10);
            targetPos.addDouble(t11);
            targetPos.addDouble(t12);
            targetPos.addDouble(t13);
            targetPos.addDouble(t20);
            targetPos.addDouble(t21);
            targetPos.addDouble(t22);
            targetPos.addDouble(t23);
			targetPos.addDouble(t30);
			targetPos.addDouble(t31);
			targetPos.addDouble(t32);
			targetPos.addDouble(t33);
			_targetPosPort.write();
		}
		else // should not fall here
		{
			printf("SOMETHING VERY WRONG HAPPENED HERE: alex@isr.ist.utl.pt\n");
		}
	}
		
	_viewPort.write();
	
		
	//report the frame rate
	if(cycles % 100 == 0)
	{
		double cps = ((double)cycles)/(yarp::os::Time::now() - _timestart);
		printf("fps: %02.2f\n", cps);
	}
    return true;
}

void ARTKPTrackSingleMarkerModule::copytrans(double src[3][4], double dst[3][4])
{
    int i, j;
    for(i=0; i < 3; i++)
        for(j=0; j<4; j++)
            dst[i][j] = src[i][j];
    return;
}

void ARTKPTrackSingleMarkerModule::calibtrans(double src[3][4], double dst[3][4])
{
    return;
}



int main(int argc, char *argv[]) {

    Network yarp;
    ARTKPTrackSingleMarkerModule module;
    module.setName("/artkpsingle"); // set default name of module
    return module.runModule(argc,argv);
}
