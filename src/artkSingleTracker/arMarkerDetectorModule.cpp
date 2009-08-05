// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 Alexandre Bernardino, Vislab, IST/ISR.
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>

#include "arMarkerDetectorModule.h"

#include <string>
using namespace std;



ARMarkerDetectorModule::ARMarkerDetectorModule()
{
    _object_center[0] = 0.0;
    _object_center[1] = 0.0;
    _reference_found = false;
}

ARMarkerDetectorModule::~ARMarkerDetectorModule()
{
}


bool ARMarkerDetectorModule::open(Searchable& config)
{
    if (config.check("help","if present, display usage message")) {
        printf("Call with --file configFile.ini\n");
        return false;
    }
    //need to initialize _xsize, _ysize, _thresh, _camerafile, _objectfile, _objectnum, _object

    _xsize = config.check("xsize", 640, "image width").asInt();
    _ysize = config.check("ysize", 480, "image height").asInt();
    _thresh = config.check("thresh", 100, "threshold for binarization").asInt();

    // check the application path option
	string strAppPath = config.check("appPath", "", "Absolute path to the application folder (string).").asString().c_str();
	if(!config.check("appPath")){
		cout << "Please specify the configuration option 'appPath'" << endl;
		return false;
	}

    string strCamConfigFile = config.check("camera","camera_para.dat","Name of the configuration file containing the camera parameters (string)").asString().c_str();
    string strCamConfigPath = strAppPath + string("/conf/") + strCamConfigFile;

    string strObjConfigFile = config.check("object","object_data","Name of the configuration file containing the object definitions (string)").asString().c_str();
    string strObjConfigPath = strAppPath + string("/conf/") + strObjConfigFile;

    /* load in the object data - trained markers and associated bitmap files */
    if( (_object=read_objectdata(strObjConfigPath.c_str(),&_object_num)) == NULL )     {
        cout << "Could not open object definition file " << strObjConfigPath << endl;
        return false;
    }

    ARParam  wparam;
    /* set the initial camera parameters */
    if( arParamLoad(strCamConfigPath.c_str(), 1, &wparam) < 0 ) {
       cout << "Could not open camera definition file " << strCamConfigPath << endl;
        return false;
    }
    arParamChangeSize( &wparam, _xsize, _ysize, &cparam );
    arInitCparam( &cparam );

    printf("*** Camera Parameter ***\n");
    arParamDisp( &cparam );


    CvSize sz = cvSize(_xsize,_ysize);
	_frame = cvCreateImage(sz, 8, 4 ); //ARTOOLKIT ON WINDOWS REQUIRES BGRA

    _imgPort.open(getName("image"));
    _configPort.open(getName("conf"));
    _targetrelativePort.open(getName("target"));
	_viewPort.open(getName("view"));
	_coordsPort.open(getName("coords"));
    attach(_configPort, true);
    return true;
}

bool ARMarkerDetectorModule::close()
{
    _imgPort.close();
    _configPort.close();
    _targetrelativePort.close();
    return true;
}

bool ARMarkerDetectorModule::interruptModule(){
    
    _imgPort.interrupt();
    _configPort.interrupt();
    _targetrelativePort.interrupt();
    return true;
}

bool ARMarkerDetectorModule::updateModule()
{
    int i, j, k;
    ImageOf<PixelRgb> *yrpImgIn;
	static int cycles = 0;

	
	
    yrpImgIn = _imgPort.read();
    if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;

	if(cycles == 0)
		_timestart = yarp::os::Time::now();
	cycles++;

    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    

    //MUST CONVERT TO BGRA
    IplImage *iplimg = (IplImage*)yrpImgIn->getIplImage();
    cvCvtColor(iplimg,_frame,CV_RGB2BGRA);
    dataPtr = (ARUint8*)_frame->imageData;
    myFrames.currTickTimeIncr();

    /* detect the markers in the video frame */
    if( arDetectMarker(dataPtr, _thresh, &marker_info, &marker_num) < 0 ) 
    {
        cout << "Unknown error in arDetectMarker" << endl;    
        return false;
    }
    
    /* check for object visibility */
    for( i = 0; i < _object_num; i++ ) {
        //cout << "Checking object " << i << endl;
        _object[i].visible = 0;
        k = -1;
        for( j = 0; j < marker_num; j++ ) {
            if( _object[i].id == marker_info[j].id ) {
                if( k == -1 ) k = j;
                else {
                    if( marker_info[k].cf < marker_info[j].cf ) k = j;
                }
            }
        }
        if( k == -1 ) continue;
		// get the transformation between the marker and the real camera
        arGetTransMat(&marker_info[k], _object_center, _object[i].marker_width, _object[i].trans);

		//the markers are not in the same order as the objects
		_object[i].markerindex = k;

        _object[i].visible = 1;
        _object[i].visible = 1;
    }

	

    double distance;
    double angle;
	Vector &objData=_targetrelativePort.prepare();
	objData.size(5);
    double xnorm = 0.0, ynorm = 0.0;
    objData[2] = 'r';  //relative mode
	objData[3] = 'p';  //pursuit
	objData[4] = 0;    //not used    
	ImageOf<PixelRgb> &yrpImgOut = _viewPort.prepare();
	yrpImgOut = *yrpImgIn;
	double px;
	double py;
    if(_object[0].visible) {    //found target
		px = marker_info[_object[0].markerindex].pos[0];
		py = marker_info[_object[0].markerindex].pos[1];
        cout << "Detected Target: X= " << px << " Y= " << py << endl;
		xnorm = px/(_xsize/2.0)-1.0;
		ynorm = py/(_ysize/2.0)-1.0;
		PixelRgb pix = PixelRgb(255,0,0);
		yarp::sig::draw::addCrossHair(yrpImgOut, pix, px, py, 10);
		Bottle &coords = _coordsPort.prepare();
		coords.clear();
		coords.addInt(px);
		coords.addInt(py);
		_coordsPort.write();
    }
	else
	{
		xnorm = -1000.0; //Invalid values - indicate tracker failure
		ynorm = -1000.0;
	}
	objData[0] = xnorm;
    objData[1] = ynorm;
    _targetrelativePort.write();   
	_viewPort.write();

	//report the number of detected markers and targets.
	cout << "Detected " << marker_num << " markers. ";
	if( _object[0].visible)
		cout << "The target was detected." << endl;
	else
		cout << "The target was not detected." << endl;


	//report the frame rate
	if(cycles % 100 == 0)
	{
		double cps = ((double)cycles)/(yarp::os::Time::now() - _timestart);
		printf("fps: %02.2f\n", cps);
	}
    return true;
}

void ARMarkerDetectorModule::copytrans(double src[3][4], double dst[3][4])
{
    int i, j;
    for(i=0; i < 3; i++)
        for(j=0; j<4; j++)
            dst[i][j] = src[i][j];
    return;
}

void ARMarkerDetectorModule::calibtrans(double src[3][4], double dst[3][4])
{
    return;
}
