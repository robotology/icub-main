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
using namespace yarp::os;

#include <iCub/iKinFwd.h>
using namespace iKin;

#include<yarp/dev/ControlBoardInterfaces.h>

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
	cout <<"Object width : " << _object_width <<endl;

	robot = rf.find("robot").asString();
	eyeName = rf.find("eye").asString();
	string threeDPosPortName = rf.find("3DPosPort").asString();

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
    //tracker->setThreshold(_thresh);
	tracker->activateAutoThreshold(true);
	cout << "Threshold : " << tracker->getThreshold() << endl;

	tracker->activateVignettingCompensation(true);
	tracker->setImageProcessingMode(IMAGE_FULL_RES);
	tracker->setNumAutoThresholdRetries(6);

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
	threeDPosPort.open(getName(threeDPosPortName.c_str()));
    attach(_configPort, true);

	drivers["head"] = CreatePolyDriver("head");
	drivers["torso"] = CreatePolyDriver("torso");

    return true;
}

bool ARTKPTrackSingleMarkerModule::close()
{
	drivers["head"]->close();
	drivers["torso"]->close();

	delete drivers["head"];
	delete drivers["torso"];

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

	//printf("\nFound %d markers.(confidence %d%%)\n  ", marker_num, (int(conf*100.0f)));
	//for(int i=0; i<marker_num ; ++i)
	//{
	//	//printf("marker %d, ID = %d, pos = (%f,%f)\n  ", i, marker_info[i].id, marker_info[i].pos[0], marker_info[i].pos[1]);
	//}

	ImageOf<PixelRgb> &yrpImgOut = _viewPort.prepare();
	yrpImgOut = *yrpImgIn;

	Bottle &visionPosBottle = _coordsPort.prepare();
	Bottle &rootPosBottle = threeDPosPort.prepare();
	visionPosBottle.clear();
	rootPosBottle.clear();

	if(conf > 0.3) // 30 percent confidence
	{
		//Bottle &coords = _coordsPort.prepare();
		//coords.clear();

		//Bottle &targetPos = _targetPosPort.prepare();
		//targetPos.clear();

		sig::Matrix transformationMatrix = GetTransformationMatrix();

		Bottle matrix = _targetPosPort.prepare();
		cout << "transformation matrix : " << transformationMatrix.toString() << endl;
		string strMatrix = transformationMatrix.toString();
		matrix.addString(strMatrix.c_str());
		_targetPosPort.write();

		//cout << "Transformation matrix : " << transformationMatrix.toString() << endl;

		for(int i=0; i<marker_num; ++i)
		{
			//process and write 2D coords
			int px = (int)marker_info[i].pos[0];
			int py = (int)marker_info[i].pos[1];

			/*Bottle twoDBottle;
			twoDBottle.clear();
			twoDBottle.addInt(marker_info[i].id);
			twoDBottle.addInt(px);
			twoDBottle.addInt(py);
			coords.addList() = twoDBottle;*/


			//draw cross on detected location and stream out
			PixelRgb pix = PixelRgb(255,0,0);
			yarp::sig::draw::addCrossHair(yrpImgOut, pix, px, py, 20);


			//get the transformation matrix and write it to port
			//ARFloat _transf[3][4]; 
			//tracker->arGetTransMat(&marker_info[i], _object_center, _object_width, _transf);
			//yarp::sig::Vector &objData=_targetrelativePort.prepare();
			//objData.resize(16);
			ARFloat _transf[3][4]; 
			tracker->arGetTransMat(&marker_info[i], _object_center, _object_width, _transf);
			//const ARFloat *modelViewMatrix = tracker->getModelViewMatrix();
			//double t00 = objData[0]  = tracker->getModelViewMatrix()[0]; //_transf[0][0];
			//double t01 = objData[1]  = tracker->getModelViewMatrix()[1]; //_transf[0][1];
			//double t02 = objData[2]  = tracker->getModelViewMatrix()[2]; //_transf[0][2];
			//double t03 = objData[3]  = tracker->getModelViewMatrix()[3]; //_transf[0][3];
			//double t10 = objData[4]  = tracker->getModelViewMatrix()[4]; //_transf[1][0];
			//double t11 = objData[5]  = tracker->getModelViewMatrix()[5]; //_transf[1][1];
			//double t12 = objData[6]  = tracker->getModelViewMatrix()[6]; //_transf[1][2];
			//double t13 = objData[7]  = tracker->getModelViewMatrix()[7]; //_transf[1][3];
			//double t20 = objData[8]  = tracker->getModelViewMatrix()[8]; //_transf[2][0];
			//double t21 = objData[9]  = tracker->getModelViewMatrix()[9]; //_transf[2][1];
			//double t22 = objData[10] = tracker->getModelViewMatrix()[10];//_transf[2][2];
			//double t23 = objData[11] = tracker->getModelViewMatrix()[11];//_transf[2][3];
			//double t30 = objData[12] = tracker->getModelViewMatrix()[12];//_transf[2][3];
			//double t31 = objData[13] = tracker->getModelViewMatrix()[13];//_transf[2][3];
			//double t32 = objData[14] = tracker->getModelViewMatrix()[14];//_transf[2][3];
			//double t33 = objData[15] = tracker->getModelViewMatrix()[15];//_transf[2][3];

			/*_targetrelativePort.write(); 

			if(t03 < -20.0)
				printf("HEEELLLPPPPPP!!!!\n");
	
			Bottle threeDBottle;
			threeDBottle.clear();
            threeDBottle.addDouble(t00);
            threeDBottle.addDouble(t01);
            threeDBottle.addDouble(t02);
            threeDBottle.addDouble(t03);
            threeDBottle.addDouble(t10);
            threeDBottle.addDouble(t11);
            threeDBottle.addDouble(t12);
            threeDBottle.addDouble(t13);
            threeDBottle.addDouble(t20);
            threeDBottle.addDouble(t21);
            threeDBottle.addDouble(t22);
            threeDBottle.addDouble(t23);
			threeDBottle.addDouble(t30);
			threeDBottle.addDouble(t31);
			threeDBottle.addDouble(t32);
			threeDBottle.addDouble(t33);
			targetPos.addList() = threeDBottle;*/

			//cout << "VIEW Tranformation matrix : " << threeDBottle.toString() << endl;

			sig::Vector visionPosition(3);
			visionPosition[0] = _transf[0][3];
			visionPosition[1] = _transf[1][3];
			visionPosition[2] = _transf[2][3];

			
			string patchColor = "???";
			if(marker_info[i].id == RED_MARKER_ID)
			{
				patchColor = "red";
			}
			else if (marker_info[i].id == GREEN_MARKER_ID)
			{
				patchColor = "green";
			}

			//if(marker_info[i].id == RED_MARKER_ID)
			//{
			//	markerBottle.addString("red");
			//}
			//else if(marker_info[i].id == GREEN_MARKER_ID)
			//{
			//	markerBottle.addString("green");
			//}
		
			//Bottle markerBottle;
			//markerBottle.addDouble(visionPosition[0]);
			//markerBottle.addDouble(visionPosition[1]);
			//markerBottle.addDouble(visionPosition[2]);
			//if(marker_info[i].id == RED_MARKER_ID)
			//{
			//	markerBottle.addString("red");
			//}
			//else if(marker_info[i].id == GREEN_MARKER_ID)
			//{
			//	markerBottle.addString("green");
			//}
			//threeDPosBottle.addList() = markerBottle;

			Bottle markerBottleVision;
			markerBottleVision.clear();
			markerBottleVision.addDouble(visionPosition[0]);
			markerBottleVision.addDouble(visionPosition[1]);
			markerBottleVision.addDouble(visionPosition[2]);
			markerBottleVision.addString(patchColor.c_str());
			visionPosBottle.addList() = markerBottleVision;


			sig::Vector rootPosition = VisionPositionToRootPosition(visionPosition, transformationMatrix);


			Bottle markerBottleRoot;
			markerBottleRoot.addDouble(rootPosition[0]);
			markerBottleRoot.addDouble(rootPosition[1]);
			markerBottleRoot.addDouble(rootPosition[2]);
			markerBottleRoot.addString(patchColor.c_str());
			rootPosBottle.addList() = markerBottleRoot;
			//ValidatePosition(rootPosition, patchColor);
		}

		//SendValidPosition();
		_coordsPort.write();
		threeDPosPort.write();
		//_coordsPort.write();
		//_targetPosPort.write();
		
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
    module.setName("/artkpmultimarker"); // set default name of module
    return module.runModule(argc,argv);
}

sig::Vector ARTKPTrackSingleMarkerModule::VisionPositionToRootPosition(sig::Vector &visionPosition, const sig::Matrix &transformationMatrix)
{
	visionPosition.resize(4);
	visionPosition[3] = 1;

	//cout << "vision position : " << visionPosition.toString() << endl;


	sig::Vector rootPosition; 
	rootPosition=transformationMatrix * visionPosition;

	//cout << "root position before : " << rootPosition.toString() << endl;

	rootPosition.resize(3);

	//cout << "root position after : " << rootPosition.toString() << endl;

	return (rootPosition);
}

sig::Vector ARTKPTrackSingleMarkerModule::GetJointAngles(string partName)
{
    IPositionControl *pos;
    IEncoders *encs;

    if (!(drivers[partName]->view(pos) && drivers[partName]->view(encs))) 
	{
        printf("Problems acquiring interfaces\n");
        return 0;
    }

	//int nj;
	//encs->getAxes(&nj);//move

	sig::Vector jointAngles(nbJoints[partName]);
	double *encoders = new double[nbJoints[partName]];
	encs->getEncoders(encoders);

	//compatibility problems between the orders of the encoders and the standart torso pitch roll yaw;
	if(partName == "torso")
	{
		jointAngles[0] = encoders[2]* M_PI/180;
		jointAngles[1] = encoders[1]* M_PI/180;
		jointAngles[2] = encoders[0]* M_PI/180;
	}
	else
	{
		for(int i=0; i<nbJoints[partName]; ++i)
		{
			jointAngles[i] = encoders[i]* M_PI/180;//invert torso
		}
	}
	delete[] encoders;
	return jointAngles;
}




PolyDriver *ARTKPTrackSingleMarkerModule::CreatePolyDriver(string partName)
{
	Property options;
    options.put("device", "remote_controlboard");

	string localPortName = getName( partName.c_str());
	//string localPortName = (string)threeDPosPort.getName() + "/" + partName;
	options.put("local", localPortName.c_str());   //local port names

	string remotePortName = "/" + robot + "/" + partName;
	options.put("remote", remotePortName.c_str());//where we connect to

    // create a device
    PolyDriver *driver = new PolyDriver(options);
    if (!driver->isValid()) 
	{
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return NULL;
    }

	IPositionControl *pos;
    IEncoders *encs;

    if (!(driver->view(pos) && driver->view(encs))) 
	{
        printf("Problems acquiring interfaces\n");
        return NULL;
    }

	int nj;
	encs->getAxes(&nj);
	nbJoints[partName] = nj;

	return driver;
}



sig::Matrix ARTKPTrackSingleMarkerModule::GetTransformationMatrix()
{
	iCubEye *eye = new iCubEye(eyeName); //we want to play with the left eye of the bot.
    iKinChain chainEyeL=*(eye->asChain());
	chainEyeL.releaseLink(0);
	chainEyeL.releaseLink(1);
	chainEyeL.releaseLink(2);
	chainEyeL.releaseLink(3);
	chainEyeL.releaseLink(4);

	sig::Vector head = GetJointAngles("head");
	sig::Vector torso = GetJointAngles("torso");

	//cout << "Joints Angles torso : " << torso.toString() << endl;
	cout << "Joints Angles head : " << head.toString() << endl;

	
	//get data and convert from degrees to radians
	//head[0]=encs[0]*M_PI/180;
	//head1=(head->get(1)).asDouble()*M_PI/180;
	//head2=(head->get(2)).asDouble()*M_PI/180;
	//head3=(head->get(3)).asDouble()*M_PI/180;
	//head4=(head->get(4)).asDouble()*M_PI/180;
	//head5=(head->get(5)).asDouble()*M_PI/180;
	////cout<<"Received head values: "<<head0<<" "<<head1<<" "<<head2<<" "<<head3<<" "<<head4<<" "<<head5<<endl;
	//receivedHead = true;
	//}

	//Bottle * torso=_inputTorsoPort.read(false);
	//if(torso!=NULL)
	//{
	////get data and convert from degrees to radians
	//torso0=(torso->get(0)).asDouble()*M_PI/180;
	//torso1=(torso->get(1)).asDouble()*M_PI/180;
	//torso2=(torso->get(2)).asDouble()*M_PI/180;
	////cout<<"received torso values: "<<torso0<<" "<<torso1<<" "<<torso2<<endl;
	//receivedTorso = true;
	//}

	//Bottle * ballPositionIn=_inputBallPositionPort.read(false);
	//if(ballPositionIn!=NULL)
	//{
	////get the data, already in meters
	//ballPositionInX=(ballPositionIn->get(0)).asDouble();
	//ballPositionInY=(ballPositionIn->get(1)).asDouble();
	//ballPositionInZ=(ballPositionIn->get(2)).asDouble();
	//ballPositionInGood=(ballPositionIn->get(6)).asDouble();
	//  if( gsl_isnan(ballPositionInX) || gsl_isnan(ballPositionInY) || gsl_isnan(ballPositionInZ))
	//  {
	//  ballPositionInGood=0;
	//  }
	////cout<<"Received ballPositionIn values: "<<ballPositionInX<<" "<<ballPositionInY<<" "<<ballPositionInZ<<" "<<ballPositionInGood<<endl;
	//receivedBallPosition = true;


	int headSize = head.length();
	int torsoSize = torso.length();

	sig::Vector allAngles(headSize + torsoSize -1);
	for(int i=0; i<torsoSize; i++)
	{
		allAngles[i] = (torso[i]);
	}
	for(int i=0; i<headSize-2; i++)
	{
		allAngles[i+torsoSize] = head[i];
	}

	allAngles[allAngles.length()-1]=head[4]+head[5]/2;

	//cout << "Joints Angles : " << allAngles.toString() << endl;

	//sig::Matrix H=chainEyeL.getH(allAngles);

	//cout << "Transformation Matrix : " << H.toString() << endl;


    return chainEyeL.getH(allAngles);
}


void ARTKPTrackSingleMarkerModule::ValidatePosition( sig::Vector &position, string color)
{
	bool found = false;
	for(unsigned int i=0; i<positionsBuffer.size(); ++i)
	{
		if(color != positionsBuffer[i].color)
		{
			continue;
		}
		sig::Vector &currentPosition = positionsBuffer[i].position;
		sig::Vector distVec = position - currentPosition;
		double distance2 = distVec[0]*distVec[0] + distVec[1]*distVec[1] + distVec[1]*distVec[1];
		//cout << "This position : " << position.toString() << endl;
		//cout << "distance : " << distance2 << endl;
		if(distance2 < EPSILON_POS)
		{
			positionsBuffer[i].counter++;
			found = true;
			break;
		}
	}


	for(unsigned int i=0; i<positionsBuffer.size(); ++i)
	{
		if(positionsBuffer[i].age > MAX_AGE)
		{
			positionsBuffer.erase(positionsBuffer.begin() + i);
		}
		else
		{
			positionsBuffer[i].age++;
		}
	}


	if(!found)
	{
		PositionValid newPosition;
		newPosition.color = color;
		newPosition.counter = 0;
		newPosition.age = 0;
		newPosition.position = position;
		positionsBuffer.push_back(newPosition);
		//cout << "\n ==== added new position : " << newPosition.position.toString() << "====" << endl;
	}
}

void ARTKPTrackSingleMarkerModule::SendValidPosition(void)
{
	Bottle &threeDPosBottle = threeDPosPort.prepare();
	threeDPosBottle.clear();
	bool dataToSend = false;
	for(unsigned int i=0; i<positionsBuffer.size(); ++i)
	{
		if(positionsBuffer[i].counter > MIN_COUNTER_VALID)
		{
			Bottle markerBottle;
			markerBottle.addDouble(positionsBuffer[i].position[0]);
			markerBottle.addDouble(positionsBuffer[i].position[1]);
			markerBottle.addDouble(positionsBuffer[i].position[2]);
			markerBottle.addString(positionsBuffer[i].color.c_str());
			threeDPosBottle.addList() = markerBottle;
			positionsBuffer.erase(positionsBuffer.begin() + i);
			i--;
			dataToSend = true;
		}
	}
	if(dataToSend)
	{
		threeDPosPort.write();
	}
}