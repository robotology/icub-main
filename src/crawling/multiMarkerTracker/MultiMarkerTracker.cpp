// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Sebastien Gay, BIRG, EPFL.
 * Courtesy to Alex Bernardino, Vislab, IST/ISR, who's work this code is inspired of.
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include <iostream>
using namespace std;

#include "MultiMarkerTracker.h"

#include <ARToolKitPlus/TrackerSingleMarkerImpl.h>

#include <iCub/iKinFwd.h>
using namespace iKin;

#include<yarp/dev/ControlBoardInterfaces.h>

MultiMarkerTracker::MultiMarkerTracker(): frame(NULL)
{

}

MultiMarkerTracker::~MultiMarkerTracker()
{

}

double MultiMarkerTracker::getPeriod()
{
	return 0.1;
}

bool MultiMarkerTracker::open(Searchable& config)
{
#ifdef _DEBUG
	log.open("3DPos.dat");
	torsoEncoders.open("torso_encoders.dat");
	headEncoders.open("head_encoders.dat");
	numIter = 0;
#endif
    if (config.check("help","if present, display usage message")) {
        printf("Call with --from config.ini\n");
        return false;
    }
    
    ResourceFinder rf;
    if (config.check("context"))
	{
        rf.setDefaultContext(config.find("context").asString());
	}
    else
	{
        rf.setDefaultContext(getName());
	}

    if (config.check("from"))
	{
        rf.setDefaultConfigFile(config.find("from").asString());
	}
    else
	{
        rf.setDefaultConfigFile("config.ini");
	}
	rf.configure("ICUB_ROOT",0,NULL);
	//initializes the context and config files

	//gets parameters 
	parameters["robot"] =  new Value(rf.find("robot"));
	parameters["eye"] = new Value(rf.find("eye"));
	parameters["object_width"] = new Value(rf.find("pattsize"));

	objectOffset.push_back(rf.find("object_offset_x").asDouble());
	objectOffset.push_back(rf.find("object_offset_y").asDouble());
	objectOffset.push_back(rf.find("object_offset_z").asDouble());

	cout << "object offset : " << objectOffset.toString() << endl;

	//initializes the tracker
	if(!initTracker(rf))
	{
		return false;
	}

	//opens some ports
	imagePort.open(getName("image"));
	viewPort.open(getName("view"));
	rootPosPort.open(getName("RootPos"));
	eyePosPort.open(getName("EyePos"));

	//creates the polydriver used to get the robot current kinematic configuration.
	//drivers["head"] = CreatePolyDriver("head");
	//drivers["torso"] = CreatePolyDriver("torso");

	OpenEncodersPorts();

    return true;
}

void MultiMarkerTracker::OpenEncodersPorts()
{
	string port_name = "/MultiMarkerTracker/head_encoders";
	headEncodersPort.open(port_name.c_str());
	bool connected = false;
	cout << "Connecting to /icub/head/state:o ..." << endl;
	for(int i=0; i<3; ++i)
	{
		cout << "Attempt " << i << " ..." << endl;
		if(Network::connect("/icub/head/state:o", port_name.c_str()))
		{
			connected = true;
			break;
		}
	}
	if(connected)
	{
		cout << "Connected." << endl;
	}
	else
	{
		cout << "Failed to connect to port /icub/head/state:o." << endl
			<< "iCubInterface must be running for this module to work" << endl;
		return;
	}


	port_name = "/MultiMarkerTracker/torso_encoders";
	torsoEncodersPort.open(port_name.c_str());
	connected = false;
	cout << "Connecting to /icub/torso/state:o ..." << endl;
	for(int i=0; i<3; ++i)
	{
		cout << "Attempt " << i << " ..." << endl;
		if(Network::connect("/icub/torso/state:o", port_name.c_str()))
		{
			connected = true;
			break;
		}
	}
	if(connected)
	{
		cout << "Connected." << endl;
	}
	else
	{
		cout << "Failed to connect to port /icub/torso/state:o." << endl
			<< "iCubInterface must be running for this module to work" << endl;
		return;
	}
}


bool MultiMarkerTracker::initTracker(ResourceFinder &rf)
{
	int xSize = rf.find("xsize").asInt();
	int ySize = rf.find("ysize").asInt();
	int threshold =  rf.find("threshold").asInt();

	cout <<"Object width : " << parameters["object_width"]->asDouble() <<endl;

	// create a tracker that does:
    //  - 6x6 sized marker images
    //  - samples at a maximum of 6x6
    //  - works with luminance (gray) images
    //  - can load a maximum of 1 pattern
    //  - can detect a maximum of 8 patterns in one image
	tracker = new ARToolKitPlus::TrackerSingleMarkerImpl< 6, 6, 6, 1, 8>(xSize,ySize);
	const char* description = tracker->getDescription();
	printf("ARToolKitPlus compile-time information:\n%s\n\n", description);

    // set a logger so we can output error messages
    tracker->setLogger(&logger);

	//sets the pixel format to greyscale
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
	//tracker->setLoadUndistLUT(true);

	//sets the camera according to weather it's a simulated or a real camera.
	bool simulation = false;//(bool)rf.find("simulation").asInt();
	if(!simulation)
	{
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
	}
	else
	{
		//do the OpenGL camera setup
		/*glMatrixMode(GL_PROJECTION)
		glLoadMatrixf(tracker->getProjectionMatrix());*/
	}

    // define size of the marker
    tracker->setPatternWidth((ARFloat)parameters["object_width"]->asDouble());

	// the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(0.250f);

    // set a threshold. alternatively we could also activate automatic thresholding
    //tracker->setThreshold(threshold);
	tracker->activateAutoThreshold(true);
	
	//activates processing on the full resolution image for detection of far away markers
	tracker->setImageProcessingMode(IMAGE_FULL_RES);

	//sets a high number of retries to detect more markers
	tracker->setNumAutoThresholdRetries(5);

	if(simulation)
	{
		//desactives vignetting compensation
		tracker->activateVignettingCompensation(false);

		// let's use lookup-table undistortion for high-speed
		// note: LUT only works with images up to 1024x1024
		tracker->setUndistortionMode(ARToolKitPlus::UNDIST_NONE);
	}
	else
	{
		//actives vignetting compensation
		tracker->activateVignettingCompensation(true);

		// let's use lookup-table undistortion for high-speed
		// note: LUT only works with images up to 1024x1024
		tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
	}

    // RPP is more robust than ARToolKit's standard pose estimator
	tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);

    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);

	//creates the image
    CvSize sz = cvSize(xSize,ySize);
	frame = cvCreateImage(sz, 8, 1); //gray scale image to process

	return true;
}


bool MultiMarkerTracker::close()
{
	for(map<string, PolyDriver *>::iterator it = drivers.begin(); it!= drivers.end(); it++)
	{
		it->second->close();
		delete it->second;
	}

	for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}

	imagePort.close();
	viewPort.close();
	rootPosPort.close();
	eyePosPort.close();

	if(tracker != NULL)
	{
		delete tracker;
	}
	if(frame != NULL)
	{
		cvReleaseImage(&frame);
	}

#ifdef _DEBUG
	log.close();
	headEncoders.close();
	torsoEncoders.close();
#endif

    return true;
}

bool MultiMarkerTracker::interruptModule()
{
    for(map<string, PolyDriver *>::iterator it = drivers.begin(); it!= drivers.end(); it++)
	{
		it->second->close();
		delete it->second;
	}

	for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}

    imagePort.close();
	viewPort.close();
	rootPosPort.close();
	eyePosPort.close();

#ifdef _DEBUG
	log.close();
	headEncoders.close();
	torsoEncoders.close();
#endif
    return true;
}

bool MultiMarkerTracker::updateModule()
{
    ImageOf<PixelRgb> *yarpImageIn;
	static int cycles = 0;

	sig::Matrix robotTransformationMatrix = GetRobotTransformationMatrix();
  
	yarpImageIn = imagePort.read();
    if (yarpImageIn == NULL) // this is the case if the module is requested to quit while waiting for image
	{
        return true;
	}

    unsigned char *dataPtr;
	ARMarkerInfo *markerInfo;
    int markerNum;
    
    IplImage *iplImg = (IplImage*)yarpImageIn->getIplImage();
    cvCvtColor(iplImg,frame,CV_RGB2GRAY);
    dataPtr = (unsigned char*)frame->imageData;

    /* detect the markers in the video frame */
	int markerId = tracker->calc(dataPtr, -1, true, &markerInfo, &markerNum);
    float confidence = (float)tracker->getConfidence();

	ImageOf<PixelRgb> &yarpImageOut = viewPort.prepare();
	yarpImageOut = *yarpImageIn;

	Bottle &rootPosBottle = rootPosPort.prepare();
	Bottle &eyePosBottle = eyePosPort.prepare();
	eyePosBottle.clear();
	rootPosBottle.clear();

	if(confidence > 0.3) // 30 percent confidence
	{
		//gets the transformation matrix of the eye (camera) in the root reference frame of the robot.
		//sig::Matrix robotTransformationMatrix = GetRobotTransformationMatrix();

		vector<yarp::sig::Vector> positions;

		for(int i=0; i<markerNum; ++i)
		{
			//handles invalid detection
			if(markerInfo[i].id < 0)
			{
				continue;
			}

			//process 2D coords
			int px = (int)markerInfo[i].pos[0];
			int py = (int)markerInfo[i].pos[1];

			//draw cross on detected location and stream out
			PixelRgb pix = PixelRgb(255,0,0);
			yarp::sig::draw::addCrossHair(yarpImageOut, pix, px, py, 20);

			ARFloat arTransf[3][4]; 
			ARFloat objectCenter[2];
			objectCenter[0] = 0;
			objectCenter[1] = 0;

			//gets the transformation matrix of the marker in the camera frame
			tracker->arGetTransMat(&markerInfo[i], objectCenter, (ARFloat)parameters["object_width"]->asDouble(), arTransf);


			if( sqrt( (double)(arTransf[0][3]*arTransf[0][3] + arTransf[1][3]*arTransf[1][3] + arTransf[2][3]*arTransf[2][3]) ) > 10000)
			{
				continue;
			}

			sig::Matrix cameraTransformationMatrix(4,4);
			for(int j=0; j<3; ++j)
			{
				for(int k=0; k<4; ++k)
				{
					cameraTransformationMatrix[j][k] = arTransf[j][k];
				}
			}
			cameraTransformationMatrix[3][0] = 0;
			cameraTransformationMatrix[3][1] = 0;
			cameraTransformationMatrix[3][2] = 0;
			cameraTransformationMatrix[3][3] = 1;

			//cout<<endl<<endl<< "========== TRANFORMATION MATRIX ==========" << endl;
			//cout<<cameraTransformationMatrix.toString()<<endl;

			objectOffset.resize(4);
			objectOffset[3] = 1;

			sig::Vector eyePosition(4);

			//the real object position is a translation of the marker position.
			//gets the real object position
			eyePosition = cameraTransformationMatrix * objectOffset;

			//sig::Vector rootPosition = robotTransformationMatrix * eyePosition;
			//sig::Vector rootPosition = objectPosition;
			
			Bottle marker;
			marker.addDouble(eyePosition[0]);
			marker.addDouble(eyePosition[1]);
			marker.addDouble(eyePosition[2]);
			marker.addInt(markerInfo[i].id);
			eyePosBottle.addList() = marker;
		}

		for(int i=0; i<eyePosBottle.size(); ++i)
		{
			Bottle *current = eyePosBottle.get(i).asList();
			yarp::sig::Vector rootPosition(4);
			rootPosition[0] = current->get(0).asDouble();
			rootPosition[1] = current->get(1).asDouble();
			rootPosition[2] = current->get(2).asDouble();
			rootPosition[3] = 1;
			rootPosition = robotTransformationMatrix * rootPosition;
			Bottle marker;
			marker.addDouble(rootPosition[0]);
			marker.addDouble(rootPosition[1]);
			marker.addDouble(rootPosition[2]);
			marker.addInt( current->get(3).asInt());
			rootPosBottle.addList() = marker;
		}

#ifdef _DEBUG
		if(rootPosBottle.size() > 0)
		{
			Bottle *firstMarkerRoot = rootPosBottle.get(0).asList();
			Bottle *firstMarkerEye = eyePosBottle.get(0).asList();
			log << numIter << "\t" 
				<< firstMarkerRoot->get(0).asDouble() << "\t" 
				<< firstMarkerRoot->get(1).asDouble() << "\t"
				<< firstMarkerRoot->get(2).asDouble() << "\t"
				<< firstMarkerEye->get(0).asDouble() << "\t"
				<< firstMarkerEye->get(1).asDouble() << "\t"
				<< firstMarkerEye->get(2).asDouble() << "\t"
				<< endl;
			numIter ++;
		}
#endif

		if(rootPosBottle.size() >0)
		{
			rootPosPort.write();
			eyePosPort.write();
		}
	}
	
	viewPort.write();

    return true;
}


sig::Vector MultiMarkerTracker::GetJointAngles(string partName)
{
	//compatibility problems between the orders of the encoders and the standart torso pitch roll yaw;
	if(partName == "torso")
	{
		sig::Vector jointAngles(3);
		Bottle *torsoAngles = torsoEncodersPort.read();
		//invert torso
		jointAngles[0] = torsoAngles->get(2).asDouble() * M_PI/180;
		jointAngles[1] = torsoAngles->get(1).asDouble() * M_PI/180;
		jointAngles[2] = torsoAngles->get(0).asDouble() * M_PI/180;
#ifdef _DEBUG
		torsoEncoders << numIter << "\t";
		for(int i=0; i<3; ++i)
		{
			torsoEncoders << jointAngles[i] << "\t";
		}
		torsoEncoders << endl;
#endif
		return jointAngles;
	}
	else // head
	{
		sig::Vector jointAngles(6);
		Bottle *headAngles = headEncodersPort.read();
		
		for(int i=0; i<6; ++i)
		{
			jointAngles[i] = headAngles->get(i).asDouble() * M_PI/180;
		}

#ifdef _DEBUG
		headEncoders << numIter << "\t";
		for(int i=0; i<6; ++i)
		{
			headEncoders << jointAngles[i] << "\t";
		}
		headEncoders << endl;
#endif 
		return jointAngles;
	}
}




PolyDriver *MultiMarkerTracker::CreatePolyDriver(string partName)
{
	//sets the device
	Property options;
    options.put("device", "remote_controlboard");

	//sets the local port name.
	string localPortName = getName( partName.c_str());
	//string localPortName = (string)threeDPosPort.getName() + "/" + partName;
	options.put("local", localPortName.c_str());   //local port names

	//sets the remote port name
	string remotePortName = "/" + (string)parameters["robot"]->asString() + "/" + partName;
	options.put("remote", remotePortName.c_str());//where we connect to

    // creates the driver
    PolyDriver *driver = new PolyDriver(options);
    if (!driver->isValid()) 
	{
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return NULL;
    }

	IPositionControl *pos;
    IEncoders *encs;

	//gets the number of joints for this part.
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



sig::Matrix MultiMarkerTracker::GetRobotTransformationMatrix()
{
	string eyeName = parameters["eye"]->asString();
	//setups a new chain for the eye we want to use.
	iCubEye eye(eyeName);
    iKinChain *chainEye= eye.asChain();

	//releases the joints of the torso and the head. The links are included in the chain.
	for(int i=0; i<8; ++i)
	{
		if(!chainEye->releaseLink(i))
		{
			cout << "Problem with link " << i << endl;
		}
	}

	//gets the joint angles for head and torso
	sig::Vector head = GetJointAngles("head");
	sig::Vector torso = GetJointAngles("torso");

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
	
	//the eye decoupled roll angle is the Vs + Vg/2 (left eye) or Vs - Vg/2 (right eye)
	if(eyeName == "left")
	{
		allAngles[allAngles.length()-1]=head[4]+head[5]/2;
	}
	else
	{
		allAngles[allAngles.length()-1]=head[4]-head[5]/2;
	}

	//returns the transformation matrix for the chain.
	
    return chainEye->getH(allAngles);
}
