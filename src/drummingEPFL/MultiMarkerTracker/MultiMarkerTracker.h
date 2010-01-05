// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Sebastien Gay, BIRG, EPFL.
 * Courtesy to Alex Bernardino, Vislab, IST/ISR, who's work this code is inspired of.
 *
 */
#ifndef MULTIMARKERTRACKER__H
#define MULTIMARKERTRACKER__H

 // std
#include <stdio.h>
#include <fstream>
#include <map>
using namespace std;

// OpenCV
#include  <cv.h>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Module.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/Polydriver.h>
#include <yarp/math/Math.h>

using namespace yarp;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace yarp::dev;

#define Z_OFFSET 0.03

// ARToolkitPlus
#include "ARToolKitPlus/TrackerSingleMarker.h"
using namespace ARToolKitPlus;

class MyLogger : public ARToolKitPlus::Logger
{
    void artLog(const char* nStr)
    {
        printf(nStr);
    }
};

class MultiMarkerTracker : public Module 
{

private:
#ifdef _DEBUG
	ofstream twoDFile;
	ofstream threeDFile;
	int numIter;
#endif

	//the tracker
	ARToolKitPlus::TrackerSingleMarker *tracker;

	//log of the tracker
	MyLogger logger;

	//single value parameters
	map<string, Value *> parameters;
	
	//image frame
    IplImage *frame;

	//ports opened by the module
	BufferedPort<Bottle> markersPort;
	BufferedPort<Bottle> visionPort;
	BufferedPort<ImageOf<PixelRgb>> imagePort;
	BufferedPort<ImageOf<PixelRgb>> viewPort;

	//drivers to get the kinematic configuration of the robot.
	map<string, PolyDriver *> drivers;

	//number of joints per part
	map<string, int> nbJoints;

	//translation vector between the real object position and  the marker position.
	//frame of reference : marker frame.
	sig::Vector objectOffset;

	//initializes the tracker according to the parameters in the configuration file
	bool initTracker(ResourceFinder &rf);


public:

    MultiMarkerTracker();
    ~MultiMarkerTracker();

	//opens the module, recovers parameters form the configuration file,
	//initializes the tracker, opens the ports.
    virtual bool open(Searchable& config);

	//closes the module, closes the ports
    virtual bool close();

	//called in case of interruption of the module
    virtual bool interruptModule();

	//main module loop, tracks markers in the image and 
	//send their 3D postion in the root reference frame of the robot
    virtual bool updateModule();

	//sets the period of the module.
	virtual double getPeriod();

	//gets the current kinematic configuration of the robot.
	//reads the joint angles of the torso and head via rpc.
	sig::Vector GetJointAngles(string partName);

	//creates the polydrivers used to get the joint angles of the robot.
	PolyDriver *CreatePolyDriver(string partName);

	//gets the transformation matrix of the eye (camera) in the root reference frame of the robot.
	sig::Matrix GetRobotTransformationMatrix();

};


#endif /*MULTIMARKERTRACKER__H*/
