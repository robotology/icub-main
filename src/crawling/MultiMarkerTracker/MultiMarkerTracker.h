/** @file MultiMarkerTracker.h Header file the CrawlGeneratorModule class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Gay Sebastien, EPFL
* Inspired by the work of Alexandre Bernardino (artkpSingleTracker).
* RobotCub Consortium, European Commission FP6 Project IST-004370
* email: sebastien.gay@epfl.ch
* website: www.robotcub.org
*
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2
* or any later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
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


/**
* A simple class to write a log.
*/
class MyLogger : public ARToolKitPlus::Logger
{
    void artLog(const char* nStr)
    {
        printf(nStr);
    }
};

/**
* An Artoolkit based tracker class to track multiple markers with identical or different IDs.
* Gets a Yarp image as input.
* Detects and tracks multiple markers with multiple IDs in the image.
* Outputs their position in the frame of reference of the eye and in the root reference frame of the robot.
*/
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

	/**
    * Constructor of the MultiMarkerTracker class.
    * Does nothing.
    */
    MultiMarkerTracker();

	/**
    * Destructor of the MultiMarkerTracker class.
    * Does nothing.
    */
    ~MultiMarkerTracker();

	//
	/**
    * Opens the module, 
	* Recovers parameters form the configuration file.
    * Initializes the tracker, opens the ports.
    */
    virtual bool open(Searchable& config);

	/**
    * Closes the module
    * Closes the ports
    */
    virtual bool close();

	/**
	* Called in case of interruption of the module 
	* Closes the module
	*/
    virtual bool interruptModule();

	/**
	* Main module loop, 
	* Tracks markers in the image
	* Sends their 3D postion in the reference frame of the eye and in the root reference frame of the robot.
	* Closes the module
	*/
    virtual bool updateModule();

	/**
    * Returns the period of the module.
    */
	virtual double getPeriod();

private:
	/**
    * Gets the current kinematic configuration of the robot.
	* Reads the joint angles of the part via rpc for forward kinematics.
	* @param partName The name of the part of which to get the encoders (here used for head and torso).
	* @return A Vector containing the joint angles.
    */
	sig::Vector GetJointAngles(string partName);

	//creates the polydrivers used to get the joint angles of the robot.
	/**
    * Creates the polydriver to access the specified part.
	* @param partName The name of the part to communicate with. (here used for head and torso).
	* @return A pointer to the newly created polydriver.
    */
	PolyDriver *CreatePolyDriver(string partName);

	//gets the transformation matrix of the eye (camera) in the root reference frame of the robot.
	/**
    * gets the transformation matrix of the eye (camera) in the root reference frame of the robot.
	* @return A Matrix containing the transformation matrix.
    */
	sig::Matrix GetRobotTransformationMatrix();

};


#endif /*MULTIMARKERTRACKER__H*/
