/** @file ReachManager.h Header file the CrawlGeneratorModule class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Gay Sebastien, EPFL
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

#ifndef REACHMANAGER_H
#define REACHMANAGER_H


#include <yarp/os/all.h>
using namespace yarp::os;

#include <iCub/iKinSlv.h>
using namespace iKin;

#include "SuperPort.h"

#define MODULE_PERIOD 0.1
#define MIN_DIFFERENCE 0.0
#define L 0.36
#define MODULE_NAME "ReachManager"


/**
* The main class to handle reaching. This class receives 3D positions of object in the root reference 
* frame of the robot and chooses the nearest "good" one for reaching. 
* It then calls the iKinCartesianSolver for both arms to see which one is the fittest for the reaching task (minimum reaching error).
* The module outputs the name of the fittest arm for reaching and the joints angles achieving the desired position.
*/
class ReachManager :
    public Module
{
public:
	struct IKinPort
	{
		SuperPort in;
		Port out;
		Port rpc;
	};

public:
	/**
    * Constructor of the GeneratorThread class.
    * Does nothing.
    */
    ReachManager(void);

	/**
    * Destructor of the GeneratorThread class.
    * Does nothing.
    */
    virtual ~ReachManager(void);

	/**
    * Main reaching loop.
    * Gets the 3D positions from the input port.
	* Computes the nearest good object and computes the joint angles for each arm to achieve its position.
	* Outputs the arm achieving the nearest position and the corresponding joint angles.
    */
    virtual bool updateModule();

	/**
    * Returns the period of the module.
    */
	virtual double getPeriod();

	/**
    * Initialization of the module.
	* Gets the parameters from the config file
	* Opens the connection with the iKinCartesianSolver modules for left and right arm.
	* Opens the input and output ports
    */
	virtual bool open(Searchable& config);

	/**
    * Closes the module
    * Closes the input and output ports
	* Closes the connection with the iKinCartesianSolver modules.
    */
	virtual bool close();


private:
    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;
	map<string, IKinPort *> iKinPorts;
	map<string, iCubArmCartesianSolver *> iKSolvers;
	map<string, Value> parameters;
	Vector leftOrientation, rightOrientation;
	map<string, PolyDriver *> polydrivers;
	map<string, int> nbJoints;
	Vector lastPosition;

private:
	void OpenIKSolver(string arm);
	void CloseIKSolver(string arm);
	Value GetValueFromConfig(Searchable& config, string valueName);
	void RobotPositionControl(string partName, const Vector &jointAngles);
	void InitPositionControl(string partName);
	void ClosePositionControl(string partName);
	Vector Solve(const sig::Vector &xd, string partName, string &resultPart);


};

#endif //REACHMANAGER_H