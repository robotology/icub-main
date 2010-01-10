/** @file CrawPlanner.h Header file the CrawPlanner class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Sebastien Gay, EPFL
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

#ifndef CRAWLPLANNER__H
#define CRAWLPLANNER__H

#include <yarp/os/all.h>
using namespace yarp::os;
#include "Potential.h"
#include <iostream> 
#include <string>
#include <vector>
#include <map>
using namespace std;

#define SUPERVISOR_OUT_PORT_NAME "/CrawlPlanner/supervisor/out"


#define MODULE_PERIOD 0.2

#define MAX_ROTATION_ANGLE 30 /**< Maximum rotation angle of the robot (roll angle of the torso) */

#define POTENTIAL_POSITION_PRECISION 0.1 /**< Precision on the position of a potential */

#define EPSILON_ANGLE 5 /**< Minimum rotation angle (angles less than this value are considered null */

#define GOAL_POTENTIAL -1 /**< Potential value for a goal */
#define OBSTACLE_POTENTIAL 1 /**< Potential value for an obstacle */

/**
* The main crawling planner module class. 
* This class gets the 3D position of the obstacles and goals in the root reference frame of the robot.
* It generates a potential field (@see Potential) and computes the new orientation fo the robot
* It outputs the new crawling orientation (torso roll angle) of the robot as well as commands for the CrawlManagerModule (@see CrawlManager).
*/
class CrawlPlanner :
    public Module
{
private:
    std::vector<Potential> potentialField;

    map<string, BufferedPort<Bottle>* > ports;
    
    double previousRotationAngle;  
    double previousNeckAngle;  
    double previousTetaDot;  

	map<string, Value *> parameters;

public:
	/**
    * Constructor of the GeneratorThread class.
    * Does nothing
    */
    CrawlPlanner(void);

	/**
    * Destructor of the GeneratorThread class.
    * Does nothing.
    */
    ~CrawlPlanner(void);

	/**
    * Opens the module.
	* Read parameters from the config files.
    * Opens the ports.
    */
	virtual bool open(Searchable &config);

	/**
    * Closes the module.
    * Closes the ports.
    */
	virtual bool close();

	/**
    * Returns the period of the module.
    */
    virtual double getPeriod(void);

	/**
    * Main lood of the planner module.
	* Gets the 3D position of the obstacles and goals in the root reference frame of the robot.
	* Generates a potential field (@see Potential) and computes the new orientation fo the robot
	* Outputs the new crawling orientation (torso roll angle) of the robot as well as commands for the CrawlManagerModule (@see CrawlManager)
    */
    virtual bool updateModule(void);

	/**
    * Gets a value from the config file.
    */
	Value GetValueFromConfig(Searchable& config, string valueName);

private:
    double ComputeRobotRotation(void) const;
	Vector ComputePotentialGradient(void) const;
	int ExistPotential(double x, double y, int objectID);
    void BuildPotentialField();
    bool IsInVisibleRange(const Potential &potential, double visionOrientationAngle) const;
    void SendToSupervisor(void);
    bool ScanFinished(double neckAngle);



    inline void WritePotentialField()
    {
        for(unsigned int i=0; i<potentialField.size(); ++i)
        {
            cout << "potential : (" << potentialField[i].GetPotentialVector()[0] << "," << potentialField[i].GetPotentialVector()[1] << ")" << endl;
        }
    }

};

#endif //CRAWLPLANNER__H