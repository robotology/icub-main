/** @file drummingControl.h Header file the DrummingControl class.
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

#ifndef DRUMMING_CONTROL__H
#define DRUMMING_CONTROL__H


#include <string>
#include <vector>
#include <map>
#include <yarp/os/all.h>

static const int NB_PARTS = 5;
static const std::string PARTS[NB_PARTS] = {"left_arm", "right_arm", "left_leg", "right_leg", "head"};
static const int SCORE_SIZE = 16;
/**
 * The class to control the drumming
 * 
 */
class DrummingControl
{

private:

	//struct TPartition
	//{
	//	double frequency;
	//	std::map< std::string, double> phase;
	//	std::map< std::string, double> target;
	//}

	struct Part
	{
		int customBeat;
		bool isUsed;
		yarp::os::BufferedPort< yarp::os::Bottle > *partitionPort;
		yarp::os::BufferedPort< yarp::os::Bottle > *phasePort;
		double currentPhase;
		//TPartitions currentPartition;
	};

	typedef std::map< std::string, Part> TParts;
	TParts parts;

    yarp::os::BufferedPort< yarp::os::Bottle > interactivePort;
    double currentTempo;
    int scoreSize;
public:
/**
 * Constructor of the DrummingGui class
 * Initializes the Window
 */
    DrummingControl();
    ~DrummingControl(void);

    std::vector<std::string> DrummingControl::Init(void);
	void SendPartition(void);





};

#endif //DRUMMING_GUI__H
