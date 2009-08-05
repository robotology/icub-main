// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* iCub grasp library VVV09, Copyright (C) 2009 RobotCub Consortium
 * authors: Kail Frank, Theo Jacobs, Julian Schill, Yan Wu
 * http://www.robotcub.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  US
*/

#include "grasp_vvv09.h"

bool grasp_vvv09::Init(int arc, char *arv[])
{
	string pRobotName;
	double pPrepose, pEndpose;
	int pSide;
	
	params.fromCommand(arc, arv);
    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return false;
    }
	else if(!params.check("arm"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--arm part (e.g. left_arm)\n");
        return false;
    }

    if (params.check("prepose"))
	{
		Prepose_param = params.find("prepose").asDouble();
		printf("Prepose = %f \n",Prepose_param);
		if (pPrepose < 0)
			pPrepose = - pPrepose;
		if (pPrepose > 1)
			pPrepose = 1;
	}
	else
	{
		fprintf(stderr, "The pregrasp pose is not specified\n");
        fprintf(stderr, "--prepose value (from 0 to 1, defaulted to 1 now)\n");
		pPrepose = 1;
    }

    if (params.check("endpose"))
	{
		Endpose_param = params.find("endpose").asDouble();
		printf("Endpose = %f \n",Endpose_param);
		if (pEndpose < 0)
			pEndpose = - pEndpose;
		if (pEndpose > 1)
			pEndpose = 1;
	}
	else
	{
		fprintf(stderr, "The endgrasp pose is not specified\n");
        fprintf(stderr, "--endpose value (from 0 to 1, defaulted to 1 now)\n");
		Endpose_param = 1;
    }

	pRobotName=params.find("robot").asString().c_str();
        
	
	Init(pRobotName, pPrepose, pEndpose, pSide); 
	
}
	
bool grasp_vvv09::Init( string robot_name, double prepose, double endpose, int side)
{
	
    Prepose_param=prepose;
    Endpose_param=endpose;

    std::string remotePorts="/";
    remotePorts+=robot_name;
	remotePorts+="/";
	std::string localPorts="/grasp/client/";
	
	if (side == 0)
		{
    		remotePorts+="left_arm";
    		localPorts+="left_arm";
		}
    else
    	{
    		remotePorts+="right_arm";	
    		localPorts+="right_arm";
    	}
    
	
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());
    options.put("remote", remotePorts.c_str());

    // create a device
    robotDevice.open(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return false;
    }

	bool ok;
    ok = robotDevice.view(pos);
	ok = ok && robotDevice.view(encs);
	ok = ok && robotDevice.view(amp);
	ok = ok && robotDevice.view(vels);
	ok = ok && robotDevice.view(pid);
	if (!ok)
	{
		printf("Problems acquiring interfaces\n");
		return 0;
    }
	pos->getAxes(&nj);
	positions.resize(nj);
	amplifier.resize(nj);
	velocities.resize(nj);
	pid_val.resize(nj);
	current_thresholds.resize(nj);
	pid_thresholds.resize(nj);
	refAccelerations_velMode.resize(nj);

	PrePose[0].resize(nj);
	PrePose[1].resize(nj);
	
	Vector tmp;
	tmp.resize(nj);
	tmp=50.0;
    vels->setRefAccelerations(tmp.data());

    currentThresholdExceeded = new bool [nj];
    pidThresholdExceeded = new bool [nj];
    collisionDetected = new bool [nj];
    currentcounter = new int [nj] ;
	pidcounter = new int [nj];
	grasp_vvv09::startingJoint = 7;
	grasp_vvv09::position_reached = new bool [nj];

	for (int i=0;i<nj;i++)
	{
		currentThresholdExceeded[i]=false;
		pidThresholdExceeded[i]=false;
		collisionDetected[i]=false;
		position_reached[i]=false;
		currentcounter[i]=0;
		pidcounter[i]=0;
	}

	current_thresholds = 30; // setting some default value
	
	current_thresholds[7] = 1000;
	current_thresholds[8] = 1000;
	current_thresholds[9] = 15;
	current_thresholds[10] = 40;
	current_thresholds[11] = 20;
	current_thresholds[12] = 30;
	current_thresholds[13] = 20;
	current_thresholds[14] = 30;
	current_thresholds[15] = 30;
	
	pid_thresholds = 10;		// setting some default value
	
	pid_thresholds[7] = 10;
	pid_thresholds[8] = 10;
	pid_thresholds[9] = 7;
	pid_thresholds[10] = 8;
    pid_thresholds[11] = 10;
	pid_thresholds[12] = 10;
	pid_thresholds[13] = 10;
	pid_thresholds[14] = 8;
	pid_thresholds[15] = 70;
	
	refAccelerations_velMode = 5;
	refAccelerations_velMode[nj-1] = 50;
	
	fistPose.resize(nj);
	fistPose=0;
	fistPose[7] = 0;
    fistPose[8] = 90;
    fistPose[9] = 40;
    fistPose[10] = 80;
    fistPose[11] = 60;
    fistPose[12] = 110;
    fistPose[13] = 40;
    fistPose[14] = 110;
    fistPose[15] = 210;

	pinchPose.resize(nj);
    pinchPose=0;
    pinchPose[7] = 5;
    pinchPose[8] = 70;
    pinchPose[9] = 45;
    pinchPose[10] = 0;
    pinchPose[11] = 70;
    pinchPose[12] = 0;
    pinchPose[13] = 70;
    pinchPose[14] = 0;
    pinchPose[15] = 180;

	max_jointSpeed = 100;
	velocityCorrectionFactor.resize(nj);
	velocityCorrectionFactor=1.0;
	velocityCorrectionFactor[10]=1.0;
	velocityCorrectionFactor[15]=1.0;

	targetPosition.resize(nj);
	targetPosition = 0;

	return true;
}

bool grasp_vvv09::move_to_pregrasp_pos()
{
	encs->getEncoders(positions.data());

	Vector command;
    command.resize(nj);
	command=50.0;
    pos->setRefAccelerations(command.data());

	command=30.0;
	command[15]=100;
    pos->setRefSpeeds(command.data());

	command=positions;
	PrePose[0]=positions;
	PrePose[1]=positions;
	for (int i=7; i<nj; i++)
	{
		PrePose[0][i]=0;
		PrePose[1][i]=0;
	}
//  For the old robot
	//PrePose[0][7] = 30;
	//PrePose[1][8] = 90;

// For the new robot
	PrePose[0][8] = 10;

	PrePose[1][7] = 0;
	PrePose[1][8] = 90;

	for (int i=7; i<nj; i++)
		command[i]=Prepose_param*PrePose[0][i]+(1-Prepose_param)*PrePose[1][i];

	position_move(command);

	return 0;
}
bool grasp_vvv09::position_move(Vector POS)
{
	pos->setPositionMode();
	for (int i=startingJoint; i<nj; i++)
		pos->positionMove(i,POS[i]);
    bool done=false;
    while(!done)
	{
		bool jointdone;
		done = true;
		for (int i=startingJoint; i<nj; i++)
		{
			pos->checkMotionDone(i, &jointdone);
			done &= jointdone;
			if (jointdone) printf ("Joint %i done.\n",i);
		}
		Time::delay(0.1);
	}
	printf("Position move done\n");
	return 0;
}

bool grasp_vvv09::doGrasp() {
	// exclude 0-7 and 15 |   0     1     2     3     4     5     6     7     8      9      10     11     12     13     14     15
	bool excludedJoints[] = {true, true, true, true, true, true, true, true, false, false, false, false, false, false, false, false};

	doGrasp(excludedJoints);
}

bool grasp_vvv09::doGrasp(bool* excludedJoints) {
// initiate and run the grasping process

	for (int i=0;i<nj;i++)
	{
		currentThresholdExceeded[i]=false;
		pidThresholdExceeded[i]=false;
		collisionDetected[i]=false;
		position_reached[i]=false;
		currentcounter[i]=0;
		pidcounter[i]=0;
	}

	// set reference accelerations
	vels->setRefAccelerations(refAccelerations_velMode.data());
	
	// calculate target position and normalized velocities
	Vector targetVelocity;
	double maxDist = 0;
	targetVelocity.resize(nj);
	targetVelocity = 0;
	get_encs();
	for (int i=startingJoint; i<nj; i++) {
		targetPosition[i] = Endpose_param*fistPose[i] + (1-Endpose_param)*pinchPose[i];
		targetVelocity[i] = targetPosition[i] - positions[i];
		if (fabs(targetVelocity[i]) > maxDist)
			maxDist = fabs(targetVelocity[i]);
	}
	for (int i=startingJoint; i<nj; i++) {
		targetVelocity[i] = max_jointSpeed*targetVelocity[i]*velocityCorrectionFactor[i]/maxDist;
	}

	velocities=targetVelocity;
	// set velocities
	velocity_move(targetVelocity);

	// loop to check for target reached or collision or timeout
	bool allreached = false;
	double startTime = Time::now();
	while (!allreached) {
		Vector velocityCommand;
		velocityCommand.resize(nj);
		velocityCommand = targetVelocity;
		allreached = true;
		checkCollisions();
		check_encoders(targetPosition);
		for (int i=startingJoint; i<nj; i++) {
		// loop leaves out joints 7 and 15 because collision check here doesn't work!
            if (collisionDetected[i] || position_reached[i])
                {
                	velocityCommand[i] = 0.0;
                	printf("stopping joint %i\n", i);
				}
            else
            {
            	if (!excludedJoints[i])
                	allreached = false;
                printf("joint %i moves on\n", i);
			}
        }   // for
        if (collisionDetected[7] || position_reached[7])
                velocityCommand[7] = 0.0;
        if (collisionDetected[15] || position_reached[15])
                velocityCommand[15] = 0.0;
		velocity_move(velocityCommand);
		
		Time::delay(0.1);
		if (Time::now()-startTime > 6.0) {
			velocityCommand = 0;
			velocity_move(velocityCommand);
			printf("### got timeout for grasping ###");
			break;
		}
	}	// while

	// test if grasping was sucessful
	printf("needed %f seconds to grasp",Time::now()-startTime);
	return contactAtEndOfGripping(excludedJoints);
}


Vector grasp_vvv09::get_encs()
{
	encs->getEncoders(positions.data());
	return positions;
}
Vector grasp_vvv09::get_velocity()
{
	return velocities;
}
Vector grasp_vvv09::get_amps()
{
	amp->getCurrents(amplifier.data());
	return amplifier;
}
Vector grasp_vvv09::get_pid()
{
	pid->getErrors(pid_val.data());
	return pid_val;
}

bool* grasp_vvv09::get_collisionDetected()
{
	return collisionDetected;
}

bool grasp_vvv09::velocity_move(Vector velocity)
{
	vels->setVelocityMode();
	for (int i=startingJoint; i<nj; i++)
		vels->velocityMove(i,velocity[i]);
	velocities=velocity;
	return true;
}
double grasp_vvv09::get_prepose_param()
{
	return Prepose_param;
}
double grasp_vvv09::get_endpose_param()
{
	return Endpose_param;
}
Vector *grasp_vvv09::get_PrePose()
{
	return PrePose;
}
int grasp_vvv09::get_joints()
{
	return nj;
}

int grasp_vvv09::get_startingJoint()
{
	return startingJoint;
}

int grasp_vvv09::checkCurrents()
//This function checks the currents for a threshold and sets currentThresholdExceeded.
{
	get_amps();
	
	for( int i=7; i<nj; i++)
	{
		if ((amplifier[i] > current_thresholds[i]) && !(currentThresholdExceeded[i]))
		{
			if (currentcounter[i]>=30)
			{
				currentThresholdExceeded[i]=true;
				printf("current contact detected on joint %i with current %f\n", i, amplifier[i]);
			}
			else
			{
				currentcounter[i]++;
			}
		}
		else if ((amplifier[i] < current_thresholds[i]) && (currentThresholdExceeded[i]))
		{
			if (currentcounter[i]<=0)
			{
				currentThresholdExceeded[i]=false;
				printf("current contact lost on joint %i with current %f\n", i, amplifier[i]);
			}
			else
			{
				currentcounter[i]--;
			}
		}
	}
	return 1;
}

int grasp_vvv09::checkPidErrors()
//This function checks the PID errors for a threshold and sets pidThresholdExceeded.
{
	get_pid();
	
	for( int i=7; i<nj; i++)
	{
		if ((fabs(pid_val[i]) > pid_thresholds[i]) && !(pidThresholdExceeded[i]))
		{
			if (pidcounter[i]>=10)
			{
				pidThresholdExceeded[i]=true;
				printf("pid error contact detected on joint %i with current %f\n", i, pid_val[i]);
			}
			else
			{
				pidcounter[i]++;
			}
		}
		else if ((fabs(pid_val[i]) < pid_thresholds[i]) && (pidThresholdExceeded[i]))
		{
			if (currentcounter[i]<=0)
			{
				pidThresholdExceeded[i]=false;
				printf("pid error contact lost on joint %i with current %f\n", i, pid_val[i]);
			}
			else
			{
				pidcounter[i]--;
			}
		}
	}
	return 1;
}

int grasp_vvv09::checkCollisions()
{
	
	checkCurrents();
    checkPidErrors();
	for(int i=startingJoint; i<nj; i++)
		collisionDetected[i] = currentThresholdExceeded[i] || pidThresholdExceeded[i];
	return 1;	
}

int grasp_vvv09::check_encoders(Vector target_positions) {
	// this function checks if the garget position has been reached
	// update values
	get_velocity();
	get_encs();
	// calculate error
    Vector position_error = vectorSubtr(target_positions,positions);
    double position_threshold = 1;

	// check calculated error
    for (int i=get_startingJoint(); i<get_joints(); i++) {
        if (fabs(position_error[i]) <= position_threshold) {
			printf("joint %d reached target position\n",i);
            position_reached[i] = true;
        }
        else if (position_error[i]*velocities[i] < 0) {
            // already past target position
            position_reached[i] = true;
			printf("joint %d is bigger than target position\n",i);
        }
        else
            position_reached[i] = false;
    }	// for

    return 0;
}

bool grasp_vvv09::contactAtEndOfGripping(bool* excludedJoints) {
	// get the newest data
	checkCollisions();
	check_encoders(targetPosition);

	// check if all joints have finished movement somehow, leaving out joint 7
	/*for (int i=get_startingJoint()+1; i<get_joints(); i++) {
		if (!(collisionDetected[i] || position_reached[i] || excludedJoints[i])) {
			printf("end of gripping not yet reached for all joints!");
			return false;
		}
	}*/
	// AND check if at least one of the thumb joints is in contact
	if (!(collisionDetected[9] || collisionDetected[10])){
		printf("thumb is not in contact");
		return false;
	}
	// AND check if any other joint is in contact
	if (!(collisionDetected[11] || collisionDetected[12] || collisionDetected[13] || collisionDetected[14] || collisionDetected[15])) {
		printf("no other finger is in contact");
		return true;
	}
}

Vector grasp_vvv09::vectorAdd(Vector A, Vector B) {
	int size = A.size();
	if (B.size() != size) 
	{
		printf("Vector sizes don't match. Aborting addition.\n");
		return NULL;
	}
	Vector result;
	result.resize(size);

	for (int i=0; i<size; i++)
		result[i] = A[i]+B[i];

	return result;
}

Vector grasp_vvv09::vectorSubtr(Vector A, Vector B) {
    int size = A.size();
    if (B.size() != size) 
	{
		printf("Vector sizes don't match. Aborting subtraction.\n");
		return NULL;
	}
    Vector result;
    result.resize(size);

    for (int i=0; i<size; i++)
        result[i] = A[i]-B[i];

    return result;
}

bool grasp_vvv09::setCurrentThreshold(Vector currentThresholds)
{
	current_thresholds=currentThresholds;
	return 0;	
}

bool grasp_vvv09::setPidThreshold(Vector PidThresholds)
{
	pid_thresholds=PidThresholds;
	return 0;
}

bool grasp_vvv09::setPreposeParam(double pprepose)
{
	Prepose_param=pprepose;
	return 0;
}

bool grasp_vvv09::setEndposeParam(double pendpose)
{
	Endpose_param=pendpose;
	return 0;
}
