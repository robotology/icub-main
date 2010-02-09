#include "ReachManager.h"

#include <yarp/math/Math.h>
using namespace yarp::math;

#include <iCub/ctrlMath.h>;
using namespace ctrl;

#include <iCub/iKinVocabs.h>

//#define TEST

ReachManager::ReachManager()
{
	//Matrix leftRotMat(3,3); leftRotMat=0.0;
	//Matrix rightRotMat(3,3); rightRotMat= 0.0;
	//leftRotMat(0,2) = 1;
	//leftRotMat(1,1) = -1;
	//leftRotMat(2,0) = 1;
	//rightRotMat(0,2) = -1;
	//rightRotMat(1,1) = 1;
	//rightRotMat(2,0) = 1;

	//leftOrientation = dcm2axis(leftRotMat);
	//rightOrientation = dcm2axis(rightRotMat);

	lastPosition = Vector(3);
	lastPosition.zero();

	numSolving = 0;


}

ReachManager::~ReachManager(void)
{
}

bool ReachManager::open(Searchable& config)
{
	cout << "config : " << config.toString() << endl;
	parameters["input_port"] = new Value(GetValueFromConfig(config, "input_port"));
	parameters["output_port"] = new Value(GetValueFromConfig(config, "output_port"));
	parameters["robot"] = new Value(GetValueFromConfig(config, "robot"));
	parameters["num_dof"] = new Value(GetValueFromConfig(config, "num_dof"));
	parameters["reach_command_code"] = new Value(GetValueFromConfig(config, "reach_command_code"));
	parameters["max_error"] = new Value(GetValueFromConfig(config, "max_error"));
	parameters["solver_name"] = new Value(GetValueFromConfig(config, "solver_name"));
	parameters["enabled_arm"] = new Value(GetValueFromConfig(config, "enabled_arm"));
	parameters["pos_vel_cont"] = new Value(GetValueFromConfig(config, "pos_vel_cont"));
	parameters["min_reach_dist"] = new Value(GetValueFromConfig(config, "min_reach_dist"));
	cout << "min reach dist : " << parameters["min_reach_dist"]->asDouble() << endl;
	parameters["reach_mode_dist"] = new Value(GetValueFromConfig(config, "reach_mode_dist"));
	cout << "reach mode dist : " << parameters["reach_mode_dist"]->asDouble() << endl;
	parameters["object_ID"] = new Value(GetValueFromConfig(config, "object_ID"));

	//cout << "cool !" << endl;
	inPort.open(parameters["input_port"]->asString().c_str());
	outPort.open(parameters["output_port"]->asString().c_str());

	//Network::connect(outPort.getName().c_str(), "/manager");

	outFile.open("reaching.dat");

	if(parameters["enabled_arm"]->asString() == "left")
	{
		OpenIKSolver("left");
	}
	else if(parameters["enabled_arm"]->asString() == "right")
	{
		OpenIKSolver("right");
	}
	else if(parameters["enabled_arm"]->asString() == "both")
	{
		OpenIKSolver("left");
		OpenIKSolver("right");
	}

	if(parameters["pos_vel_cont"]->asInt())
	{

		InitPositionControl("right");
		InitPositionControl("left");
	}
	return true;
}


bool ReachManager::close()
{
	if(parameters["enabled_arm"]->asString() == "left")
	{
		CloseIKSolver("left");
		delete iKinPorts["left"];
	}
	else if(parameters["enabled_arm"]->asString() == "right")
	{
		CloseIKSolver("right");
		delete iKinPorts["right"];
	}
	else if(parameters["enabled_arm"]->asString() == "both")
	{
		CloseIKSolver("left");
		CloseIKSolver("right");
		delete iKinPorts["left"];
		delete iKinPorts["right"];
	}
	inPort.close();
    outPort.close();

	if(parameters["pos_vel_cont"]->asInt())
	{
		ClosePositionControl("left");
		ClosePositionControl("right");
	}
	for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}
	return true;
}

bool ReachManager::interruptModule()
{
	if(parameters["enabled_arm"]->asString() == "left")
	{
		CloseIKSolver("left");
		delete iKinPorts["left"];
	}
	else if(parameters["enabled_arm"]->asString() == "right")
	{
		CloseIKSolver("right");
		delete iKinPorts["right"];
	}
	else if(parameters["enabled_arm"]->asString() == "both")
	{
		CloseIKSolver("left");
		CloseIKSolver("right");
		delete iKinPorts["left"];
		delete iKinPorts["right"];
	}
	inPort.close();
    outPort.close();
	if(parameters["pos_vel_cont"]->asInt())
	{
		ClosePositionControl("left");
		ClosePositionControl("right");
	}
	return true;
}


bool ReachManager::updateModule(void)
{
    Vector xd(3);
	Bottle *visionBottle = inPort.read();
	double minDistanceSQR = 999999;
	bool patchToReach = false;

	if(!visionBottle)
	{
		return true;
	}

	for(int i=0; i<visionBottle->size(); ++i)
	{
		Bottle *patchBottle=visionBottle->get(i).asList();
		if(patchBottle->size() != 4 || patchBottle->get(0).asDouble() > 100)
		{
			cout << "--------ERROR GETTING THE 3D POSITION OF THE OBJECT ---------- " << endl;
			return true;
		}

		if(patchBottle->get(3).asInt() != parameters["object_ID"]->asInt())
		{
			continue;
		}

		Vector position(3);
		position[0] = patchBottle->get(0).asDouble();
		position[1] = patchBottle->get(1).asDouble();
		position[2] = patchBottle->get(2).asDouble();

		cout<< "Solving position : " << position.toString() << endl;
		double distanceSQR = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];

		if(distanceSQR < minDistanceSQR)
		{
			//the object is too far to even try reaching it.
			if(position[2] > parameters["reach_mode_dist"]->asDouble())
			{
				continue;
			}
			//tue object is too near. it is dangerous to reach it.
			if(fabs(position[0]) < parameters["min_reach_dist"]->asDouble())
			{
				continue;
			}

			//everything looks fine.
			xd[0] = position[0];
			xd[1] = position[1];
			xd[2] = position[2];
			patchToReach = true;
			minDistanceSQR = distanceSQR;
		}
		
	}

	if(!patchToReach)
	{
		return true;
	}


	Vector differenceVector = xd - lastPosition;
	double difference = sqrt (differenceVector[0]*differenceVector[0] 
						+ differenceVector[1]*differenceVector[1] 
						+ differenceVector[2]*differenceVector[2]);


	//The position is in new position is in the noise range of the last one.
	//Give the little dude some time to reach for god sake !
	if(difference < MIN_DIFFERENCE)
	{
		return true;
	}




	string bestArm;
	//xd[3] = leftOrientation[0];
	//xd[4] = leftOrientation[1];
	//xd[5] = leftOrientation[2];
	//xd[6] = leftOrientation[3];

	Vector resultQ = Solve(xd, ((string)parameters["enabled_arm"]->asString()), bestArm);

	if(bestArm == "none")
	{
		return true;
	}

	lastPosition = xd;
	//cout << "result : " << resultQ.toString();

	if(numSolving == 0)
	{
		numSolving ++;
		return true;
	}

	outFile << xd[0] << "\t" << xd[1] << "\t" << xd[2] << "\t" << endl;

	//setting crawling to init position and reach.
	Bottle &outBottle = outPort.prepare();
	outBottle.clear();
	outBottle.addInt(parameters["reach_command_code"]->asInt());
	Bottle reachCommand;
	reachCommand.addString((bestArm + "_arm").c_str());
	for(int i=0; i<parameters["num_dof"]->asInt(); ++i)
	{
		outFile << resultQ[i] << "\t" ;
		reachCommand.addDouble(resultQ[i]);
	}
	outFile << endl;
	outBottle.addList() = reachCommand;
	cout << "=============REACHING===============" << endl; 
	cout << "Sending : " << outBottle.toString() << endl;
	outPort.write();

	close();
	return false;
}

double ReachManager::getPeriod(void)
{
	return MODULE_PERIOD;
}

void ReachManager::OpenIKSolver(string arm)
{
	cout << "=====================================" << endl;
	cout << "Opening IKin Catesian Solver for " << arm << " arm." << endl;
	cout << "=====================================" << endl;
	
	string solverName = ((string)parameters["solver_name"]->asString()) + "/" + arm + "_arm";

	iKinPorts[arm] = new IKinPort;
	iKinPorts[arm]->in.open(("/" + (string)MODULE_NAME + "/" + arm + "/in").c_str());
	iKinPorts[arm]->out.open(("/" + (string)MODULE_NAME + "/" + arm + "/out").c_str());
	iKinPorts[arm]->rpc.open(("/" + (string)MODULE_NAME + "/" + arm + "/rpc").c_str());

    Network::connect(("/" + solverName + "/out").c_str(),iKinPorts[arm]->in.getName().c_str());
    Network::connect(iKinPorts[arm]->out.getName().c_str(),("/" + solverName + "/in").c_str());
    Network::connect(iKinPorts[arm]->rpc.getName().c_str(),("/" + solverName + "/rpc").c_str());

	Bottle cmd, reply;
	cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_SET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_MODE);
    cmd.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
    cout<<"switching to track mode...";
    iKinPorts[arm]->rpc.write(cmd,reply);
    cout<<reply.toString()<<endl;  
}


void ReachManager::CloseIKSolver(string arm)
{

	iKinPorts[arm]->in.close();
	iKinPorts[arm]->out.close();
	iKinPorts[arm]->rpc.close();
}

Value ReachManager::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}

void ReachManager::InitPositionControl(string partName)
{
	Property options;
    options.put("device", "remote_controlboard");
	options.put("local", ("/reach_manager/position_control/" + partName).c_str());   //local port names

	string remotePortName = "/" + (string)parameters["robot"]->asString() + "/" + partName + "_arm";
	options.put("remote", remotePortName.c_str());         //where we connect to

    // create a device
    polydrivers[partName] = new PolyDriver(options);
    if (!polydrivers[partName]->isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return;
    }

    IPositionControl *pos;
    IEncoders *encs;

    if (!(polydrivers[partName]->view(pos) && polydrivers[partName]->view(encs))) {
        printf("Problems acquiring interfaces\n");
        return;
    }

    pos->getAxes(&nbJoints[partName]);
    Vector encoders;
    Vector tmp;
    encoders.resize(nbJoints[partName]);
    tmp.resize(nbJoints[partName]);
    

    for (int i = 0; i < nbJoints[partName]; i++) 
	{
         tmp[i] = 5.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (int i = 0; i < nbJoints[partName]; i++) 
	{
        tmp[i] = 3.0;
        pos->setRefSpeed(i, tmp[i]);
    }
}

void ReachManager::RobotPositionControl(string partName, const Vector &jointAngles)
{
	IPositionControl *pos;
    IEncoders *encs;

    if (!(polydrivers[partName]->view(pos) && polydrivers[partName]->view(encs))) {
        printf("Problems acquiring interfaces\n");
        return;
    }

    Vector command;
	command.resize(nbJoints[partName]);

    //first zero all joints
    command=0;
    //now set the shoulder to some value
    command[0]=jointAngles[0] * 180 / 3.14;
    command[1]=jointAngles[1] * 180 / 3.14;
    command[2]=jointAngles[2] * 180 / 3.14;
    command[3]=jointAngles[3] * 180 / 3.14;
	command[4]=jointAngles[4] * 180 / 3.14;
	command[5]=jointAngles[5] * 180 / 3.14;
	command[6]=jointAngles[6] * 180 / 3.14;

	cout << "Moving to : " << command.toString() << endl;

	pos->positionMove(command.data());
    
    return;
}


void ReachManager::ClosePositionControl(string partName)
{
	polydrivers[partName]->close();
	delete polydrivers[partName];
}

Vector ReachManager::Solve(const Vector &xd, string partName, string &resultPart)
{
	double minNorm2;
	Bottle *resultQBottle;
	Vector resultQ(7);


	if(partName == "both")
	{
		//ask to resolve for some xyz position
		Bottle cmd, replyLeft, replyRight;
		cmd.clear();
		
		cout << "SOLVING WITH RIGHT ARM" << endl;

		iCubArmCartesianSolver::addTargetOption(cmd,xd);

		iKinPorts["right"]->out.write(cmd);
		iKinPorts["left"]->out.write(cmd);
		iKinPorts["right"]->in.wait(replyRight);
		iKinPorts["left"]->in.wait(replyLeft);

		if(replyRight.isNull() || replyLeft.isNull())
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}

		Bottle *xdBottleRight = CartesianSolver::getTargetOption(replyRight);
		Bottle *xBottleRight = CartesianSolver::getEndEffectorPoseOption(replyRight);
		Bottle *qBottleRight = CartesianSolver::getJointsOption(replyRight);

		
		if (!xdBottleRight || !xBottleRight || !qBottleRight)
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}
		if (xdBottleRight->isNull() || xBottleRight->isNull() || qBottleRight->isNull())
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}

		Vector deltaRight(3);
		deltaRight[0] = xBottleRight->get(0).asDouble() - xdBottleRight->get(0).asDouble();
		deltaRight[1] = xBottleRight->get(1).asDouble() - xdBottleRight->get(1).asDouble();
		deltaRight[2] = xBottleRight->get(2).asDouble() - xdBottleRight->get(2).asDouble();

		double deltaRightNorm2 = deltaRight[0]*deltaRight[0] + deltaRight[1]*deltaRight[1] + deltaRight[2]*deltaRight[2];
	
		cout << "right error : " << sqrt(deltaRightNorm2) << endl;
		cout<<"xd      ="<<xdBottleRight->toString()<<endl;
		cout<<"x       ="<<xBottleRight->toString()<<endl;
		cout<<"q [rad] ="<<qBottleRight->toString()<<endl;
		cout<<endl;


		cout << "SOLVING WITH LEFT ARM" << endl;
		
		Bottle *xdBottleLeft = CartesianSolver::getTargetOption(replyLeft);
		Bottle *xBottleLeft = CartesianSolver::getEndEffectorPoseOption(replyLeft);
		Bottle *qBottleLeft = CartesianSolver::getJointsOption(replyLeft);

		if (!xdBottleLeft || !xBottleLeft || !qBottleLeft)
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}
		if (xdBottleLeft->isNull() || xBottleLeft->isNull() || qBottleLeft->isNull())
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}

		Vector deltaLeft(3);
		deltaLeft[0] = xBottleLeft->get(0).asDouble() - xdBottleLeft->get(0).asDouble();
		deltaLeft[1] = xBottleLeft->get(1).asDouble() - xdBottleLeft->get(1).asDouble();
		deltaLeft[2] = xBottleLeft->get(2).asDouble() - xdBottleLeft->get(2).asDouble();

		double deltaLeftNorm2 = deltaLeft[0]*deltaLeft[0] + deltaLeft[1]*deltaLeft[1] + deltaLeft[2]*deltaLeft[2];
		
		cout << "left error : " << sqrt(deltaLeftNorm2) << endl;
		cout<<"xd      ="<<xdBottleLeft->toString()<<endl;
		cout<<"x       ="<<xBottleLeft->toString()<<endl;
		cout<<"q [rad] ="<<qBottleLeft->toString()<<endl;
		cout<<endl;

		if(deltaLeftNorm2<deltaRightNorm2)
		{
			resultPart = "left";
			resultQBottle = qBottleLeft;
		}
		else
		{
			resultPart = "right";
			resultQBottle = qBottleRight;
		}

		minNorm2 = min(deltaLeftNorm2, deltaRightNorm2);
		if(minNorm2>(pow(parameters["max_error"]->asDouble(),2)))
		{
			cout << "TOO BIG ERROR !!! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}

		if(resultPart == "left" && deltaLeft[0] < 0)
		{
			cout << "REACHING TOO FAR !!! " << endl;
			resultPart = "none";
			return resultQ;
		}
		if(resultPart == "right" && deltaRight[0] < 0)
		{
			cout << "REACHING TOO FAR !!! " << endl;
			resultPart = "none";
			return resultQ;
		}

		for(int i =0; i<7; ++i)
			resultQ[i] = resultQBottle->get(i).asDouble() * M_PI/180;
	}
	else
	{
		resultPart = partName;

		Bottle cmd, reply;
		cmd.clear();
		
		cout << "SOLVING WITH " << partName << " ARM" << endl;

		iCubArmCartesianSolver::addTargetOption(cmd,xd);

		iKinPorts[partName]->out.write(cmd);
		iKinPorts[partName]->in.wait(reply);

		if(reply.isNull())
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}


		Bottle *xdBottle = CartesianSolver::getTargetOption(reply);
		Bottle *xBottle = CartesianSolver::getEndEffectorPoseOption(reply);
		Bottle *qBottle = CartesianSolver::getJointsOption(reply);

		if (!xdBottle || !xBottle || !qBottle)
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}
		if (xdBottle->isNull() || xBottle->isNull() || qBottle->isNull())
		{
			cout << "\n\nERROR ! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}

		Vector delta(3);
		delta[0] = xBottle->get(0).asDouble() - xdBottle->get(0).asDouble();
		delta[1] = xBottle->get(1).asDouble() - xdBottle->get(1).asDouble();
		delta[2] = xBottle->get(2).asDouble() - xdBottle->get(2).asDouble();

		
		double deltaNorm2 = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];
		cout << "error : " << sqrt(deltaNorm2) << endl;
		cout << "xd      =" << xdBottle->toString()<<endl;
		cout << "x       =" << xBottle->toString()<<endl;
		cout << "q [rad] =" << qBottle->toString()<<endl;
		cout << endl;

		if(delta[0] < 0)
		{
			cout << "REACHING TOO FAR !!! " << endl;
			resultPart = "none";
			return resultQ;
		}

		if(deltaNorm2>(pow(parameters["max_error"]->asDouble(),2)))
		{
			cout << "TOO BIG ERROR !!! \n\n" << endl;
			resultPart = "none";
			return resultQ;
		}
	
		for(int i =0; i<7; ++i)
			resultQ[i] = qBottle->get(i).asDouble() * M_PI/180;
	}

	//cout << "MAX ERROR " << parameters["max_error"].asDouble() << endl;

	return resultQ;
}