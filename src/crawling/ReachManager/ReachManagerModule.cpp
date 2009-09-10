#include "ReachManagerModule.h"

#include <yarp/math/Math.h>
using namespace yarp::math;
//#include "ctrlMath.h";

ReachManagerModule::ReachManagerModule()
{
	Matrix leftRotMat(3,3); leftRotMat=0.0;
	Matrix rightRotMat(3,3); rightRotMat= 0.0;
	leftRotMat(0,2) = 1;
	leftRotMat(1,1) = -1;
	leftRotMat(2,0) = 1;
	rightRotMat(0,2) = -1;
	rightRotMat(1,1) = 1;
	rightRotMat(2,0) = 1;
	lastPosition = Vector(3);

	leftOrientation = dcm2axis(leftRotMat);
	rightOrientation = dcm2axis(rightRotMat);

	cout << "left orientation " << leftOrientation.toString() << endl;
}

ReachManagerModule::~ReachManagerModule(void)
{
}

bool ReachManagerModule::open(Searchable& config)
{
	cout << "config : " << config.toString() << endl;
	parameters["input_port"] = GetValueFromConfig(config, "input_port");
	parameters["output_port"] = GetValueFromConfig(config, "output_port");
	parameters["robot"] = GetValueFromConfig(config, "robot");
	parameters["num_dof"] = GetValueFromConfig(config, "num_dof");
	parameters["reach_command_code"] = GetValueFromConfig(config, "reach_command_code");
	parameters["max_error"] = GetValueFromConfig(config, "max_error");
	parameters["solver_name"] = GetValueFromConfig(config, "solver_name");
	parameters["enabled_arm"] = GetValueFromConfig(config, "enabled_arm");
	parameters["pos_vel_cont"] = GetValueFromConfig(config, "pos_vel_cont");
	parameters["min_reach_dist"] = GetValueFromConfig(config, "min_reach_dist");
	cout << "min reach dist : " << parameters["min_reach_dist"].asDouble() << endl;
	parameters["reach_mode_dist"] = GetValueFromConfig(config, "reach_mode_dist");
	cout << "reach mode dist : " << parameters["reach_mode_dist"].asDouble() << endl;

	
	inPort.open(parameters["input_port"].asString().c_str());
	outPort.open(parameters["output_port"].asString().c_str());

	Network::connect(outPort.getName().c_str(), "/manager");

	OpenIKSolver("right");
	OpenIKSolver("left");

	if(parameters["pos_vel_cont"].asInt())
	{

		InitPositionControl("right");
		InitPositionControl("left");
	}
	return true;
}


bool ReachManagerModule::close()
{
	CloseIKSolver("left");
	CloseIKSolver("right");
	inPort.close();
    inPort.close();
	delete iKinPorts["left"];
	delete iKinPorts["right"];
	if(parameters["pos_vel_cont"].asInt())
	{
		ClosePositionControl("left");
		ClosePositionControl("right");
	}
	return true;
}


bool ReachManagerModule::updateModule(void)
{
    Vector xd(7);
	Bottle *visionBottle = inPort.read();
	double minDistanceSQR = 999999;
	bool patchToReach = false;

	for(int i=0; i<visionBottle->size(); ++i)
	{
		Bottle *patchBottle=visionBottle->get(i).asList();
		if(patchBottle->size() != 4 || patchBottle->get(0).asDouble() > 100)
		{
			cout << "--------ERROR GETTING THE 3D POSITION OF THE OBJECT ---------- " << endl;
			return true;
		}

		if(patchBottle->get(3).asString() != "green")
		{
			continue;
		}

		Vector position(3);
		position[0] = patchBottle->get(0).asDouble();
		position[1] = patchBottle->get(1).asDouble();
		position[2] = patchBottle->get(2).asDouble();

		cout<< "Solving position : " << position.toString() << endl;
		double distanceSQR = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];

		if(distanceSQR<minDistanceSQR)
		{
			if(position[0]>-parameters["min_reach_dist"].asDouble())
			{
				continue;
			}
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
	double difference = differenceVector[0]*differenceVector[0] 
						+ differenceVector[1]*differenceVector[1] 
						+ differenceVector[2]*differenceVector[2];

	if(difference < MIN_DIFFERENCE)
	{
		return true;
	}

	/*Property test;
	if(!test.fromConfigFile("test.ini"))
	{
		cout << "no bene !!!" << endl;
	}

	xd[0] = test.find("xdx").asDouble();
	xd[1] = test.find("xdy").asDouble();
	xd[2] = test.find("xdz").asDouble();*/

	string bestArm;
	xd[3] = leftOrientation[0];
	xd[4] = leftOrientation[1];
	xd[5] = leftOrientation[2];
	xd[6] = leftOrientation[3];

	Vector resultQ = Solve(xd, ((string)parameters["enabled_arm"].toString()), bestArm);
	
	lastPosition = xd;

	cout << "distance : " << sqrt(minDistanceSQR) << endl;
	cout << "min distance : " << parameters["reach_mode_dist"].asDouble() << endl;
	if(minDistanceSQR<pow(parameters["reach_mode_dist"].asDouble(),2))
	{
		double headPitchAngle = -atan((xd[2]-L)/xd[0]);
		double headYawAngle = atan((xd[1]+0.1)/xd[0]);
		cout << "Head angle pitch : " << headPitchAngle << endl;
		cout << "Head angle Yaw : " << headYawAngle << endl;
		Bottle &outBottle = outPort.prepare();
		outBottle.clear();
		outBottle.addInt(55);
		outBottle.addDouble(headPitchAngle);
		outBottle.addDouble(headYawAngle);
		outPort.write();
	}

	if(bestArm == "none")
	{
		return true;
	}

	//cout << "result : " << resultQ.toString();

	if(parameters["pos_vel_cont"].asInt())
	{
		RobotPositionControl(bestArm, resultQ);
	}
	else
	{
		//setting crawling to init position and reach.
		Bottle &outBottle = outPort.prepare();
		outBottle.clear();
		outBottle.addInt(parameters["reach_command_code"].asInt());
		Bottle reachCommand;
		reachCommand.addString((bestArm + "_arm").c_str());
		for(int i=0; i<parameters["num_dof"].asInt(); ++i)
		{
			reachCommand.addDouble(resultQ[i]);
		}
		outBottle.addList() = reachCommand;
		outPort.write();
	}

	return true;
}

double ReachManagerModule::getPeriod(void)
{
	return MODULE_PERIOD;
}

void ReachManagerModule::OpenIKSolver(string arm)
{
	cout << "=====================================" << endl;
	cout << "Opening IKin Catesian Solver for " << arm << " arm." << endl;
	cout << "=====================================" << endl;
	// declare the on-line arm solver called "solver"
	string solverName = ((string)parameters["solver_name"].asString()) + "/" + arm + "_arm";
 //   iKSolvers[arm] = new ArmCartesianSolver(solverName.c_str());
	//Property options;
 //   // it will operate on the simulator (which is supposed to be already running)
	//options.put("robot",parameters["robot"].asString().c_str());
 //   // it will work with the right arm
	//options.put("type",arm.c_str());
 //   // it will achieve just the positional pose
 //   options.put("pose","xyz");
 //   // switch off verbosity
 //   options.put("verbosity","off");

 //   // launch the solver and make it connect to the simulator
 //   if (!iKSolvers[arm]->open(options))
	//{
	//	cout << "Error opening the IKinCartesianSolver " << solverName << endl;
 //       return;
	//}
    
    // prepare ports
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


void ReachManagerModule::CloseIKSolver(string arm)
{
	/*iKSolvers[arm]->close();*/
	iKinPorts[arm]->in.close();
	iKinPorts[arm]->out.close();
	iKinPorts[arm]->rpc.close();
}

Value ReachManagerModule::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}

void ReachManagerModule::InitPositionControl(string partName)
{
	Property options;
    options.put("device", "remote_controlboard");
	options.put("local", ("/reach_manager/position_control/" + partName).c_str());   //local port names

	string remotePortName = "/" + (string)parameters["robot"].asString() + "/" + partName + "_arm";
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

void ReachManagerModule::RobotPositionControl(string partName, const Vector &jointAngles)
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

	//bool done=false;

 //   while(!done)
 //   {
 //       pos->checkMotionDone(&done);
 //       Time::delay(0.1);
 //   }

	/*Vector tmpvec = jointAngles;
	cout << "jointAngles: " <<tmpvec.toString() << endl;
    pos->positionMove(tmpvec.data());

	while(!done)
    {
        pos->checkMotionDone(&done);
        Time::delay(0.1);
    }*/
    
    return;
}


void ReachManagerModule::ClosePositionControl(string partName)
{
	polydrivers[partName]->close();
	delete polydrivers[partName];
}

Vector ReachManagerModule::Solve(const sig::Vector &xd, string partName, string &resultPart)
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

		//cout <<  "VECTOR : " << xd.toString() << endl;

		ArmCartesianSolver::addTargetOption(cmd,xd);

		iKinPorts["right"]->out.write(cmd);
		iKinPorts["left"]->out.write(cmd);
		iKinPorts["right"]->in.wait(replyRight);
		iKinPorts["left"]->in.wait(replyLeft);

		Bottle *xdBottleRight = CartesianSolver::getTargetOption(replyRight);
		Bottle *xBottleRight = CartesianSolver::getEndEffectorPoseOption(replyRight);
		Bottle *qBottleRight = CartesianSolver::getJointsOption(replyRight);

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

		//xd[3]=leftOrientation[0];
		//xd[4]=leftOrientation[1];
		//xd[5]=leftOrientation[2];
		//xd[6]=leftOrientation[3];

		cout << "SOLVING WITH LEFT ARM" << endl;
		
		Bottle *xdBottleLeft = CartesianSolver::getTargetOption(replyLeft);
		Bottle *xBottleLeft = CartesianSolver::getEndEffectorPoseOption(replyLeft);
		Bottle *qBottleLeft = CartesianSolver::getJointsOption(replyLeft);

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
		if(minNorm2>(pow(parameters["max_error"].asDouble(),2)))
		{
			resultPart = "none";
			return resultQ;
		}

		for(int i =0; i<7; ++i)
			resultQ[i] = resultQBottle->get(i).asDouble();
	}
	else
	{
		resultPart = partName;

		Bottle cmd, reply;
		cmd.clear();
		
		cout << "SOLVING WITH " << partName << " ARM" << endl;

		//cout <<  "VECTOR : " << xd.toString() << endl;

		ArmCartesianSolver::addTargetOption(cmd,xd);

		iKinPorts[partName]->out.write(cmd);
		iKinPorts[partName]->in.wait(reply);

		Bottle *xdBottle = CartesianSolver::getTargetOption(reply);
		Bottle *xBottle = CartesianSolver::getEndEffectorPoseOption(reply);
		Bottle *qBottle = CartesianSolver::getJointsOption(reply);

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

		if(deltaNorm2>(pow(parameters["max_error"].asDouble(),2)))
		{
			resultPart = "none";
			return resultQ;
		}
	
		for(int i =0; i<7; ++i)
			resultQ[i] = qBottle->get(i).asDouble();
	}

	//cout << "MAX ERROR " << parameters["max_error"].asDouble() << endl;

	return resultQ;
}