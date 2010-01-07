#include "DrumIKin.h"

#include <yarp/math/Math.h>
using namespace yarp::math;
#include <iCub/ctrlMath.h>
using namespace ctrl;

#include <iCub/iKinVocabs.h>


#define TEST

DrumIKin::DrumIKin()
{
}

DrumIKin::~DrumIKin(void)
{
}

bool DrumIKin::open(Searchable& config)
{
	cout << "config : " << config.toString() << endl;
	parameters["input_port"] = new Value(GetValueFromConfig(config, "input_port"));
	parameters["output_port"] = new Value(GetValueFromConfig(config, "output_port"));
	parameters["robot"] = new Value(GetValueFromConfig(config, "robot"));
	parameters["num_dof"] = new Value(GetValueFromConfig(config, "num_dof"));
	parameters["max_error"] = new Value(GetValueFromConfig(config, "max_error"));
	parameters["solver_name"] = new Value(GetValueFromConfig(config, "solver_name"));
	parameters["pos_vel_cont"] = new Value(GetValueFromConfig(config, "pos_vel_cont"));
	parameters["markers_id_mappings"] = new Value(GetValueFromConfig(config, "markers_id_mappings"));
	parameters["mode"] = new Value(GetValueFromConfig(config, "mode"));
	Bottle mappings = config.findGroup("markers_id_mappings");

	armMarkerMapping[mappings.get(1).asInt()] = "left_arm";
	armMarkerMapping[mappings.get(2).asInt()] = "left_arm";
	armMarkerMapping[mappings.get(3).asInt()] = "right_arm";
	armMarkerMapping[mappings.get(4).asInt()] = "right_arm";

	inPort.open(parameters["input_port"]->asString().c_str());
	outPort.open(parameters["output_port"]->asString().c_str());

	OpenIKSolver("right_arm");
	OpenIKSolver("left_arm");

	if(parameters["pos_vel_cont"]->asInt())
	{

		InitPositionControl("right_arm");
		InitPositionControl("left_arm");
	}
	return true;
}


bool DrumIKin::close()
{
	CloseIKSolver("left_arm");
	CloseIKSolver("right_arm");
	inPort.close();
    outPort.close();
	delete iKinPorts["left_arm"];
	delete iKinPorts["right_arm"];
	if(parameters["pos_vel_cont"]->asInt())
	{
		ClosePositionControl("left_arm");
		ClosePositionControl("right_arm");
	}
	
	for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}
	return true;
}


bool DrumIKin::updateModule(void)
{
	Bottle *visionBottle = inPort.read();

	for(int i=0; i<visionBottle->size(); ++i)
	{
		Vector xp(3);
		int markerID;

#ifdef TEST
		xp[0] = visionBottle->get(0).asDouble();
		xp[1] = visionBottle->get(1).asDouble();
		xp[2] = visionBottle->get(2).asDouble();
		markerID = 3;
#else
		Bottle *patchBottle=visionBottle->get(i).asList();
		if(patchBottle->size() != 4 || patchBottle->get(0).asDouble() > 100)
		{
			cout << "--------ERROR GETTING THE 3D POSITION OF THE OBJECT ---------- " << endl;
			return true;
		}

		//position of a patch;
		xp[0] = patchBottle->get(0).asDouble();
		xp[1] = patchBottle->get(1).asDouble();
		xp[2] = patchBottle->get(2).asDouble();

		markerID = patchBottle->get(3).asInt();
#endif
		//get the target vector;
		Vector xd = GetTargetVector(xp, armMarkerMapping[markerID]);

		cout<< "Solving position : " << xd.toString() << endl;

	/*Property test;
	if(!test.fromConfigFile("test.ini"))
	{
		cout << "no bene !!!" << endl;
	}

	xd[0] = test.find("xdx").asDouble();
	xd[1] = test.find("xdy").asDouble();
	xd[2] = test.find("xdz").asDouble();*/

		double precision;
		Vector resultQ = Solve(xd, armMarkerMapping[markerID], precision);

		if(precision > parameters["max_error"]->asDouble())
		{
			cout<<"WARNING : precision is less than the maximum error set to "
				<< parameters["max_error"]->asDouble() << endl;
			continue;
		}

		if(parameters["pos_vel_cont"]->asInt())
		{
			RobotPositionControl(armMarkerMapping[markerID], resultQ);
		}
		else
		{
			//setting crawling to init position and reach.
			Bottle &outBottle = outPort.prepare();
			outBottle.clear();
			//outBottle.addInt(parameters["reach_command_code"].asInt());
			Bottle jointsAnglesBottle;
			jointsAnglesBottle.addInt(markerID);
		/*	for(int i=0; i<parameters["num_dof"]->asInt(); ++i)
			{
				jointsAnglesBottle.addDouble(resultQ[i]);
			}*/
			jointsAnglesBottle.addDouble(resultQ[0]);
			jointsAnglesBottle.addDouble(resultQ[1] * 0.91);
			jointsAnglesBottle.addDouble(resultQ[2] * 0.86);
			jointsAnglesBottle.addDouble(resultQ[3]);


			outBottle.addList() = jointsAnglesBottle;
			cout << " ============== sending : " << outBottle.toString() << " ===================" << endl;
			outPort.write();
		}
		Time::delay(0.5);
	}

	return true;
}

Vector DrumIKin::GetTargetVector(const Vector &xp, string arm)
{
	if(parameters["mode"]->asString() == "xyz")
	{
		//no orientation;
		return xp;
	}

	//======[computes the desired orientation with respect to the patch position]=====

	//defines the arm position
	double Lz; 
	double Ly;
	double sinThetax;
	Lz = ARM_Z_OFFSET/1000;

	//matrix to have the hand point forward 
	//(i.e. for right hand : zero rotation,
	//for left hand : rotation of PI around z.
	Matrix RAlign(3,3); 
	RAlign.zero();
	if(arm == "left_arm")
	{
		RAlign(0,0) = -1;
		RAlign(1,1) = -1;
		RAlign(2,2) = 1;
		sinThetax = -1; //sin(-PI/2)
		Ly = -ARM_Y_OFFSET/1000; 
	}
	else
	{
		sinThetax = 1; //sin(PI/2)
		Ly = ARM_Y_OFFSET/1000;
	}

	//rotation matrix along the x axis (of an angle +-Pi/2)
	//cos(+-PI/2 = 0)
	Matrix Rx(3,3);
	Rx.zero();
	Rx(0,0) = 1;
	Rx(1,2) = -sinThetax;
	Rx(2,1) = sinThetax;

	//rotation matrix along the y axis (i.e. in the xz plan)
	Matrix Ry(3,3);
	//radius in the xz plan
	double Radxz = sqrt( pow(xp[0], 2) + pow(xp[2] - Lz, 2) );
	//rotation angle along the y axis
	double cosThetay = xp[0] / Radxz;
	double sinThetay = (xp[2] - Lz) / Radxz;
	Ry.zero();
	Ry(0,0) = cosThetay;
	Ry(0,2) = sinThetay;
	Ry(1,1) = 1;
	Ry(2,0) = -sinThetay;
	Ry(2,2) = cosThetay;

	//rotation matrix along the z axis (i.e. in the xy plan)
	Matrix Rz(3,3);
	//radius in the xy axis
	double Radxy = sqrt( pow(xp[0], 2) + pow(xp[1] - Ly, 2) );
	//rotation angle along the z axis
	double cosThetaz = xp[0] / Radxy;
	double sinThetaz = -(Ly + xp[1]) / Radxy;
	Rz.zero();
	Rz(0,0) = cosThetaz;
	Rz(0,1) = -sinThetaz;
	Rz(1,0) = sinThetaz;
	Rz(1,1) = cosThetaz;
	Rz(2,2) = 1;

	//total rotation
	Matrix rotationMatix = RAlign * Rx * Ry * Rz;

	//switch to axixangle notation
	Vector orientation = dcm2axis(rotationMatix);

	Vector xd(7);
	for(int i=0; i<xp.size(); ++i)
	{
		xd[i] = xp[i];
	}
	for(int i=0; i<orientation.size(); ++i)
	{
		xd[i+xp.size()] = orientation[i];
	}
	return xd;
}

double DrumIKin::getPeriod(void)
{
	return MODULE_PERIOD;
}

void DrumIKin::OpenIKSolver(string arm)
{
	cout << "=====================================" << endl;
	cout << "Opening IKin Catesian Solver for " << arm << " arm." << endl;
	cout << "=====================================" << endl;
	// declare the on-line arm solver
	string solverName = ((string)parameters["solver_name"]->asString()) + "/" + arm ;

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
	cout<<reply.size()<<endl;  

	cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_SET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_LIM);
    cmd.addInt(3);
	cmd.addDouble(-89);
	cmd.addDouble(0);
    iKinPorts[arm]->rpc.write(cmd,reply);
	cout<<reply.size()<<endl;  

	cmd.clear();
    cmd.addVocab(IKINSLV_VOCAB_CMD_SET);
    cmd.addVocab(IKINSLV_VOCAB_OPT_LIM);
    cmd.addInt(4);
	cmd.addDouble(0.02);
	cmd.addDouble(25);
    iKinPorts[arm]->rpc.write(cmd,reply);
	cout<<reply.size()<<endl;  

	cmd.clear();
	cmd.addVocab(IKINSLV_VOCAB_CMD_GET);
	cmd.addVocab(IKINSLV_VOCAB_OPT_POSE);
    iKinPorts[arm]->rpc.write(cmd,reply);
	cout<< "pose : " << reply.toString()<<endl;  

	cmd.clear();
	cmd.addVocab(IKINSLV_VOCAB_CMD_GET);
	cmd.addVocab(IKINSLV_VOCAB_OPT_DOF);
    iKinPorts[arm]->rpc.write(cmd,reply);
	cout<< "dof : " << reply.toString()<<endl;
	
}


void DrumIKin::CloseIKSolver(string arm)
{
	iKinPorts[arm]->in.close();
	iKinPorts[arm]->out.close();
	iKinPorts[arm]->rpc.close();
}

Value DrumIKin::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}

void DrumIKin::InitPositionControl(string partName)
{
	Property options;
    options.put("device", "remote_controlboard");
	options.put("local", ("/reach_manager/position_control/" + partName).c_str());   //local port names

	string remotePortName = "/" + (string)parameters["robot"]->asString() + "/" + partName;
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

void DrumIKin::RobotPositionControl(string partName, const Vector &jointAngles)
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
    //now set the arm joints to the computed value value
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


void DrumIKin::ClosePositionControl(string partName)
{
	polydrivers[partName]->close();
	delete polydrivers[partName];
}

Vector DrumIKin::Solve(const Vector &xd, string partName, double &precision)
{
	Vector resultQ(7);

	Bottle cmd, reply;
	cmd.clear();
	
	cout << "SOLVING WITH " << partName << endl;

	iCubArmCartesianSolver::addTargetOption(cmd,xd);

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
	precision = sqrt(deltaNorm2);
	if(precision < parameters["max_error"]->asDouble())
	{
		cout << "error : " << precision << endl;
		cout << "xd      =" << xdBottle->toString()<<endl;
		cout << "x       =" << xBottle->toString()<<endl;
		cout << "q [deg] =" ;
		for (int i=0; i<qBottle->size(); ++i)
		{
			cout << 180/M_PI*qBottle->get(i).asDouble()<<" , ";
		}
		cout << endl;
	}
	for(int i =0; i<7; ++i)
		resultQ[i] = qBottle->get(i).asDouble();

	return resultQ;
}