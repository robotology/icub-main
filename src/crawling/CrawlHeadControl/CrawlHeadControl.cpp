#include "CrawlHeadControl.h"

#include <yarp/math/Math.h>
using namespace yarp::math;
//#include "ctrlMath.h";

#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream>
using namespace std;

CrawlHeadControl::CrawlHeadControl()
{
	
}

CrawlHeadControl::~CrawlHeadControl(void)
{
}

bool CrawlHeadControl::open(Searchable& config)
{
	cout << "config : " << config.toString() << endl;
	parameters["input_port"] = GetValueFromConfig(config, "input_port");
	parameters["output_port"] = GetValueFromConfig(config, "output_port");
	parameters["robot"] = GetValueFromConfig(config, "robot");
	parameters["head_control_command_code"] = GetValueFromConfig(config, "head_control_command_code");
	parameters["head_control_mode_dist"] = GetValueFromConfig(config, "head_control_mode_dist");
	cout << "head_control_mode_dist : " << parameters["head_control_mode_dist"].asDouble() << endl;
	
	inPort.open(parameters["input_port"].asString().c_str());
	outPort.open(parameters["output_port"].asString().c_str());

	InitPolydriver();

	return true;
}


bool CrawlHeadControl::close()
{
	inPort.close();
    outPort.close();
	return true;
}


bool CrawlHeadControl::updateModule(void)
{ 
	Bottle *visionBottle = inPort.read();
	double minDistanceSQR = 999999;
	bool patchToReach = false;

	Vector nearestPatchPosition(3);

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

		double distanceSQR = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];

		if(distanceSQR<minDistanceSQR)
		{
			nearestPatchPosition[0] = position[0];
			nearestPatchPosition[1] = position[1];
			nearestPatchPosition[2] = position[2];
			patchToReach = true;
			minDistanceSQR = distanceSQR;
		}
	}

	if(!patchToReach)
	{
		return true;
	}

	map<string, double> headAngles = GetHeadAngles();

	cout << "distance : " << sqrt(minDistanceSQR) << endl;
	cout << "min distance : " << parameters["head_control_mode_dist"].asDouble() << endl;
	if(minDistanceSQR<pow(parameters["head_control_mode_dist"].asDouble(),2))
	{
		if(nearestPatchPosition[2]<0.001 && nearestPatchPosition[2]>-0.001)
		{
			return true;
		}
		double turnPitchAngle = - atan(nearestPatchPosition[1]/nearestPatchPosition[2]);
		double turnYawAngle = - atan(nearestPatchPosition[0]/nearestPatchPosition[2]);
		double headPitchAngle = headAngles["pitch"] + turnPitchAngle ;
		double headYawAngle = headAngles["yaw"] + turnYawAngle; 
		cout << "current pitch angle : " << headAngles["pitch"] << endl;
		cout << "current yaw angle : " << headAngles["yaw"] << endl;
		cout << "turn pitch angle : " << turnPitchAngle << endl;
		cout << "turn yaw angle : " << turnYawAngle << endl;
		cout << "Head angle pitch : " << headPitchAngle << endl;
		cout << "Head angle Yaw : " << headYawAngle << endl;
		Bottle &outBottle = outPort.prepare();
		outBottle.clear();
		outBottle.addInt(parameters["head_control_command_code"].asInt());
		outBottle.addDouble(headPitchAngle);
		outBottle.addDouble(headYawAngle);
		outPort.write();
	}

	return true;
}

double CrawlHeadControl::getPeriod(void)
{
	return MODULE_PERIOD;
}


Value CrawlHeadControl::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}


void CrawlHeadControl::InitPolydriver()
{
	Property options;
    options.put("device", "remote_controlboard");
	options.put("local", "/head_control/head");   //local port names

	string remotePortName = "/" + (string)parameters["robot"].asString() + "/head";
	options.put("remote", remotePortName.c_str());         //where we connect to

    // create a device
    polydriver = new PolyDriver(options);
    if (!polydriver->isValid()) 
	{
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return;
    }

    IPositionControl *pos;
    IEncoders *encs;

    if (!(polydriver->view(pos) || polydriver->view(encs))) 
	{
        printf("Problems acquiring interfaces\n");
        return;
    }

    pos->getAxes(&nbJoints);
}

map<string, double> CrawlHeadControl::GetHeadAngles(void)
{
	map<string, double> headAngles;
	IPositionControl *pos;
    IEncoders *encs;

    if (!(polydriver->view(pos) && polydriver->view(encs))) 
	{
        printf("Problems acquiring interfaces\n");
        return headAngles;
    }
	double *encoders = new double[nbJoints];
	encs->getEncoders(encoders);
	headAngles["pitch"] = encoders[0]*3.14/180;
	headAngles["yaw"] = encoders[2]*3.14/180;
	return headAngles;
	delete[] encoders;
}