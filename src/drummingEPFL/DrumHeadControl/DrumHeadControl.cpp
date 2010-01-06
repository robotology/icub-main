#include "DrumHeadControl.h"

#include <yarp/math/Math.h>
using namespace yarp::math;
//#include "ctrlMath.h";

#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream>
using namespace std;

DrumHeadControl::DrumHeadControl()
{
	
}

DrumHeadControl::~DrumHeadControl(void)
{
}

bool DrumHeadControl::open(Searchable& config)
{
	cout << "config : " << config.toString() << endl;
	parameters["input_port"] = new Value(GetValueFromConfig(config, "input_port"));
	parameters["output_port"] = new Value(GetValueFromConfig(config, "output_port"));
	parameters["robot"] = new Value(GetValueFromConfig(config, "robot"));
	
	inPort.open(parameters["input_port"]->asString().c_str());
	outPort.open(parameters["output_port"]->asString().c_str());

	InitPolydriver();

	return true;
}


bool DrumHeadControl::close()
{
	inPort.close();
    outPort.close();
	for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}
	return true;
}


bool DrumHeadControl::updateModule(void)
{ 
	Bottle *visionBottle = inPort.read();
	Bottle &outBottle = outPort.prepare();
	outBottle.clear();

	map<string, double> headAngles = GetHeadAngles();

	for(int i=0; i<visionBottle->size(); ++i)
	{
		Bottle *patchBottle=visionBottle->get(i).asList();
		if(patchBottle->size() != 4 || patchBottle->get(0).asDouble() > 100)
		{
			cout << "--------ERROR GETTING THE 3D POSITION OF THE OBJECT ---------- " << endl;
			return true;
		}

		Vector position(3);
		position[0] = patchBottle->get(0).asDouble();
		position[1] = patchBottle->get(1).asDouble();
		position[2] = patchBottle->get(2).asDouble();

		double turnPitchAngle = - atan(position[1]/position[2]);
		double turnYawAngle = - atan(position[0]/position[2]);
		double headPitchAngle = headAngles["pitch"] + turnPitchAngle ;
		double headYawAngle = headAngles["yaw"] + turnYawAngle; 

		Bottle headBottle;
		headBottle.addInt(patchBottle->get(3).asInt());
		headBottle.addDouble(headPitchAngle);
		headBottle.addDouble(0);
		headBottle.addDouble(headYawAngle);
		outBottle.addList() = headBottle;
	}

	outPort.write();

	return true;
}

double DrumHeadControl::getPeriod(void)
{
	return MODULE_PERIOD;
}


Value DrumHeadControl::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}


void DrumHeadControl::InitPolydriver()
{
	Property options;
    options.put("device", "remote_controlboard");
	options.put("local", "/head_control/head");   //local port names

	string remotePortName = "/" + (string)parameters["robot"]->asString() + "/head";
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

map<string, double> DrumHeadControl::GetHeadAngles(void)
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