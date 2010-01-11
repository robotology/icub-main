#include "CrawlPlanner.h"
#include <iostream>
using namespace std;
#include <math.h>
#include <string>
#include <time.h> //to delete

#include <iCub/ctrlMath.h>
using namespace ctrl;

#include <yarp/math/Math.h>
using namespace yarp::math;

#include "../Common/Common.h"

CrawlPlanner::CrawlPlanner(void) : previousRotationAngle(0), previousNeckAngle(0), previousTetaDot(0)
{
}

CrawlPlanner::~CrawlPlanner(void)
{
}


double CrawlPlanner::getPeriod(void)
{
    return MODULE_PERIOD;
}

bool CrawlPlanner::close()
{
	//close all ports;
	for ( map<string, BufferedPort<Bottle>* >::const_iterator iter = ports.begin();
      iter != ports.end(); ++iter )
	{
		iter->second->close();
		delete iter->second;
	}

	for (map<string, Value *>::iterator it = parameters.begin(); it!=parameters.end(); ++it)
	{
		delete it->second;
	}
	return true;
}

bool CrawlPlanner::open(Searchable &config)
{
	parameters["module_name"] = new Value(GetValueFromConfig(config, "module_name"));

	parameters["objects_port_name"] = new Value(GetValueFromConfig(config, "objects_port_name"));
	parameters["manager_out_port_name"] = new Value(GetValueFromConfig(config, "manager_out_port_name"));

	ports["objects"] = new BufferedPort<Bottle>;
	string port_name = "/" + (string)parameters["module_name"]->asString() + (string)parameters["objects_port_name"]->asString();
	ports["objects"]->open(port_name.c_str());

	ports["head_encoders"] = new BufferedPort<Bottle>;
	port_name = "/" + (string)parameters["module_name"]->asString() + "/head_encoders";
	ports["head_encoders"]->open(port_name.c_str());

	bool connected = false;
	cout << "Connecting to /icub/head/state:o ..." << endl;
	for(int i=0; i<3; ++i)
	{
		cout << "Attempt " << i << " ..." << endl;
		if(Network::connect("/icub/head/state:o", ports["head_encoders"]->getName().c_str()))
		{
			connected = true;
		}
	}
	if(connected)
	{
		cout << "Connected." << endl;
	}
	else
	{
		cout << "Failed to connect to port /icub/head/state:o." << endl
			<< "iCubInterface must be running for this module to work" << endl;
		return false;
	}
	

	ports["manager_out"] = new BufferedPort<Bottle>;
	port_name = "/" + (string)parameters["module_name"]->asString() + (string)parameters["manager_out_port_name"]->asString();
	ports["manager_out"]->open(port_name.c_str());

	ports["supervisor_out"] = new BufferedPort<Bottle>;
	ports["supervisor_out"]->open(SUPERVISOR_OUT_PORT_NAME);


	parameters["crawl_command"] = new Value(GetValueFromConfig(config, "crawl_command"));
	parameters["turn_command"] = new Value(GetValueFromConfig(config, "turn_command"));

	parameters["obstacle_ID"] = new Value(GetValueFromConfig(config, "obstacle_ID"));
	parameters["goal_ID"] = new Value(GetValueFromConfig(config, "goal_ID"));

	return true;
}

Value CrawlPlanner::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}

bool CrawlPlanner::updateModule(void)
{
	Bottle *headBottle = ports["head_encoders"]->read();
	double neckAngle = headBottle->get(2).asDouble();

	BuildPotentialField();

    if(ScanFinished(neckAngle))
    {
		SendToSupervisor();

        double angle = ComputeRobotRotation();
		
		potentialField.clear();

        if(fabs(angle - previousRotationAngle) < EPSILON_ANGLE)
        {
			cout << "No need to turn : angle : " << angle << endl;
            return true;
        }
        previousRotationAngle = angle;

        Bottle& managerBottle = ports["manager_out"]->prepare();
        managerBottle.clear();
        if(fabs(angle) < EPSILON_ANGLE)
        {
			cout << "No need to turn (angle = " << angle << ")" << endl;
			managerBottle.addInt(parameters["crawl_command"]->asInt());
        }
        else
        {
			cout << "turning with angle " << angle << endl;
            managerBottle.addInt(parameters["turn_command"]->asInt());
            managerBottle.addDouble(radians(angle));
        }
        ports["manager_out"]->write();

    }

    return true;
}

double CrawlPlanner::ComputeRobotRotation(void) const
{
	Vector potentialGradient = ComputePotentialGradient();

    if(potentialGradient[0] == 0 && potentialGradient[1] == 0)
    {
        return 0;
    }

    Vector u(2);
	u[0] = 0;
	u[1] = 1;
    double angle = orientedAngle(u, potentialGradient);

    if(fabs(angle) < EPSILON_ANGLE)
    {
        return 0;
    }
    else if(angle>MAX_ROTATION_ANGLE)
    {
        return MAX_ROTATION_ANGLE;
    }
    else if(angle < -MAX_ROTATION_ANGLE)
    {
        return -MAX_ROTATION_ANGLE;
    }

    return angle;
}

Vector CrawlPlanner::ComputePotentialGradient(void) const
{
	Vector gradient(2);
	gradient.zero();

	if(potentialField.size()>0)
	{
		for(unsigned int i=0; i<potentialField.size(); ++i)
		{
            Vector currentPotentialVector = potentialField[i].GetPotentialVector();
            gradient = gradient + currentPotentialVector;
		}
		//normalise gradient vector
		Normalize(gradient);
	}
	return gradient;
}


int CrawlPlanner::ExistPotential(double x, double y, int objectID)
{
	for(unsigned int i=0; i<potentialField.size(); ++i)
	{
		int potentialID = (potentialField[i].GetPotential()>0) ? (int)parameters["obstacle_ID"]->asInt() : (int)parameters["goal_ID"]->asInt();
		if(potentialID != objectID)
		{
			continue;
		}
		const Vector &potentialPosition = potentialField[i].GetPosition();

		if(x < potentialPosition[0] + POTENTIAL_POSITION_PRECISION
			&& x > potentialPosition[0] - POTENTIAL_POSITION_PRECISION
			&& y < potentialPosition[1] + POTENTIAL_POSITION_PRECISION
			&& y > potentialPosition[1] - POTENTIAL_POSITION_PRECISION)
		{
			return i;
		}
	}
	return -1;
}


void CrawlPlanner::BuildPotentialField()
{
    Bottle *objectBottle = ports["objects"]->read(false);
	if(!objectBottle)
	{
		return;
	}
    int nbObjects = objectBottle->size();

   
    for(int i=0; i < nbObjects;++i)
    {
        Bottle *currentObjectBottle = objectBottle->get(i).asList();

        double x3D = currentObjectBottle->get(0).asDouble();
        double y3D = currentObjectBottle->get(1).asDouble();
        double z3D = currentObjectBottle->get(2).asDouble();
		double radius = 0.1;

		int objectID = currentObjectBottle->get(3).asInt();

        Vector position2D(2);
		position2D[0] = y3D;
		position2D[1] = z3D;
        
		int idObjectExists = ExistPotential(position2D[0], position2D[1], objectID);
		if(idObjectExists > -1)
		{
			const Vector &currentPosition = potentialField[idObjectExists].GetPosition();
			double newX = (currentPosition[0] + position2D[0])/2;
			double newY = (currentPosition[1] + position2D[1])/2;

			potentialField[idObjectExists].MoveTo(newX, newY);
			continue;
		}

        if(objectID == parameters["obstacle_ID"]->asInt())
		{
			Potential potential(position2D[0], position2D[1], radius, OBSTACLE_POTENTIAL);
			potentialField.push_back(potential);
		}
		else if (objectID == parameters["goal_ID"]->asInt())
		{
			Potential potential(position2D[0], position2D[1], radius, GOAL_POTENTIAL);
			potentialField.push_back(potential);
		}
        //WritePotentialField();
    }
}

void CrawlPlanner::SendToSupervisor(void)
{
    Bottle& supervisorBottle = ports["supervisor_out"]->prepare();
    supervisorBottle.clear();

    Bottle potentialFieldBottle;
    potentialFieldBottle.clear();

    Vector potentialGradient = ComputePotentialGradient();

    potentialFieldBottle.addDouble(potentialGradient[0]);
    potentialFieldBottle.addDouble(potentialGradient[1]);

    supervisorBottle.addList() = potentialFieldBottle;

    for(unsigned int i=0; i<potentialField.size(); ++i)
    {
        Bottle patchBottle;
        patchBottle.clear();

        patchBottle.addDouble(potentialField[i].GetPosition()[0]);
        patchBottle.addDouble(potentialField[i].GetPosition()[1]);
        patchBottle.addDouble(potentialField[i].GetRadius());
        patchBottle.addDouble(potentialField[i].GetPotential());

        supervisorBottle.addList() = patchBottle;
    }

    ports["supervisor_out"]->write();
}



bool CrawlPlanner::ScanFinished(double neckAngle)
{
 //   bool scanFinished = false;
 //   double tetaDot = neckAngle - previousNeckAngle;

 //   //we define the start/end of the scan period as the point
 //   //where the head is at it's maximum on the left or on the right
	////i.e. the angle variation changes sign.
 //   if(tetaDot * previousTetaDot < 0)
 //   {
 //       scanFinished = true;
	//	cout << "===============SCAN FINISHED==================" << endl;
 //   }
 //   
 //   previousTetaDot = tetaDot;
 //   return scanFinished;
	return true;
}

