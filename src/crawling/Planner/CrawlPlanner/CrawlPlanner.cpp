#include "CrawlPlanner.h"
#include <iostream>
using namespace std;
#include <math.h>
#include <string>
#include <time.h> //to delete

#include <glm/gtx/rotate_vector.hpp>
using namespace glm;
#include <glm/gtx/vector_angle.hpp>
using namespace gtx;
#include <glm/gtx.hpp>
using namespace gtx::norm;


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

	parameters["patches_port_name"] = new Value(GetValueFromConfig(config, "patches_port_name"));
	parameters["manager_out_port_name"] = new Value(GetValueFromConfig(config, "manager_out_port_name"));

	ports["patches"] = new BufferedPort<Bottle>;
	string port_name = "/" + (string)parameters["module_name"]->asString() + (string)parameters["patches_port_name"]->asString();
	ports["patches"]->open(port_name.c_str());

	ports["neck_angle"] = new BufferedPort<Bottle>;
	port_name = "/" + (string)parameters["module_name"]->asString() + "/neckAngle";
	ports["neck_angle"]->open(port_name.c_str());

	Network::connect("/icub/head/state:o", ports["neck_angle"]->getName().c_str());

	ports["manager_out"] = new BufferedPort<Bottle>;
	port_name = "/" + (string)parameters["module_name"]->asString() + (string)parameters["manager_out_port_name"]->asString();
	ports["manager_out"]->open(port_name.c_str());

	ports["supervisor_out"] = new BufferedPort<Bottle>;
	ports["supervisor_out"]->open(SUPERVISOR_OUT_PORT_NAME);


	parameters["crawl_command"] = new Value(GetValueFromConfig(config, "crawl_command"));
	parameters["turn_command"] = new Value(GetValueFromConfig(config, "turn_command"));

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
	Bottle *neckAngleBottle = ports["neck_angle"]->read();
	double neckAngle = neckAngleBottle->get(2).asDouble();
	cout << "neck angle : " << neckAngle << endl;

	//return true;
	BuildPotentialField();

    if(ScanFinished(neckAngle))
    {
		SendToSupervisor();

        double angle = ComputeRobotRotation();
		
		potentialField.clear();

        if(abs(angle - previousRotationAngle) < EPSILON_ANGLE)
        {
            return true;
        }
        previousRotationAngle = angle;

        Bottle& managerBottle = ports["manager_out"]->prepare();
        managerBottle.clear();
        if(abs(angle) < EPSILON_ANGLE)
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
	dvec2 potentialGradient = ComputePotentialGradient();

#ifdef _DEBUG
    cout << "potential gradient : (" << potentialGradient.x << "," << potentialGradient.y << ")" << endl;
#endif

    if(potentialGradient.x == 0 && potentialGradient.y == 0)
    {
        return 0;
    }

    dvec2 y(0,1);
    double angle = orientedAngle(y, potentialGradient);

    if(abs(angle) < EPSILON_ANGLE)
    {
        return 0;
    }
    /*else if(abs(abs(angle) - 180) < EPSILON_ANGLE)
    {
        return previousRotationAngle;
    }*/
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

dvec2 CrawlPlanner::ComputePotentialGradient(void) const
{
	dvec2 gradient(0,0);

	if(potentialField.size()>0)
	{
		for(unsigned int i=0; i<potentialField.size(); ++i)
		{
            dvec2 currentPotentialVector = potentialField[i].GetPotentialVector();
            gradient += currentPotentialVector;
		}
		//normalise gradient vector
        if(length2(gradient)>0)
        {
            gradient = normalize(gradient);
        }
	}
	return gradient;
}


int CrawlPlanner::ExistPotential(double x, double y, string color) const
{
	for(unsigned int i=0; i<potentialField.size(); ++i)
	{
		string potentialColor = (potentialField[i].GetPotential()>0) ? "red" : "green";
		if(potentialColor != color)
		{
			continue;
		}
		const dvec2 &potentialPosition = potentialField[i].GetPosition();

		if(x < potentialPosition.x + POTENTIAL_POSITION_PRECISION
			&& x > potentialPosition.x - POTENTIAL_POSITION_PRECISION
			&& y < potentialPosition.y + POTENTIAL_POSITION_PRECISION
			&& y > potentialPosition.y - POTENTIAL_POSITION_PRECISION)
		{
			return i;
		}
	}
	return -1;
}

//double CrawlPlanner::GetNeckAngle(void)
//{
//    ;
//    //bodyAngles["neck_pitch"] = paramBottle->get(1).asDouble();
//    //bodyAngles["neck_roll"] = paramBottle->get(2).asDouble();
//    //bodyAngles["neck_yaw"] = paramBottle->get(3).asDouble();
//    //bodyAngles["torso_roll"] = paramBottle->get(4).asDouble();
//
//
//	bodyAngles["eyes_tilt"] = 0;
//    bodyAngles["neck_pitch"] = 0;
//    bodyAngles["neck_roll"] = 0;
//    bodyAngles["neck_yaw"] = 0;
//    bodyAngles["torso_roll"] = 0;
//
//    return bodyAngles;
//}




void CrawlPlanner::BuildPotentialField()
{
    Bottle *readBottle = ports["patches"]->read(false);
	if(!readBottle)
	{
		return;
	}
    int nbPatches = readBottle->size();

    //potentialField.clear();

    //std::vector<double> rotationAngles;//x, y, z
    //rotationAngles.push_back(degrees(-(M_PI/2 - (bodyAngles["eyes_tilt"] + bodyAngles["neck_pitch"]))));
    //rotationAngles.push_back(degrees(-(M_PI/2 - bodyAngles["neck_roll"]));
    //rotationAngles.push_back(degrees(bodyAngles["neck_yaw"] + bodyAngles["torso_roll"]));

#if _DEBUG
    cout << nbPatches <<" patches detected." << endl;
#endif
    
    //for(unsigned int i=0; i<potentialField.size(); ++i)
    //{
    //    if(IsInVisibleRange(potentialField[i], rotationAngles[2]))
    //    {
    //        potentialField.erase(potentialField.begin() + i);
    //    }
    //}

    for(int i=0; i<nbPatches;++i)
    {
        Bottle *currentPatchBottle = readBottle->get(i).asList();

        double x3D = currentPatchBottle->get(0).asDouble();
        double y3D = currentPatchBottle->get(1).asDouble();
        double z3D = currentPatchBottle->get(2).asDouble();
        //double radius = currentPatchBottle->get(3).asDouble();
		double radius = 0.1;
		int patchID = currentPatchBottle->get(3).asInt();

        //string color = currentPatchBottle->get(3).asString();
		string color = (patchID == RED_ID) ? "red" : "green";


        //convertion of the positions of the patches to body frame.
        //dvec3 position3D(x3D/3, y3D/3, z3D/3);
        //dvec3 visionPosition = rotateX(position3D, rotationAngles[0]);
        //dvec3 bodyPosition = rotateY(visionPosition, rotationAngles[1]);
        //bodyPosition = rotateZ(bodyPosition, rotationAngles[2]);
        dvec2 position2D(y3D, z3D);
        
		int idPatchExists = ExistPotential(position2D.x, position2D.y, color);
		if(idPatchExists>-1)
		{
			dvec2 currentPosition = potentialField[idPatchExists].GetPosition();
			double newX = (currentPosition.x + position2D.x)/2;
			double newY = (currentPosition.y + position2D.y)/2;

			//cout << "MOVED PATCH FROM (" <<  currentPosition.x << " , " << currentPosition.y << ") to (" << newX << " , " << newY << ")" << endl;
			potentialField[idPatchExists].MoveTo(newX, newY);
			continue;
		}
		
		//cout << "patch at (" << bodyPosition.x << "," << bodyPosition.x << ")" << endl;

        if(color == "red")
		{
			Potential potential(position2D.x, position2D.y, radius, RED_POTENTIAL);
			potentialField.push_back(potential);
		}
		else if (color == "green")
		{
			Potential potential(position2D.x, position2D.y, radius, GREEN_POTENTIAL);
			potentialField.push_back(potential);
		}
        //WritePotentialField();

#if _DEBUG
        cout << "received " << color << " patch at (" << x3D << "," << y3D << "," << z3D << ")" << endl;
        cout << "2D position : (" << position2D.x << "," << position2D.y << ")" << endl;
#endif
    }
}



//bool CrawlPlanner::IsInVisibleRange(const Potential &potential, double visionOrientationAngle) const
//{
//    dvec2 y(0,1);
//    double leftLimitAngle = visionOrientationAngle + FOV/2;
//    double rightLimitAngle = visionOrientationAngle - FOV/2;
//
//    dvec2 potentialDirection = normalize(potential.GetPosition());
//    double patchAngle = orientedAngle(y, potentialDirection);
//
//    if((patchAngle < (leftLimitAngle+5)) && (patchAngle > (rightLimitAngle-5)))
//    {
//        return true;
//    }
//    return false;
//}

void CrawlPlanner::SendToSupervisor(void)
{
    Bottle& supervisorBottle = ports["supervisor_out"]->prepare();
    supervisorBottle.clear();

    Bottle potentialFieldBottle;
    potentialFieldBottle.clear();

    dvec2 potentialGradient = ComputePotentialGradient();

    potentialFieldBottle.addDouble(potentialGradient.x);
    potentialFieldBottle.addDouble(potentialGradient.y);

    supervisorBottle.addList() = potentialFieldBottle;

    for(unsigned int i=0; i<potentialField.size(); ++i)
    {
        Bottle patchBottle;
        patchBottle.clear();

        patchBottle.addDouble(potentialField[i].GetPosition().x);
        patchBottle.addDouble(potentialField[i].GetPosition().y);
        patchBottle.addDouble(potentialField[i].GetRadius());
        patchBottle.addDouble(potentialField[i].GetPotential());

        supervisorBottle.addList() = patchBottle;
    }

    ports["supervisor_out"]->write();
}



bool CrawlPlanner::ScanFinished(double neckAngle)
{
    bool scanFinished = false;
    double tetaDot = neckAngle - previousNeckAngle;

    //we define the start/end of the scan period as the point
    //where the head is at it's maximum on the left or on the right
	//i.e. the angle variation changes sign.
    if(tetaDot * previousTetaDot < 0)
    {
        scanFinished = true;
		cout << "===============SCAN FINISHED==================" << endl;
    }
    
    previousTetaDot = tetaDot;
    return scanFinished;
}

