#pragma once
#include <yarp/os/all.h>
using namespace yarp::os;

#include <iCub/iKinSlv.h>
using namespace iKin;

#include "SuperPort.h"

#define MODULE_PERIOD 0.0
#define MIN_DIFFERENCE 0.0
#define L 0.36
#define MODULE_NAME "ReachManagerModule"

class ReachManagerModule :
    public Module
{
public:
	struct IKinPort
	{
		SuperPort in;
		Port out;
		Port rpc;
	};

public:
    ReachManagerModule(void);
    virtual ~ReachManagerModule(void);
    virtual bool updateModule();
	virtual double getPeriod();
	virtual bool open(Searchable& config);
	virtual bool close();


protected:
    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;
	map<string, IKinPort *> iKinPorts;
	map<string, iCubArmCartesianSolver *> iKSolvers;
	map<string, Value> parameters;
	Vector leftOrientation, rightOrientation;
	map<string, PolyDriver *> polydrivers;
	map<string, int> nbJoints;
	Vector lastPosition;

private:
	void OpenIKSolver(string arm);
	void CloseIKSolver(string arm);
	Value GetValueFromConfig(Searchable& config, string valueName);
	void RobotPositionControl(string partName, const Vector &jointAngles);
	void InitPositionControl(string partName);
	void ClosePositionControl(string partName);
	Vector Solve(const sig::Vector &xd, string partName, string &resultPart);


};