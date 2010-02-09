#pragma once
#include <yarp/os/all.h>
using namespace yarp::os;

#include <yarp/dev/PolyDriver.h>
using namespace yarp::dev;

#include<yarp/sig/Vector.h>
using namespace yarp::sig;

#include <iCub/iKinSlv.h>
using namespace iKin;

#include "SuperPort.h"

#include <map>
using namespace std;

#define MODULE_PERIOD 0.2
#define MIN_DIFFERENCE 0.02
#define MODULE_NAME "DrumIKin"

//according to the Denavit-Hartenberg parameters on the wiki
#define ARM_Z_OFFSET 110.24
#define ARM_Y_OFFSET 107.74

class DrumIKin :
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
    DrumIKin(void);
    virtual ~DrumIKin(void);
    virtual bool updateModule();
	virtual double getPeriod();
	virtual bool open(Searchable& config);
	virtual bool close();


protected:
    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;
	map<string, IKinPort *> iKinPorts;
	map<string, iCubArmCartesianSolver *> iKSolvers;
	map<string, Value *> parameters;
	map<int, string> armMarkerMapping;
	Vector leftOrientation, rightOrientation;
	map<string, PolyDriver *> polydrivers;
	map<string, int> nbJoints;

private:
	void OpenIKSolver(string arm);
	void CloseIKSolver(string arm);
	Vector GetTargetVector(const Vector &xp, string arm);
	Value GetValueFromConfig(Searchable& config, string valueName);
	void RobotPositionControl(string partName, const Vector &jointAngles);
	void InitPositionControl(string partName);
	void ClosePositionControl(string partName);
	Vector Solve(const Vector &xd, string partName, double &precision);
	map<int, Vector> previousPosition;


};