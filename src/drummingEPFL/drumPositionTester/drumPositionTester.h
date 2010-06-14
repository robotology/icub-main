#pragma once
#include <yarp/os/all.h>
using namespace yarp::os;

#include <yarp/dev/PolyDriver.h>
using namespace yarp::dev;

#include<yarp/sig/Vector.h>
using namespace yarp::sig;

#include <iCub/iKinSlv.h>
using namespace iKin;


#include <map>
#include <fstream>
using namespace std;

#define MODULE_PERIOD 0.05
#define MODULE_NAME "drumPositionTester"

static const string FILE_LEFT = "C:\\Data\\iKin\\left.dat";
static const string FILE_RIGHT = "C:\\Data\\iKin\\right.dat";

static const double MIN_X = -1, MAX_X = 0;
static const double MIN_Y = -0.5, MAX_Y = 0.5;
static const double MIN_Z = -0.5, MAX_Z = 0.5;

static const double STEP = 0.02;

struct Stuff
{
    int a;
    int b;
    Stuff(int mya, int myb)
        :a(mya), b(myb)
    {
    }
};

class DrumPositionTester :
    public Module
{

  
public:
	struct IKinPort
	{
		Port in;
		Port out;
		Port rpc;
	};

public:
    DrumPositionTester(void);
    virtual ~DrumPositionTester(void);
    virtual bool updateModule();
	virtual double getPeriod();
	virtual bool open(Searchable& config);
	virtual bool close();


protected:
	map<string, IKinPort *> iKinPorts;
	map<string, iCubArmCartesianSolver *> iKSolvers;
	map<string, PolyDriver *> polydrivers;
	map<string, int> nbJoints;

    ofstream outFileLeft, outFileRight;
    double x, y, z;

private:
	void OpenIKSolver(string arm);
	void CloseIKSolver(string arm);
	Vector Solve(const Vector &xd, string partName, double &precision);

    void DoStuff(Stuff &stuff);


};

