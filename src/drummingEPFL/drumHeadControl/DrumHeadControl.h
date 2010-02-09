#pragma once
#include <yarp/os/all.h>
using namespace yarp::os;

#include <yarp/sig/Vector.h>
using namespace yarp::sig;

#include <yarp/dev/PolyDriver.h>
using namespace yarp::dev;

#include <math.h>
#include <map>
#include <string>
using namespace std;



#define L 0.36
#define MODULE_PERIOD 0.2

class DrumHeadControl :
    public Module
{

public:
    DrumHeadControl(void);
    virtual ~DrumHeadControl(void);
    virtual bool updateModule();
	virtual double getPeriod();
	virtual bool open(Searchable& config);
	virtual bool close();


protected:
    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;
	map<string, Value *> parameters;
	PolyDriver *polydriver;
	int nbJoints;

private:
	Value GetValueFromConfig(Searchable& config, string valueName);
	void InitPolydriver();
	map<string, double> GetHeadAngles(void);



};