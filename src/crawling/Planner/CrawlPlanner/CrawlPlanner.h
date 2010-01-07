#pragma once

#include <yarp/os/all.h>
using namespace yarp::os;
#include "Potential.h"
#include <iostream> 
#include <string>
#include <vector>
#include <map>
using namespace std;

#include <glm/glm.hpp>
#include <glm/gtc/double_float.hpp>
using namespace std;

#define SUPERVISOR_OUT_PORT_NAME "/CrawlPlanner/supervisor/out"


#ifdef _DEBUG
#define MODULE_PERIOD 0.1

#else
#define MODULE_PERIOD 0.2
#endif

#define NB_BODY_ANGLES 5
#define FOV 45
#define NECK_MVT_AMPLITUDE 1

#define DELTA_DISPLACEMENT 0.2
#define MAX_ROTATION_ANGLE 30

#define POTENTIAL_POSITION_PRECISION 0.1
#define EPSILON_ANGLE 5
#define EPSILON_LENGHT 0.01

#define GREEN_POTENTIAL -1
#define RED_POTENTIAL 1
#define REACHING_DISTANCE 0.4

#define RED_ID 3
#define GREEN_ID 76

class CrawlPlanner :
    public Module
{
public:
    //enum Movement{LEFT, RIGHT, STRAIT};
private:
    std::vector<Potential> potentialField;
    //std::vector<Potential> tempPotentialField;

protected:
    map<string, BufferedPort<Bottle>* > ports;
    
    double previousRotationAngle;  
    double previousNeckAngle;  
    double previousTetaDot;  
	map<string, Value *> parameters;

public:
    CrawlPlanner(void);
    ~CrawlPlanner(void);
	virtual bool open(Searchable &config);
	virtual bool close();
    virtual double getPeriod(void);
    virtual bool updateModule(void);
	Value GetValueFromConfig(Searchable& config, string valueName);

private:
    //Movement ComputeRobotDisplacement(void);
    double ComputeRobotRotation(void) const;
	dvec2 ComputePotentialGradient(void) const;
	int ExistPotential(double x, double y, string color) const;
    //map<string, double>GetBodyAngles(void);
    void BuildPotentialField();
    bool IsInVisibleRange(const Potential &potential, double visionOrientationAngle) const;
    void SendToSupervisor(void);
    bool ScanFinished(double neckAngle);



    inline void WritePotentialField()
    {
        for(unsigned int i=0; i<potentialField.size(); ++i)
        {
            cout << "potential : (" << potentialField[i].GetPotentialVector().x << "," << potentialField[i].GetPotentialVector().y << ")" << endl;
        }
    }

};
