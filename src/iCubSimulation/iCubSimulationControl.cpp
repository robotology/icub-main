// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * Thanks to Ruben Smits for the Velocity Control limits patch
 *
 */

/**
 * \file iCubSimulationControl.cpp
 * \brief This file is the yarp interface of the iCubSimulation. It initializes the joints number and names, and deals with the PID motion (position control velocity control)
 * \author Vadim Tikhanoff, Paul Fitzpatrick, Giorgio Metta
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/
#ifdef _MSC_VER
#pragma warning(disable:4355)  //for VC++, no precision loss complaints
#endif
/// general purpose stuff.
#include <yarp/os/Time.h>
#include <yarp/os/ConstString.h>

#include <math.h>
#include <string>

///specific to this device driver.
#include "iCubSimulationControl.h"
#include "OdeInit.h"
#include "pidfilter.h"

//extern OdeInit odeinit;
extern OdeInit& getOdeInit();
 #define odeinit (getOdeInit())

#include "ControlBoardInterfacesImpl_without_ace.inl"

using namespace yarp::os;
using namespace yarp::dev;
static bool NOT_YET_IMPLEMENTED(const char *txt)
{
    ACE_OS::fprintf(stderr, "%s not yet implemented for iCubSimulationControl\n", txt);

    return false;
}

//////////////////////////////////

// establish a mapping from the model to the external axes of control
// PENDING: deal with linked series of joints

class SimControl {
private:
    int number;
	std::string unit;
    dJointID *joint;
    dReal *speed;
    double speedSetpoint;
    bool hinged;
    int universal;
    int sign;
    SimControl *sub;
    int subLength;
    bool active;
    double vel;
    double acc;
    PidFilter filter;

    SimControl *left, *right, *peer;
    int verge;
public:
    SimControl() : filter(6,0.3,0.0,100) {
        sub = NULL;
        joint = NULL;
        speed = NULL;
        active = false;
        left = NULL;
        right = NULL;
        verge = 0;
        speedSetpoint = 0;
        number = -1;
        vel = 1;
        acc = 1;
    }

    virtual ~SimControl() {
        if (sub!=NULL) {
            delete[] sub;
            sub = NULL;
        }
    }

    SimControl *nest(int len) {
        if (sub!=NULL) {
            delete[] sub;
            sub = NULL;
        }
        sub = new SimControl[len];
        subLength = len;
        return sub;
    }

    SimControl *at(int index) {
        if (sub!=NULL) {
            return sub+index;
        }
        return NULL;
    }

    //differential pair
    void init(SimControl& left,
              SimControl& right,
              SimControl& peer,
              int sgn) {
        active = true;
        verge = sgn;
        sign = 1;
        this->left = &left;
        this->right = &right;
        this->peer = &peer;
    }

    //ordinary joint
    void init(const char *unit,
              const char *type,
              int index,
              int sign) {
        active = true;
		number = index;
        this->sign = sign;
		this->unit = unit;
        universal = 0;
        ConstString sunit = unit;
        ConstString stype = type;
        printf("Motor %s %s %d %d\n", unit, type, index, sign);
        if (stype=="hinge") {
            hinged = true;
            universal = 0;
        } else if (stype=="universalAngle1") {
            hinged = false;
            universal = 1;
        } else if (stype=="universalAngle2") {
            hinged = false;
            universal = 2;
        } else {
            printf("Unknown axis type %s\n", type);
            exit(1);
        }
        if (sunit=="leftarm") {
            joint = &(odeinit._iCub->LAjoints[index]);
            if (hinged) {
                speed = &(odeinit._iCub->la_speed[index]);
            } else if (universal==2) {
                speed = &(odeinit._iCub->la_speed1[index]);
            } else {
                speed = &(odeinit._iCub->la_speed[index]);
            }
        } else if (sunit=="rightarm") {
            joint = &(odeinit._iCub->RAjoints[index]);
            if (hinged) {
                speed = &(odeinit._iCub->ra_speed[index]);
            } else if (universal==2) {
                speed = &(odeinit._iCub->ra_speed1[index]);
            } else {
                speed = &(odeinit._iCub->ra_speed[index]);
            }
		} 
		else if (sunit=="head") {
            joint = &(odeinit._iCub->Hjoints[index]);
            speed = &(odeinit._iCub->h_speed[index]);
        } 
		else if (sunit=="leftleg") {
			joint = &(odeinit._iCub->LLegjoints[index]);
            if (hinged) {
			speed = &(odeinit._iCub->LLeg_speed[index]);	
            }
		}
		else if (sunit=="rightleg") {
            joint = &(odeinit._iCub->RLegjoints[index]);
            if (hinged) {
              speed = &(odeinit._iCub->RLeg_speed[index]);
            }
		}
		else if (sunit=="torso") {
              joint = &(odeinit._iCub->Torsojoints[index]);
            if (hinged) {
              speed = &(odeinit._iCub->Torso_speed[index]);
            }
		}
		else {
            printf("Unknown body unit %s\n", unit);
            exit(1);
        }
    }

    double getAngle() {
        return getAngleRaw()*sign;
    }

    double getVelocity() {
        return getVelocityRaw()*sign;
    }

    double getAngleRaw() {
        //printf("GETTING ANGLE, joint number %d\n", number);
        if (verge==0) {
            if (hinged) {
                return dJointGetHingeAngle(*joint);
            } else if (universal == 1) {
                return dJointGetUniversalAngle1(*joint);
            } else if (universal == 2) {
                return dJointGetUniversalAngle2(*joint);
            } else {
                return 0;
            }
        } else {
            double result = (left->getAngleRaw() + 
                             verge*right->getAngleRaw());
            //printf("verger %d angle %g\n", verge, result);
            return result;
		}
    }

    double getVelocityRaw() {
		//printf("Joint arm 0 %lf   arm 1 %lf   arm 2 %lf   arm 3 %lf\n",dJointGetHingeAngle(odeinit._iCub->LAjoints[0]),dJointGetHingeAngle(odeinit._iCub->LAjoints[1]),dJointGetHingeAngle(odeinit._iCub->LAjoints[2]),dJointGetHingeAngle(odeinit._iCub->LAjoints[3]));
       // printf("GETTING ANGLE, joint number %d\n", number);
        if (verge==0) {
			if (joint == NULL){
				return 0;
			}
            if (hinged) {
                return dJointGetHingeAngleRate(*joint);
            } else if (universal == 1) {
                return dJointGetUniversalAngle1Rate(*joint);
            } else if (universal == 2) {
                return dJointGetUniversalAngle2Rate(*joint);
            } else {
                return 0;
            }
        } else {
            double result = (left->getVelocityRaw() + 
                             verge*right->getVelocityRaw());
            //printf("verger %d angle %g\n", verge, result);
            return result;
        }
    }

    double getSpeedSetpoint() {
        return speedSetpoint;
    }

    void setControlParameters(double vel, double acc) {
        this->vel = vel;
        this->acc = acc;
        if (sub!=NULL) {
            for (int i=0; i<subLength; i++) {
                sub[i].setControlParameters(vel,acc);
            }
        }
        if (verge==1) {
            left->setControlParameters(vel,acc);
            right->setControlParameters(vel,acc);
        }
    }

    void setPosition(double target) {
        //printf("TAKING NO ACTION\n");
        double error = target - getAngleRaw()*sign;
        double ctrl = filter.pid(error);
		//printf("unit is %s, number is %d\n", unit.c_str(), number);
        setVelocityRaw(ctrl*sign*vel);
        //setVelocityRaw(error*sign*vel);
        if (sub!=NULL) {
            for (int i=0; i<subLength; i++) {
                sub[i].setPosition(target);
            }
        }
		}

    void setVelocity(double target) {
        if (sub!=NULL) {
            for (int i=0; i<subLength; i++) {
                sub[i].setVelocity(target);
            }
        }
        setVelocityRaw(sign*target);
    }

    void setVelocityRaw(double target) {
        speedSetpoint = target;
        if (verge==0) {
			if (speed != NULL){
            (*speed) = speedSetpoint;
			}
       } else {
            if (fabs(target)>0.1) {
                //printf("verger %d velocity %g\n", verge, target);
           }
            double altSpeed = peer->getSpeedSetpoint();
            if (verge==1) {
                left->setVelocityRaw(speedSetpoint+altSpeed);
                right->setVelocityRaw(speedSetpoint-altSpeed);
            }
		}
	}
};

#define MAX_PART 10
#define MAX_AXIS 40

#define PART_ARM_LEFT 1
#define PART_ARM_RIGHT 2
#define PART_HEAD 3
#define PART_HEAD_RAW 9
#define PART_LEG_LEFT 4
#define PART_LEG_RIGHT 5
#define PART_TORSO 6


static SimControl simControl[MAX_PART][MAX_AXIS];
static bool simLinked = false;

static SimControl& getController(int part, int axis) {
    return simControl[part][axis];
}

static void linkController() {
    if (!simLinked) {
		
        ////////////////////////////////////////////////////////////
        // Setting up the head

        int head = PART_HEAD;
        int rawHead = PART_HEAD_RAW;
		if (odeinit._iCub->actHead == "on"){      
		const char *headName = "head";
       // for (int i=0; i<4; i++) {
            getController(head,0).init(headName,"hinge",0,+1);
			getController(head,1).init(headName,"hinge",1,-1);
			getController(head,2).init(headName,"hinge",2,-1);
			getController(head,3).init(headName,"hinge",3,+1);
       // }
        // for the eyes, we need to map vergence/version onto
        // individual hinge joints
        getController(rawHead,4).init(headName,"hinge",4,-1);
        getController(rawHead,5).init(headName,"hinge",5,+1);
       
		getController(head,4).init(getController(rawHead,4),
                                   getController(rawHead,5),
                                   getController(head,5),
                                   +1);
        getController(head,5).init(getController(rawHead,4),
                                   getController(rawHead,5),
                                   getController(head,4),
                                   -1);
		}
        ////////////////////////////////////////////////////////////
			// Setting up the left and right arms

		for (int arm = PART_ARM_LEFT; arm <= PART_ARM_RIGHT; arm++) {
			const char *armName = (arm==PART_ARM_LEFT)?"leftarm":"rightarm";
			
			if (arm == PART_ARM_RIGHT && odeinit._iCub->actRArm == "on"){
				getController(arm,0).init(armName,"hinge",0,-1);
				getController(arm,1).init(armName,"hinge",1,+1);
				getController(arm,2).init(armName,"hinge",2,-1);
				getController(arm,3).init(armName,"hinge",3,+1);
				getController(arm,4).init(armName,"hinge",4,-1);
				getController(arm,5).init(armName,"universalAngle1",5,-1);
				getController(arm,6).init(armName,"universalAngle2",5,-1);
			}
		
			if (arm == PART_ARM_LEFT && odeinit._iCub->actLArm == "on"){
				getController(arm,0).init(armName,"hinge",0,-1);
				getController(arm,1).init(armName,"hinge",1,-1);
				getController(arm,2).init(armName,"hinge",2,+1);
				getController(arm,3).init(armName,"hinge",3,+1);
				getController(arm,4).init(armName,"hinge",4,+1);
				getController(arm,5).init(armName,"universalAngle1",5,+1);
				getController(arm,6).init(armName,"universalAngle2",5,-1);
		
			}
			if (arm == PART_ARM_RIGHT && odeinit._iCub->actRHand == "on"){
				getController(arm,7).init(armName,"hinge",6,+1);
				SimControl *sub = getController(arm,7).nest(1);
				sub[0].init(armName,"hinge",8,-1);

				getController(arm,8).init(armName,"universalAngle1",22,-1);//thumb
				getController(arm,9).init(armName,"universalAngle2",22,-1);//thumb
				
				getController(arm,10).init(armName,"hinge",23,-1);
				sub = getController(arm,10).nest(1);
				sub[0].init(armName,"hinge",24,-1);
        
				getController(arm,11).init(armName,"hinge",10,-1);//index proximal
				getController(arm,12).init(armName,"hinge",14,-1);
				sub = getController(arm,12).nest(1);
				sub[0].init(armName,"hinge",18,-1);

				getController(arm,13).init(armName,"hinge",11,-1);//middle finger
				getController(arm,14).init(armName,"hinge",15,-1);
				sub = getController(arm,14).nest(1);
				sub[0].init(armName,"hinge",19,-1);
			     
				getController(arm,15).init(armName,"hinge",12,-1);//ring + pinky
				sub = getController(arm,15).nest(2);
				sub[0].init(armName,"hinge",16,-1);
				sub[1].init(armName,"hinge",20,-1);
			}
			if (arm == PART_ARM_LEFT && odeinit._iCub->actLHand == "on" ){
				getController(arm,7).init(armName,"hinge",6,+1);
				SimControl *sub = getController(arm,7).nest(1);
				sub[0].init(armName,"hinge",8,-1);

				/*getController(arm,8).init(armName,"hinge",22,+1);//thumb
				getController(arm,9).init(armName,"hinge",23,+1);
				getController(arm,10).init(armName,"hinge",24,+1);
				*/
				getController(arm,8).init(armName,"universalAngle1",22,+1);//thumb
				getController(arm,9).init(armName,"universalAngle2",22,-1);//thumb
				
				getController(arm,10).init(armName,"hinge",23,+1);
				sub = getController(arm,10).nest(1);
				sub[0].init(armName,"hinge",24,+1);
        
				getController(arm,11).init(armName,"hinge",10,+1);//index proximal
				getController(arm,12).init(armName,"hinge",14,+1);
				sub = getController(arm,12).nest(1);
				sub[0].init(armName,"hinge",18,+1);

				getController(arm,13).init(armName,"hinge",11,+1);//middle finger
				getController(arm,14).init(armName,"hinge",15,+1);
				sub = getController(arm,14).nest(1);
				sub[0].init(armName,"hinge",19,+1);
			     
				getController(arm,15).init(armName,"hinge",12,+1);//ring + pinky
				sub = getController(arm,15).nest(2);
				sub[0].init(armName,"hinge",16,+1);
				sub[1].init(armName,"hinge",20,+1);
			}
		}

		if (odeinit._iCub->actLegs == "on"){
			for (int leg = PART_LEG_LEFT; leg <= PART_LEG_RIGHT; leg++) {
			const char *legName = (leg==PART_LEG_LEFT)?"leftleg":"rightleg";
   //       changed for demo look below for previous joint setup
				if (leg == PART_LEG_RIGHT){
					getController(leg,0).init(legName,"hinge",5,-1);
					getController(leg,1).init(legName,"hinge",4,-1);
					getController(leg,2).init(legName,"hinge",3,-1);
					getController(leg,3).init(legName,"hinge",2,-1);
					getController(leg,4).init(legName,"hinge",1,+1);
					getController(leg,5).init(legName,"hinge",0,-1);
				}else{
					getController(leg,0).init(legName,"hinge",5,-1);
					getController(leg,1).init(legName,"hinge",4,+1);
					getController(leg,2).init(legName,"hinge",3,+1);
					getController(leg,3).init(legName,"hinge",2,-1);
					getController(leg,4).init(legName,"hinge",1,+1);
					getController(leg,5).init(legName,"hinge",0,+1);
				}
			}
		}

		if (odeinit._iCub->actTorso == "on"){
		  int torso = PART_TORSO;
		  const char *torsoName = "torso";
          getController(torso,0).init(torsoName,"hinge",2,+1);
		  getController(torso,1).init(torsoName,"hinge",1,+1);
		  getController(torso,2).init(torsoName,"hinge",0,-1);
		}
        simLinked = true;
    }
}

class SimControlManager {
public:
    SimControl& control(int part, int axis) {
        linkController();
        return ::getController(part, axis);
    }
};

static SimControlManager manager;
bool isExperimental = false;


//////////////////////////////////

iCubSimulationControl::iCubSimulationControl() : 
    ImplementPositionControl<iCubSimulationControl, IPositionControl>(this),
   ImplementVelocityControl<iCubSimulationControl, IVelocityControl>(this),
    ImplementPidControl<iCubSimulationControl, IPidControl>(this),
    ImplementEncoders<iCubSimulationControl, IEncoders>(this),
    ImplementControlCalibration<iCubSimulationControl, IControlCalibration>(this),
    ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>(this),
    ImplementControlLimits<iCubSimulationControl, IControlLimits>(this),/* */
    _done(0),
    _mutex(1)
{
    _opened = false;
}


iCubSimulationControl::~iCubSimulationControl ()
{
}

bool iCubSimulationControl::open(yarp::os::Searchable& config) {

    Searchable& p = config;

	if (!p.check("GENERAL","section for general motor control parameters")) {
        ACE_OS::fprintf(stderr, "Cannot understand configuration parameters\n");
        return false;
    }
	
	int TypeArm = p.findGroup("GENERAL").check("Type",Value(1),
                                          "what did the user select?").asInt();

	int numTOTjoints = p.findGroup("GENERAL").check("TotalJoints",Value(1),
                                          "Number of total joints").asInt();

	double velocity = p.findGroup("GENERAL").check("Vel",Value(1),
                                          "Default velocity").asDouble();
	int i;

	_mutex.wait();
	partSelec = TypeArm;
	
	njoints = numTOTjoints;

	vel = velocity;

	angleToEncoder = allocAndCheck<double>(njoints);
    zeros = allocAndCheck<double>(njoints);
    
    limitsMin = allocAndCheck<double>(njoints);
    limitsMax = allocAndCheck<double>(njoints);
    torqueLimits = allocAndCheck<double>(njoints);
    
    refSpeed = allocAndCheck<double>(njoints);
    refAccel = allocAndCheck<double>(njoints);
    controlP = allocAndCheck<double>(njoints);    

    axisMap = allocAndCheck<int>(njoints);
//  jointNames = new ConstString[njoints];

    current_pos = allocAndCheck<double>(njoints);
    current_vel = allocAndCheck<double>(njoints);
    next_pos = allocAndCheck<double>(njoints);
    next_vel = allocAndCheck<double>(njoints);
    inputs = allocAndCheck<int>(njoints);
    vels = allocAndCheck<double>(njoints);

    error_tol = allocAndCheck<double>(njoints);
    
//  joint_dev = new DeviceTag[njoints];

    motor_on = allocAndCheck<bool>(njoints);
    for(int i = 0;i<njoints;i++)
        motor_on[i] = false;

	/////////////////////////
    /*   GENERAL           */
    /////////////////////////

	Bottle& xtmp = p.findGroup("GENERAL").findGroup("AxisMap","a list of reordered indices for the axes");
    
	if (xtmp.size() != njoints+1) {
        ACE_OS::printf("AxisMap does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) axisMap[i-1] = xtmp.get(i).asInt();

	 xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
	if (xtmp.size() != njoints+1) {
        ACE_OS::printf("Encoder does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) angleToEncoder[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
	if (xtmp.size() != njoints+1) {
        ACE_OS::printf("Zeros does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) zeros[i-1] = xtmp.get(i).asDouble();
	////////////////////////
    /*   LIMITS           */
    ////////////////////////
    xtmp = p.findGroup("LIMITS").findGroup("Max","access the joint limits max");
    if(xtmp.size() != njoints+1)
        {
            ACE_OS::printf("Not enough max joint limits\n");
            return false;
        }
    for( i =1;i<xtmp.size();i++) 
        limitsMax[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];
        
    xtmp = p.findGroup("LIMITS").findGroup("Min","access the joint limits min");
    if(xtmp.size() != njoints+1)
        {
            ACE_OS::printf("Not enough min joint limits\n");
            return false;
        }
    for(int i =1;i<xtmp.size();i++)
        limitsMin[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];

	xtmp = p.findGroup("LIMITS").findGroup("error_tol","error tolerance during tracking");
    if(xtmp.size() != njoints+1)
        {
            ACE_OS::printf("Not enough error_tol\n");
            return false;
        }
    for(int i=1;i<xtmp.size();i++)
        error_tol[i-1] = xtmp.get(i).asDouble()*angleToEncoder[i-1];


	for(int i =0;i<njoints;i++)
    {
        current_pos[i] = 0.0;
     	ErrorPos[i] = 0.0;
		double v = 0.0;
		if (v<limitsMin[i]) v = limitsMin[i];
		if (v>limitsMax[i]) v = limitsMax[i];
		//else v = next_pos[i];
        next_pos[i] = M_PI*(v/angleToEncoder[i])/180;//removed (v/angleToEncoder[i]-1)/180
        next_vel[i] = 0.0;
		input = 0;
        inputs[i] = 0;
        vels[i] = 1;
   }
	

	ImplementPositionControl<iCubSimulationControl, IPositionControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementVelocityControl<iCubSimulationControl, IVelocityControl>::
		initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementPidControl<iCubSimulationControl, IPidControl>::
		initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementEncoders<iCubSimulationControl, IEncoders>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
    ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
	ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
	ImplementControlLimits<iCubSimulationControl, IControlLimits>::
        initialize(njoints, axisMap, angleToEncoder, zeros);
		
    velocityMode = false;

	Thread::start();
	//_done.wait ();
	_mutex.post();
	_opened = true;
    return true;
}

bool iCubSimulationControl::close (void)
{
    if (_opened) {

        if (Thread::isRunning())
            {
            }
        
        Thread::stop();	/// stops the thread first (joins too).
        
        ImplementPositionControl<iCubSimulationControl, IPositionControl>::uninitialize ();
        ImplementVelocityControl<iCubSimulationControl, IVelocityControl>::uninitialize();
        ImplementPidControl<iCubSimulationControl, IPidControl>::uninitialize();
        ImplementEncoders<iCubSimulationControl, IEncoders>::uninitialize();
        ImplementControlCalibration<iCubSimulationControl, IControlCalibration>::uninitialize();
        ImplementAmplifierControl<iCubSimulationControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits<iCubSimulationControl, IControlLimits>::uninitialize(); /**/
    }

	checkAndDestroy<double>(current_pos);
	checkAndDestroy<double>(current_vel);
    checkAndDestroy<double>(next_pos);
    checkAndDestroy<double>(next_vel);
   // delete[] joint_dev;

    checkAndDestroy<double>(angleToEncoder);
    checkAndDestroy<double>(zeros);
    checkAndDestroy<double>(limitsMin);
    checkAndDestroy<double>(limitsMax);
    checkAndDestroy<int>(axisMap);
    checkAndDestroy<int>(inputs);
    checkAndDestroy<double>(vels);
    checkAndDestroy<double>(torqueLimits);
    
    checkAndDestroy<double>(refSpeed);
    checkAndDestroy<double>(refAccel);
    checkAndDestroy<double>(controlP);   

    checkAndDestroy<double>(error_tol);
    checkAndDestroy<bool>(motor_on);
    
  //  delete[] jointNames;

	_opened = false;
	return true;
}

void iCubSimulationControl::run() {
	while(!isStopping()) {
        _mutex.wait();
		int lengths[] = { 0, 16, 16, 6, 6 , 6, 3};
        if (partSelec<=6) {
            int len = lengths[partSelec];
            int i;
            
			for (i=0; i<len; i++) {
                SimControl& ctrl = manager.control(partSelec,i); 
                current_pos[i] = ctrl.getAngle();
                current_vel[i] = ctrl.getVelocity();
            }

			for (i=0; i<len; i++) {
                SimControl& ctrl = manager.control(partSelec,i); 
                motor_on[i] = true; // no reason to turn motors off, for now
                if (velocityMode) {
                    //ctrl.setVelocity(next_vel[i]);
					if(((current_pos[i]<limitsMin[i])&&(next_vel[i]<0)) || ((current_pos[i]>limitsMax[i])&&(next_vel[i]>0)))
                         ctrl.setVelocity(0.0);
                     else{
                         ctrl.setVelocity(next_vel[i]);
					}
						
                } else {
                    // no acceleration control right now, just velocity
                    ctrl.setControlParameters(vels[i],1);
                    ctrl.setPosition(next_pos[i]);
                }
            }
        }
        _mutex.post();
        Time::delay(0.01);
    }    
}


bool iCubSimulationControl::getAxes(int *ax)
{
    *ax = njoints;
    return true;
}

bool iCubSimulationControl::setPidRaw (int axis, const Pid &pid)
{
   return NOT_YET_IMPLEMENTED("setPidRaw");
}
bool iCubSimulationControl::getPidRaw (int axis, Pid *out)
{
   return NOT_YET_IMPLEMENTED("getPidRaw");
}

bool iCubSimulationControl::getPidsRaw (Pid *out)
{
    return NOT_YET_IMPLEMENTED("getPidsRaw");
}

bool iCubSimulationControl::setPidsRaw(const Pid *pids)
{
    return NOT_YET_IMPLEMENTED("setPidsRaw");
}

/// cmd is a SingleAxis pointer with 1 double arg
bool iCubSimulationControl::setReferenceRaw (int j, double ref)
{
    if(j<njoints)
        {
	//		printf(" GETTING THE REFERENCE??? 1\n");
            _mutex.wait();
            next_pos[j] = ref;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("trying to set ref of an unknown joint number %d\n",j);
    return false;
}

/// cmd is an array of double (LATER: to be optimized).
bool iCubSimulationControl::setReferencesRaw (const double *refs)
{
   _mutex.wait();
   for(int i = 0;i<njoints;i++){
        next_pos[i] = refs[i];
   }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::setErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimitRaw");
}

bool iCubSimulationControl::setErrorLimitsRaw(const double *limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimitsRaw");
}

bool iCubSimulationControl::getErrorRaw(int axis, double *err)
{
    return NOT_YET_IMPLEMENTED("getErrorRaw");
}

bool iCubSimulationControl::getErrorsRaw(double *errs)
{
    return NOT_YET_IMPLEMENTED("getErrorsRaw");
}

bool iCubSimulationControl::getOutputRaw(int axis, double *out)
{
   return NOT_YET_IMPLEMENTED("getOutputRaw");
}

bool iCubSimulationControl::getOutputsRaw(double *outs)
{
    return NOT_YET_IMPLEMENTED("getOutputsRaw");
}

bool iCubSimulationControl::getReferenceRaw(int j, double *ref)
{
    if(j<njoints)
        {
            _mutex.wait();
            *ref = next_pos[j];
            _mutex.post();
            return true;
        }
    ACE_OS::printf("getReferenceRaw axis number too high %d\n",j);
    return false;
}

bool iCubSimulationControl::getReferencesRaw(double *ref)
{
    _mutex.wait();
    for(int i = 0;i<njoints;i++)
        ref[i] = next_pos[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getErrorLimitRaw(int j, double *err)
{
     return NOT_YET_IMPLEMENTED("getErrorLimitRaw");
}

bool iCubSimulationControl::getErrorLimitsRaw(double *errs)
{
   return NOT_YET_IMPLEMENTED("getErrorLimitsRaw");
}

bool iCubSimulationControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetPidRaw");
}

bool iCubSimulationControl::enablePidRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("enablePidRaw");
}

bool iCubSimulationControl::setOffsetRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setOffsetRaw");
}

bool iCubSimulationControl::disablePidRaw(int axis)
{
	return NOT_YET_IMPLEMENTED("disablePidRaw");
}

bool iCubSimulationControl::setPositionMode()
{
    velocityMode = false;
    return true;
}

bool iCubSimulationControl::setVelocityMode()
{
    velocityMode = true;
    return true;
}

bool iCubSimulationControl::positionMoveRaw(int axis, double ref)
{
    velocityMode = false;
	if(axis<njoints)
        {
            _mutex.wait();
			if(ref< limitsMin[axis])
                next_pos[axis] = limitsMin[axis];
			else if(ref > limitsMax[axis])
                next_pos[axis] = limitsMax[axis];
			else
                next_pos[axis] = ref;
            motor_on[axis]=true;
				
            ACE_OS::printf("changed position %d to %f\n",axis,next_pos[axis]);
            _mutex.post();
            return true;
        }
    ACE_OS::printf("positionMoveRaw joint access too high %d \n",axis);
    return false;    
}

bool iCubSimulationControl::positionMoveRaw(const double *refs)
{
    velocityMode = false;
      _mutex.wait();
 	for(int i = 0; i<njoints; i++)
        {
            double ref = refs[i];
			if(ref< limitsMin[i]){
                next_pos[i] = limitsMin[i];
			}
			else if(ref > limitsMax[i]){
                next_pos[i] = limitsMax[i];
			}
            else
			 next_pos[i] = ref;
            motor_on[i]=true;
            ACE_OS::printf("moving joint %d to pos %f\n",i,next_pos[i]);
        }
    _mutex.post();
    return true;
}

bool iCubSimulationControl::relativeMoveRaw(int j, double delta)
{
    velocityMode = false;
	return positionMoveRaw(j,next_pos[j]+delta);
}

bool iCubSimulationControl::relativeMoveRaw(const double *deltas)
{
    velocityMode = false;
 	for(int i = 0; i<njoints; i++) {
        relativeMoveRaw(i,deltas[i]);
    }
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw (bool *ret)
{
    _mutex.wait();
    bool fin = true;
    for(int i = 0;i<njoints;i++)
        {
            if(!(fabs(current_pos[i]-next_pos[i])<error_tol[i]))
                {
                    fin = false;
                    // ACE_OS::printf("axes %d unfinished\n");
                }
        }
    //if(fin)
    printf("motion finished error tol %f %f %f\n",error_tol[0],current_pos[0],next_pos[0]);
    *ret = fin;
    _mutex.post();
    return true;
}

bool iCubSimulationControl::checkMotionDoneRaw(int axis, bool *ret)
{
    if(axis<njoints)
        {
            _mutex.wait();
            if(fabs(current_pos[axis]-next_pos[axis])<error_tol[axis])
                *ret = true;
            else
                *ret = false;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("checkMotionDoneRaw axis too high %d\n",axis);
    return false;
}
bool iCubSimulationControl::setRefSpeedRaw(int j, double sp)
{
	if(j<njoints)
        {
            _mutex.wait();
			 vel = sp *180/M_PI ;
             vels[j] = vel/10;
			 printf("the velocity: %lf %lf\n",vel, sp );
            _mutex.post();
            return true;
        }
    return false;
}

bool iCubSimulationControl::setRefSpeedsRaw(const double *spds)
{
	_mutex.wait();
	for(int i = 0; i<njoints; i++)
        {
             vel = spds[i];
			 
	}
	_mutex.post();
    return true;
}
bool iCubSimulationControl::setRefAccelerationRaw(int j, double acc)
{
	return NOT_YET_IMPLEMENTED("setRefAccelerationRaw");
}
bool iCubSimulationControl::setRefAccelerationsRaw(const double *accs)
{
	return NOT_YET_IMPLEMENTED("setRefAccelerationsRaw");
}
bool iCubSimulationControl::getRefSpeedRaw(int j, double *ref)
{
	return NOT_YET_IMPLEMENTED("getRefSpeedRaw");
}
bool iCubSimulationControl::getRefSpeedsRaw(double *spds)
{
	return NOT_YET_IMPLEMENTED("getRefSpeedsRaw");
}
bool iCubSimulationControl::getRefAccelerationRaw(int j, double *acc)
{
	return NOT_YET_IMPLEMENTED("getRefAccelerationRaw");
}
bool iCubSimulationControl::getRefAccelerationsRaw(double *accs)
{
	return NOT_YET_IMPLEMENTED("getRefAccelerationsRaw");
}
bool iCubSimulationControl::stopRaw(int j)
{
	if(j<njoints)
        {
            _mutex.wait();
            next_pos[j] = current_pos[j];
            next_vel[j] = 0;
            _mutex.post();
            return true;
        }
    ACE_OS::printf("stopRaw joint num too high %d \n",j);
    return false;
}
bool iCubSimulationControl::stopRaw()
{
	_mutex.wait();
	for(int i=0;i<njoints;i++){
        next_pos[i] = current_pos[i];
        next_vel[i] = 0;
	}
    _mutex.post();
    return true;
}
// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (int axis, double sp)
{
    velocityMode = true;
	if(axis<njoints) {
        _mutex.wait();
        next_vel[axis] = sp;
        motor_on[axis] = true;
        _mutex.post();
        return true;
    }
    return false;    
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool iCubSimulationControl::velocityMoveRaw (const double *sp)
{
    velocityMode = true;
    for (int i=0; i<njoints; i++) {
        velocityMoveRaw(i,sp[i]);
    }
    return true;
}

bool iCubSimulationControl::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoderRaw");
}

bool iCubSimulationControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncodersRaw");
}

bool iCubSimulationControl::resetEncoderRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetEncoderRaw");
}

bool iCubSimulationControl::resetEncodersRaw()
{
   return NOT_YET_IMPLEMENTED("resetEncodersRaw");
}

bool iCubSimulationControl::getEncodersRaw(double *v)
{
   _mutex.wait();
    for(int i = 0;i<njoints;i++)
        v[i] = current_pos[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderRaw(int axis, double *v)
{
	_mutex.wait();
    *v = current_pos[axis];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderSpeedsRaw(double *v)
{
   _mutex.wait();
    for(int i = 0;i<njoints;i++)
        v[i] = current_vel[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderSpeedRaw(int j, double *v)
{
	_mutex.wait();
    *v = current_vel[j];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::getEncoderAccelerationsRaw(double *v)
{
  	return NOT_YET_IMPLEMENTED("getEncoderAccelerationsRaw");
}

bool iCubSimulationControl::getEncoderAccelerationRaw(int j, double *v)
{
    return NOT_YET_IMPLEMENTED("getEncoderAcc");
}

bool iCubSimulationControl::disableAmpRaw(int axis)
{
    if(axis<njoints)
        {
            _mutex.wait();
            motor_on[axis] = false;
            _mutex.post();
            return true;            
        }
    ACE_OS::printf("disableAmpRaw axis num too high %d\n",axis);
    return false;  
}

bool iCubSimulationControl::enableAmpRaw(int axis)
{
   
	if(axis<njoints)
        {
            _mutex.wait();
            motor_on[axis] = true;
            _mutex.post();
            return true;            
        }
    ACE_OS::printf("enableAmpRaw axis num too high %d\n",axis);
    return false;
}

// bcast
bool iCubSimulationControl::getCurrentsRaw(double *cs)
{
    return NOT_YET_IMPLEMENTED("getCurrentsRaw");
}

// bcast currents
bool iCubSimulationControl::getCurrentRaw(int axis, double *c)
{
    return NOT_YET_IMPLEMENTED("getCurrentRaw");
}

bool iCubSimulationControl::setMaxCurrentRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setMaxCurrentRaw");
}

bool iCubSimulationControl::calibrateRaw(int axis, double p)
{
    return NOT_YET_IMPLEMENTED("calibrateRaw");
}

bool iCubSimulationControl::doneRaw(int axis)
{
    return NOT_YET_IMPLEMENTED("doneRaw");
}


bool iCubSimulationControl::getAmpStatusRaw(int *st)
{
	_mutex.wait();
    for(int i =0;i<njoints;i++)
        st[i] = (int)motor_on[i];
    _mutex.post();
    return true;
}

bool iCubSimulationControl::setLimitsRaw(int axis, double min, double max)
{
    if(axis > -1 && axis < njoints){
        _mutex.wait();
        limitsMax[axis] = max;
        limitsMin[axis] = min;
        _mutex.post();
       return true;
    }
    return false;
}

bool iCubSimulationControl::getLimitsRaw(int axis, double *min, double *max)
{
	if(axis > -1 && axis < njoints){
         *min = limitsMin[axis];
         *max = limitsMax[axis];
         return true;
     }
     else
         return false;
}

