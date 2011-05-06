/**
@ingroup icub_module

\defgroup gravityCompensator gravityCompensator
 
Estimates the gravitational contribute to motors based on estimation of the robot dynamics
 
Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 24/07/2010 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module estimates the gravitational term acting on joints due to links weight.
The estimation is perfomed relying on rigid body dynamics using CAD 
parameters. 
For further information about the use of this module and of the iCub force control interface, please refer to the force control page:
http://eris.liralab.it/wiki/Force_Control

\section lib_sec Libraries 
- YARP libraries. 
- iDyn library.  

\section parameters_sec Parameters

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
  specified \e 20ms is assumed. The minimum suggested rate is \e 20ms.

--no_legs
- This option disables the gravity compensation for the legs joints.

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e /wholebody_gComp/inertial:i 
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
gravityCompensator --rate 50  
\endcode 
 
the module add offset values which are assigned to the IImpedanceControl interface and ITorqueControl interface
  
\author Matteo Fumagalli

This file can be edited at \in src/gravityCompensator/main.cpp.
*/ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>
#include <iomanip>
#include <string.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;


#define MAX_JN 12
#define MAX_FILTER_ORDER 6
enum{GRAVITY_COMPENSATION_OFF = 0, GRAVITY_COMPENSATION_ON = 1};
enum{TORQUE_INTERFACE = 0, IMPEDANCE_POSITION = 1, IMPEDANCE_VELOCITY = 2};
int gravity_mode = GRAVITY_COMPENSATION_ON;

class gravityCompensatorThread: public RateThread
{
private:

	BufferedPort<Vector> *port_inertial;
	BufferedPort<Vector> *additional_offset;
	BufferedPort<Vector> *left_arm_torques;
	BufferedPort<Vector> *right_arm_torques;
	BufferedPort<Vector> *left_leg_torques;
	BufferedPort<Vector> *right_leg_torques;

	PolyDriver *ddLA;
    PolyDriver *ddRA;
    PolyDriver *ddH;
    IEncoders  *iencs_arm_left;
    IEncoders  *iencs_arm_right;
    IEncoders  *iencs_head;
	IControlMode *iCtrlMode_arm_left;
	IControlMode *iCtrlMode_arm_right;
	IImpedanceControl *iImp_arm_left;
	ITorqueControl *iTqs_arm_left;
	IImpedanceControl *iImp_arm_right;
	ITorqueControl *iTqs_arm_right;
	
	
    PolyDriver *ddLL;
    PolyDriver *ddRL;
    PolyDriver *ddT;
    IEncoders  *iencs_leg_left;
    IEncoders  *iencs_leg_right;
    IEncoders  *iencs_torso;
	IControlMode *iCtrlMode_leg_left;
	IControlMode *iCtrlMode_leg_right;
	IImpedanceControl *iImp_leg_left;
	ITorqueControl *iTqs_leg_left;
	IImpedanceControl *iImp_leg_right;
	ITorqueControl *iTqs_leg_right;

	Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;

    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;

    Vector *inertial;
    Vector *offset_input;

    AWLinEstimator  *linEstUp;
    AWQuadEstimator *quadEstUp;
    AWLinEstimator  *linEstLow;
    AWQuadEstimator *quadEstLow;

    int ctrlJnt;
	int allJnt;

	iCubWholeBody icub;

	Vector q_head, dq_head, d2q_head;
	Vector q_larm, dq_larm, d2q_larm;
	Vector q_rarm, dq_rarm, d2q_rarm;
	Vector all_q_up, all_dq_up, all_d2q_up;

	Vector q_torso, dq_torso, d2q_torso;
	Vector q_lleg, dq_lleg, d2q_lleg;
	Vector q_rleg, dq_rleg, d2q_rleg;
	Vector all_q_low, all_dq_low, all_d2q_low;

	Vector w0,dw0,d2p0,Fend,Muend;
	Vector F_LArm, F_RArm, F_iDyn_LArm, F_iDyn_RArm, Offset_LArm, Offset_RArm;
    Vector F_LLeg, F_RLeg, F_iDyn_LLeg, F_iDyn_RLeg, Offset_LLeg, Offset_RLeg;
	Matrix F_ext_up, F_ext_low;
	Vector inertial_measurements;
	Vector torque_offset;

	Vector torques_LA,torques_RA,torques_LL,torques_RL;
	Vector ampli_larm, ampli_rarm, ampli_lleg, ampli_rleg;
	bool isCalibrated;
	
    Vector evalVelUp(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return linEstUp->estimate(el);
    }
	Vector evalVelLow(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return linEstLow->estimate(el);
    }

    Vector evalAccUp(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return quadEstUp->estimate(el);
    }
    Vector evalAccLow(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return quadEstLow->estimate(el);
    }

	void init_upper()
	{
		//---------------------PARTS-------------------------//
		// Left_arm variables
		allJnt = 0;
        int jnt=0;
        iencs_arm_left->getAxes(&jnt);
        encoders_arm_left.resize(jnt,0.0);
		F_LArm.resize(6,0.0);
        F_iDyn_LArm.resize(6,0.0);
        Offset_LArm.resize(6,0.0);
        q_larm.resize(7,0.0);
		dq_larm.resize(7,0.0);
		d2q_larm.resize(7,0.0);
		allJnt+=jnt;

		// Right_arm variables
		jnt = 0;
        iencs_arm_right->getAxes(&jnt);
        encoders_arm_right.resize(jnt,0.0);
		F_RArm.resize(6,0.0);
        F_iDyn_RArm.resize(6,0.0);
        Offset_RArm.resize(6,0.0);
		q_rarm.resize(7,0.0);
		dq_rarm.resize(7,0.0);
		d2q_rarm.resize(7,0.0);
        allJnt+=jnt;

		// Head variables
		jnt = 0;
        iencs_head->getAxes(&jnt);
        encoders_head.resize(jnt,0.0);
		q_head.resize(3,0.0);
		dq_head.resize(3,0.0);
		d2q_head.resize(3,0.0);
		allJnt+=jnt;

		all_q_up.resize(allJnt,0.0);
		all_dq_up.resize(allJnt,0.0);
		all_d2q_up.resize(allJnt,0.0); 
		torques_LA.resize(7);torques_RA.resize(7);
		ampli_larm.resize(7);ampli_larm=1.0;//ampli_larm[0]=1.1;ampli_larm[1]=1.1;ampli_larm[3]=0.8;
		ampli_rarm.resize(7);ampli_rarm=1.0;//ampli_rarm[0]=1.1;ampli_rarm[1]=1.1;ampli_rarm[3]=0.8;
	}	
	void init_lower()
	{
		//---------------------PARTS-------------------------//
		// Left_leg variables
		allJnt = 0;
        int jnt=0;
        if (iencs_leg_left) iencs_leg_left->getAxes(&jnt);
		else jnt = 6; //default value
        encoders_leg_left.resize(jnt,0.0);
        q_lleg.resize(6,0.0);
		dq_lleg.resize(6,0.0);
		d2q_lleg.resize(6,0.0);
		allJnt+=jnt;

		// Right_leg variables
		jnt = 0;
        if (iencs_leg_right) iencs_leg_right->getAxes(&jnt);
		else jnt = 6; //default value
        encoders_leg_right.resize(jnt,0.0);
		q_rleg.resize(6,0.0);
		dq_rleg.resize(6,0.0);
		d2q_rleg.resize(6,0.0);
        allJnt+=jnt;

		// Head variables
		jnt = 0;
        iencs_torso->getAxes(&jnt);
        encoders_torso.resize(jnt,0.0);
		q_torso.resize(3,0.0);
		dq_torso.resize(3,0.0);
		d2q_torso.resize(3,0.0);
		allJnt+=jnt;

		all_q_low.resize(allJnt,0.0);
		all_dq_low.resize(allJnt,0.0);
		all_d2q_low.resize(allJnt,0.0);
		torques_LL.resize(6);torques_RL.resize(6);
		ampli_lleg.resize(6);ampli_lleg=1.0;//ampli_lleg[0]=1.0;ampli_lleg[1]=1.1;ampli_lleg[2]=1.1;
		ampli_rleg.resize(6);ampli_rleg=1.0;//ampli_rleg[0]=1.0;ampli_rleg[1]=1.1;ampli_rleg[2]=1.1;
	
	}

	int limbJnt;
	string side;
	string part;

	double converter;
	double ampli;
	Vector G;

    
	void setLowerMeasure()
	{
		icub.lowerTorso->setAng("torso",CTRL_DEG2RAD * q_torso);
		icub.lowerTorso->setDAng("torso",CTRL_DEG2RAD * dq_torso);
		icub.lowerTorso->setD2Ang("torso",CTRL_DEG2RAD * d2q_torso);

		icub.lowerTorso->setAng("left_leg",CTRL_DEG2RAD * q_lleg);
		icub.lowerTorso->setDAng("left_leg",CTRL_DEG2RAD * dq_lleg);
		icub.lowerTorso->setD2Ang("left_leg",CTRL_DEG2RAD * d2q_lleg);

		icub.lowerTorso->setAng("right_leg",CTRL_DEG2RAD * q_rleg);
		icub.lowerTorso->setDAng("right_leg",CTRL_DEG2RAD * dq_rleg);
		icub.lowerTorso->setD2Ang("right_leg",CTRL_DEG2RAD * d2q_rleg);
	}

	void setUpperMeasure()
	{
		icub.upperTorso->setAng("head",CTRL_DEG2RAD * q_head);
		icub.upperTorso->setAng("left_arm",CTRL_DEG2RAD * q_larm);
		icub.upperTorso->setAng("right_arm",CTRL_DEG2RAD * q_rarm);
		icub.upperTorso->setDAng("head",CTRL_DEG2RAD * dq_head);
		icub.upperTorso->setDAng("left_arm",CTRL_DEG2RAD * dq_larm);
		icub.upperTorso->setDAng("right_arm",CTRL_DEG2RAD * dq_rarm);
		icub.upperTorso->setD2Ang("head",CTRL_DEG2RAD * d2q_head);
		icub.upperTorso->setD2Ang("left_arm",CTRL_DEG2RAD * d2q_larm);
		icub.upperTorso->setD2Ang("right_arm",CTRL_DEG2RAD * d2q_rarm);
		icub.upperTorso->setInertialMeasure(w0,dw0,d2p0);
	}
	

public:
    gravityCompensatorThread(int _rate, PolyDriver *_ddLA, PolyDriver *_ddRA, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddRL, PolyDriver *_ddT) : RateThread(_rate), ddLA(_ddLA), ddRA(_ddRA), ddLL(_ddLL), ddRL(_ddRL), ddH(_ddH), ddT(_ddT)
    {   

		//--------------INTERFACE INITIALIZATION-------------//
		iencs_arm_left      = 0;
        iencs_arm_right		= 0;
        iencs_head			= 0;
	    iCtrlMode_arm_left  = 0;
	    iCtrlMode_arm_right = 0;
		iImp_arm_left		= 0;
	    iTqs_arm_left		= 0;
	    iImp_arm_right		= 0;
		iTqs_arm_right		= 0;
        iencs_leg_left		= 0;
		iencs_leg_right		= 0;
		iencs_torso			= 0;
		iCtrlMode_leg_left	= 0;
		iCtrlMode_leg_right = 0;
		iImp_leg_left		= 0;
		iTqs_leg_left		= 0;
		iImp_leg_right		= 0;
		iTqs_leg_right		= 0;
		isCalibrated = false;

	    //---------------------PORTS-------------------------//
		port_inertial=new BufferedPort<Vector>;
		port_inertial->open("/gravityCompensator/inertial:i");

		
		additional_offset=new BufferedPort<Vector>;
		additional_offset->open("/gravityCompensator/ctrlOffset:i");

		
		left_arm_torques = new BufferedPort<Vector>;
		left_arm_torques->open("/gravityCompensator/left_arm_torques:o");
		right_arm_torques = new BufferedPort<Vector>;
		right_arm_torques->open("/gravityCompensator/right_arm_torques:o");
		left_leg_torques = new BufferedPort<Vector>;
		left_leg_torques->open("/gravityCompensator/left_leg_torques:o");
		right_leg_torques = new BufferedPort<Vector>;
		right_leg_torques->open("/gravityCompensator/right_leg_torques:o");
		//*offset_input = 0.0;

		//---------------------DEVICES--------------------------//
		if (ddLA) ddLA->view(iencs_arm_left);
        if (ddRA) ddRA->view(iencs_arm_right);
		if (ddH)  ddH->view(iencs_head);
		if (ddLL) ddLL->view(iencs_leg_left);
        if (ddRL) ddRL->view(iencs_leg_right);
		if (ddT)  ddT->view(iencs_torso);
		
		if (ddLA) ddLA->view(iCtrlMode_arm_left);
        if (ddRA) ddRA->view(iCtrlMode_arm_right);
		if (ddLA) ddLA->view(iImp_arm_left);
		if (ddLA) ddLA->view(iTqs_arm_left);
        if (ddRA) ddRA->view(iImp_arm_right);
		if (ddRA) ddRA->view(iTqs_arm_right);

		if (ddLL) ddLL->view(iCtrlMode_leg_left);
        if (ddRL) ddRL->view(iCtrlMode_leg_right);
		if (ddLL) ddLL->view(iImp_leg_left);
		if (ddLL) ddLL->view(iTqs_leg_left);
		if (ddRL) ddRL->view(iImp_leg_right);
		if (ddRL) ddRL->view(iTqs_leg_right);	
		
        linEstUp =new AWLinEstimator(16,1.0);
        quadEstUp=new AWQuadEstimator(25,1.0);
	    linEstLow =new AWLinEstimator(16,1.0);
        quadEstLow=new AWQuadEstimator(25,1.0);
		
		converter = 0.0; 
		ampli = 0.0;
        //-----------parts INIT VARIABLES----------------//
		init_upper();
		init_lower();
		//-----------CARTESIAN INIT VARIABLES----------------//
		
		ctrlJnt = 4;
		w0.resize(3,0.0);
		dw0.resize(3,0.0);
		d2p0.resize(3,0.0);
		Fend.resize(3,0.0);
		Muend.resize(3,0.0);
		F_ext_up.resize(6,3);
		F_ext_up = 0.0;
		F_ext_low.resize(6,3);
		F_ext_low = 0.0;
		inertial_measurements.resize(12);
		inertial_measurements.zero();
		torque_offset.resize(ctrlJnt);

		int ctrl_mode = 0;
		
		switch(gravity_mode)
		{
			case GRAVITY_COMPENSATION_OFF:		printf("GRAVITY_COMPENSATION_OFF     \n");	break;
			case GRAVITY_COMPENSATION_ON:		printf("GRAVITY_COMPENSATION_ON      \n");	break;
			default:
			case VOCAB_CM_UNKNOWN:		printf("UNKNOWN  \n");	break;
		}
		
	}
	void setZeroJntAngVelAcc()
	{
		dq_head = 0.0;
		d2q_head = 0.0;
		dq_larm = 0.0;
		d2q_larm = 0.0;
		dq_rarm = 0.0;
		d2q_rarm = 0.0;
		
		dq_rleg=0.0;
		d2q_rleg=0.0;
		dq_lleg=0.0;
		d2q_lleg=0.0;
		dq_torso=0.0;
		d2q_torso=0.0;
	}
	void readAndUpdate(bool waitMeasure=false)
	{
		inertial = port_inertial->read(waitMeasure);
		offset_input = additional_offset->read(false);
		int sz = 0;
		if(offset_input!=0)
		{
			sz = offset_input->length();
			Vector o=*offset_input;
			torque_offset = 0.0;
			for(int i=0;i<ctrlJnt;i++)
				torque_offset[i] = o[i];
			if(sz>ctrlJnt)
				fprintf(stderr,"warning...controlled joint < of offsets size!!!");
		}
		

		sz = 0;
		if(inertial!=0)
		{
			sz = inertial->length();
			inertial_measurements.resize(sz) ;
			inertial_measurements= *inertial;
			d2p0[0] = inertial_measurements[0];
			d2p0[1] = inertial_measurements[1];
			d2p0[2] = inertial_measurements[2];
			w0 [0] = 0;
			w0 [1] = 0;
			w0 [2] = 0;
			dw0 [0] = 0;
			dw0 [1] = 0;
			dw0 [2] = 0;
		}
		
		getUpperEncodersSpeedAndAcceleration();
		setUpperMeasure();
		getLowerEncodersSpeedAndAcceleration();
		setLowerMeasure();

	}

	void getLowerEncodersSpeedAndAcceleration()
		{
			
			if (iencs_leg_left)  iencs_leg_left->getEncoders(encoders_leg_left.data());
			else encoders_leg_left.zero();

			if (iencs_leg_right) iencs_leg_right->getEncoders(encoders_leg_right.data());
			else encoders_leg_right.zero();

			iencs_torso->getEncoders(encoders_torso.data());

			for (int i=0;i<q_torso.length();i++)
			{
				q_torso(i) = encoders_torso(2-i);
				all_q_low(i) = q_torso(i);
			}
			for (int i=0;i<q_lleg.length();i++)
			{
				q_lleg(i) = encoders_leg_left(i);
				all_q_low(q_torso.length()+i) = q_lleg(i);
			}
			for (int i=0;i<q_rleg.length();i++)
			{
				q_rleg(i) = encoders_leg_right(i);
				all_q_low(q_torso.length()+q_lleg.length()+i) = q_rleg(i);
			}
            setZeroJntAngVelAcc();
			//all_dq_low = evalVelLow(all_q_low);
			//all_d2q_low = evalAccLow(all_q_low);
			/*for (int i=0;i<q_torso.length();i++)
			{
				dq_torso(i) = all_dq_low(i);
				d2q_torso(i) = all_d2q_low(i);
			}
			for (int i=0;i<q_lleg.length();i++)
			{
				dq_lleg(i) = all_dq_low(i+q_torso.length());
				d2q_lleg(i) = all_d2q_low(i+q_torso.length());
			}
			for (int i=0;i<q_rleg.length();i++)
			{
				dq_rleg(i) = all_dq_low(i+q_torso.length()+q_lleg.length());
				d2q_rleg(i) = all_d2q_low(i+q_torso.length()+q_lleg.length());
			}*/
		}


	void getUpperEncodersSpeedAndAcceleration()
	{
		iencs_arm_left->getEncoders(encoders_arm_left.data());
		iencs_arm_right->getEncoders(encoders_arm_right.data());
		iencs_head->getEncoders(encoders_head.data());
		

		for (int i=0;i<q_head.length();i++)
		{
			q_head(i) = encoders_head(i);
			all_q_up(i) = q_head(i);
		}
		for (int i=0;i<q_larm.length();i++)
		{
			q_larm(i) = encoders_arm_left(i);
			all_q_up(q_head.length()+i) = q_larm(i);
		}
		for (int i=0;i<q_rarm.length();i++)
		{
			q_rarm(i) = encoders_arm_right(i);
			all_q_up(q_head.length()+q_larm.length()+i) = q_rarm(i);
		}

        setZeroJntAngVelAcc();
/*
		all_dq_up = evalVelUp(all_q_up);
		all_d2q_up = evalAccUp(all_q_up);
		
		for (int i=0;i<q_head.length();i++)
		{
			dq_head(i) = all_dq_up(i);
			d2q_head(i) = all_d2q_up(i);
		}
		for (int i=0;i<q_larm.length();i++)
		{
			dq_larm(i) = all_dq_up(i+q_head.length());
			d2q_larm(i) = all_d2q_up(i+q_head.length());
		}
		for (int i=0;i<q_rarm.length();i++)
		{
			dq_rarm(i) = all_dq_up(i+q_head.length()+q_larm.length());
			d2q_rarm(i) = all_d2q_up(i+q_head.length()+q_larm.length());
		}*/
	}


	void evalTorques()
	{
		torques_LA = icub.upperTorso->left->getTorques();
		torques_RA = icub.upperTorso->right->getTorques();
		torques_LL = icub.lowerTorso->left->getTorques();
		torques_RL = icub.lowerTorso->right->getTorques();   
	}

    bool threadInit()
    {       
		return true;
    }
	void feedFwdGravityControl(IControlMode *iCtrlMode, ITorqueControl *iTqs, IImpedanceControl *iImp,const Vector &G, const Vector &ampli, bool releasing=false)
	{
		int ctrl_mode = 0;
		for(int i=0;i<ctrlJnt;i++)
		{
			iCtrlMode->getControlMode(i,&ctrl_mode);
			switch(ctrl_mode)
			{
				case VOCAB_CM_TORQUE:	
					if(gravity_mode == GRAVITY_COMPENSATION_ON)
					{
						iTqs->setRefTorque(i,ampli[i]*G[i]+torque_offset[i]);
					}
					else
					{
						iTqs->setRefTorque(i,torque_offset[i]);
					}
					break;
				case VOCAB_CM_IMPEDANCE_POS:
				case VOCAB_CM_IMPEDANCE_VEL:
					if(gravity_mode == GRAVITY_COMPENSATION_ON)
					{
						//fprintf(stderr,"compensating gravity\n");
						double off = ampli[i]*G[i]+torque_offset[i];
						iImp->setImpedanceOffset(i,off);
						//iImp->getImpedance(i,&k,&d,&o);
						//iImp->setImpedance(i,k,d,ampli[i]*G[i]+torque_offset[i]);
					}
					else
					{
						//iImp->getImpedance(i,&k,&d,&o);
						iImp->setImpedanceOffset(i,torque_offset[i]);
					}
					break;
				default:
					break;
			}
			if(releasing)
			{
                fprintf(stderr,"releasing... \n");
				iImp->setImpedanceOffset(i,0.0);
				iTqs->setRefTorque(i,0.0);
			}
		}
	}
    void run()
    {  
		if(isCalibrated==true)
		{
		
			setUpperMeasure();
			setLowerMeasure();

			readAndUpdate(true);

			Vector F_up(6);
			F_up=0.0;
			icub.upperTorso->setInertialMeasure(w0,dw0,d2p0);
			Matrix F_sens_up = icub.upperTorso->estimateSensorsWrench(F_ext_up,false);
			icub.lowerTorso->setInertialMeasure(icub.upperTorso->getTorsoAngVel(),
												icub.upperTorso->getTorsoAngAcc(),
												icub.upperTorso->getTorsoLinAcc());
			Matrix F_sens_low = icub.lowerTorso->estimateSensorsWrench(F_ext_low,false);
			evalTorques();
			
			if (iCtrlMode_arm_left)  
			{
				feedFwdGravityControl(iCtrlMode_arm_left,iTqs_arm_left,iImp_arm_left,torques_LA,ampli_larm);
				left_arm_torques->prepare()  =  torques_LA;
				left_arm_torques->write();
			}
			if (iCtrlMode_arm_right)
			{
				feedFwdGravityControl(iCtrlMode_arm_right,iTqs_arm_right,iImp_arm_right,torques_RA,ampli_rarm);
				right_arm_torques->prepare() =  torques_RA;
				right_arm_torques->write();
			}
			if (iCtrlMode_leg_left)	
			{
				feedFwdGravityControl(iCtrlMode_leg_left,iTqs_leg_left,iImp_leg_left,torques_LL,ampli_lleg);
				right_leg_torques->prepare() =  torques_RL;
				right_leg_torques->write();
			}
			if (iCtrlMode_leg_right)
			{
				feedFwdGravityControl(iCtrlMode_leg_right,iTqs_leg_right,iImp_leg_right,torques_RL,ampli_rleg);
				left_leg_torques->prepare()  =  torques_LL;
				left_leg_torques->write();
			}
		}
		else
		{
			if(Network::exists("/filtered/inertial:o"))
			{
				fprintf(stderr,"connection exists! starting calibration...\n");
				//the following delay is required because even if the filtered port exists, may be the 
				//low pass filtered values have not reached yet the correct value. 
				Time::delay(1.0); 

				isCalibrated = true;
				Network::connect("/filtered/inertial:o","/gravityCompensator/inertial:i");
				setZeroJntAngVelAcc();
				setUpperMeasure();
				setLowerMeasure();

				readAndUpdate(true);

				Vector F_up(6);
				F_up=0.0;
				icub.upperTorso->setInertialMeasure(w0,dw0,d2p0);
				Matrix F_sens_up = icub.upperTorso->estimateSensorsWrench(F_ext_up,false);
				icub.lowerTorso->setInertialMeasure(icub.upperTorso->getTorsoAngVel(),
													icub.upperTorso->getTorsoAngAcc(),
													icub.upperTorso->getTorsoLinAcc());
				Matrix F_sens_low = icub.lowerTorso->estimateSensorsWrench(F_ext_low,false);
				evalTorques();
				Vector LATorques = icub.upperTorso->getTorques("left_arm");
				printf("encoders: %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf\n", encoders_arm_left(0), encoders_arm_left(1), encoders_arm_left(2), encoders_arm_left(3), encoders_arm_left(4), encoders_arm_left(5), encoders_arm_left(6)); 
				printf("torques: %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", LATorques(0), LATorques(1), LATorques(2), LATorques(3), LATorques(4), LATorques(5), LATorques(6)); 
				printf("inertial: %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf, %.1lf\n", d2p0(0), d2p0(1), d2p0(2), w0(0), w0(1), w0(2), dw0(0), dw0(1), dw0(2)); 
			}
			else
			{
				fprintf(stderr,"waiting for connections from wholeBodyTorqueObserver...\n");
				Time::delay(1.0);
			}
		}
    }
    void threadRelease()
    {
		Vector Z(10);Z=0.0;
		
		feedFwdGravityControl(iCtrlMode_arm_left,iTqs_arm_left,iImp_arm_left,Z,ampli_larm,true);
		feedFwdGravityControl(iCtrlMode_arm_right,iTqs_arm_right,iImp_arm_right,Z,ampli_rarm,true);
		feedFwdGravityControl(iCtrlMode_leg_left,iTqs_leg_left,iImp_leg_left,Z,ampli_lleg,true);
		feedFwdGravityControl(iCtrlMode_leg_right,iTqs_leg_right,iImp_leg_right,Z,ampli_rleg,true);
        
		Time::delay(0.5);

        if (left_arm_torques)  {delete left_arm_torques; left_arm_torques = 0;}
        if (right_arm_torques) {delete right_arm_torques; right_arm_torques = 0;}
        if (left_leg_torques)  {delete left_leg_torques; left_leg_torques = 0;}
        if (right_leg_torques) {delete right_leg_torques; right_leg_torques = 0;}
		
		if (linEstUp)          {delete linEstUp; linEstUp = 0;}
		if (quadEstUp)         {delete quadEstUp; quadEstUp = 0;}
		if (linEstLow)         {delete linEstLow; linEstLow = 0;}
		if (quadEstLow)        {delete quadEstLow; quadEstLow = 0;}
		
		//closing ports
		port_inertial->interrupt();
		additional_offset->interrupt();
		left_arm_torques->interrupt();
	    right_arm_torques->interrupt();
        left_leg_torques->interrupt();
	    right_leg_torques->interrupt();
		port_inertial->close();
		additional_offset->close();
		left_arm_torques->close();
	    right_arm_torques->close();
        left_leg_torques->close();
	    right_leg_torques->close();

    }   
	void closePort(Contactable *_port)
	{
	}
};
// class dataCollector: class for reading from Vrow and providing for FT values on an output port

class gravityModuleCompensator: public RFModule
{
private:
	int rate;
	gravityCompensatorThread *g_comp;

	Property OptionsLeftArm;
    Property OptionsRightArm;
    Property OptionsHead;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    Property OptionsTorso;

    Port rpcPort;

    PolyDriver *dd_left_arm;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_head;
    PolyDriver *dd_left_leg;
    PolyDriver *dd_right_leg;
    PolyDriver *dd_torso;

	bool legs_enabled;
	string m_side;
	string m_part;

public:
    gravityModuleCompensator()
    {
		legs_enabled  = true;
		dd_left_arm   = 0;
		dd_right_arm  = 0;
		dd_head       = 0;
		dd_left_leg   = 0;
		dd_right_leg  = 0;
		dd_torso      = 0;
		m_side = "right";
		m_part = "arm";
    }

    virtual bool createDriver(PolyDriver *_dd)
    {
        // Creating Driver for Limb...
        if (!_dd || !(_dd->isValid()))
        {
            fprintf(stderr,"It is not possible to instantiate the device driver\nreturning...");
            return 0;
        }

        
		IEncoders         *encs     = 0;
		IControlMode      *ctrlMode = 0;
		IImpedanceControl *imp      = 0;
		ITorqueControl    *tqs      = 0;
	
        bool ok = true;
        ok = ok & _dd->view(encs);
        ok = ok & _dd->view(ctrlMode);
        ok = ok & _dd->view(imp);
        ok = ok & _dd->view(tqs);
        if (!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        string name;
        name = "gravityCompensator";	
        
        int rate;
        if (rf.check("rate"))
            rate = rf.find("rate").asInt();
        else rate = 20;

		//------------------CHECK IF LEGS ARE ENABLED-----------//
		if (rf.check("no_legs"))
		{
			legs_enabled= false;
			fprintf(stderr,"'no_legs' option found. Legs will be disabled.\n");
		}
        //---------------------DEVICES--------------------------//
        
		OptionsHead.put("device","remote_controlboard");
		OptionsHead.put("local","/gravityCompensator/head/client");
		OptionsHead.put("remote","/icub/head");

		dd_head = new PolyDriver(OptionsHead);
		if (!createDriver(dd_head))
		{
			fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");
        

        OptionsLeftArm.put("device","remote_controlboard");
        OptionsLeftArm.put("local","/gravityCompensator/left_arm/client");
        OptionsLeftArm.put("remote","/icub/left_arm");
        dd_left_arm = new PolyDriver(OptionsLeftArm);
        if (!createDriver(dd_left_arm))
        {
            fprintf(stderr,"ERROR: unable to create left arm device driver...quitting\n");
            return false;
        }

        OptionsRightArm.put("device","remote_controlboard");
        OptionsRightArm.put("local","/gravityCompensator/right_arm/client");
        OptionsRightArm.put("remote","/icub/right_arm");
        dd_right_arm = new PolyDriver(OptionsRightArm);
        if (!createDriver(dd_right_arm))
        {
            fprintf(stderr,"ERROR: unable to create right arm device driver...quitting\n");
            return false;
        }

		if (legs_enabled)
		{
			OptionsLeftLeg.put("device","remote_controlboard");
			OptionsLeftLeg.put("local","/gravityCompensator/left_leg/client");
			OptionsLeftLeg.put("remote","/icub/left_leg");
			dd_left_leg = new PolyDriver(OptionsLeftLeg);
			if (!createDriver(dd_left_leg))
			{
				fprintf(stderr,"ERROR: unable to create left leg device driver...quitting\n");
				return false;
			}

			OptionsRightLeg.put("device","remote_controlboard");
			OptionsRightLeg.put("local","/gravityCompensator/right_leg/client");
			OptionsRightLeg.put("remote","/icub/right_leg");
			dd_right_leg = new PolyDriver(OptionsRightLeg);
			if (!createDriver(dd_right_leg))
			{
				fprintf(stderr,"ERROR: unable to create right leg device driver...quitting\n");
				return false;
			}
		}
		
		OptionsTorso.put("device","remote_controlboard");
		OptionsTorso.put("local","/gravityCompensator/torso/client");
		OptionsTorso.put("remote","/icub/torso");

		dd_torso = new PolyDriver(OptionsTorso);
		if (!createDriver(dd_torso))
		{
			fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");

        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);        


        //--------------------------THREAD--------------------------

        g_comp = new gravityCompensatorThread(rate, dd_left_arm, dd_right_arm, dd_head, dd_left_leg, dd_right_leg, dd_torso);
        fprintf(stderr,"ft thread istantiated...\n");
        g_comp->start();
        fprintf(stderr,"thread started\n");
        return true;
    }

	bool respond(const Bottle& command, Bottle& reply) 
	{
		Bottle position_bot;
		string helpMessage =  string(getName().c_str()) + 
							" commands are: \n" +  
							"help       to display this message\n" + 
							"on         to set the gravity compensation term \n" + 
							"off        to set the zero torque reference \n";

		  reply.clear(); 
		if (command.get(0).asString()=="help")
		{
			cout << helpMessage;
			reply.addString(helpMessage.c_str());
		}
		else if (command.get(0).asString()=="on" ||
			     command.get(0).asString()=="ON" )
		{
			gravity_mode = GRAVITY_COMPENSATION_ON;
			reply.addString("assigned gravity compensation feed-forward term");
	    }
		else if (command.get(0).asString()=="off" ||
			     command.get(0).asString()=="OFF" )
	    {
			gravity_mode = GRAVITY_COMPENSATION_OFF;
			reply.addString("gravity compensation off");
	    }
	    else
		{
			reply.addString("unknown command. type help.");
		}
		
		return true;
	}


    bool close()
    {
		/*if(g_comp->isRunning()) { g_comp->stop(); delete g_comp; g_comp = 0;}
		if(dd) { delete dd; dd = 0;}*/
        if(g_comp)
        {
            g_comp->stop();
            delete g_comp; g_comp = 0;
        }
	
		//closing ports
        rpcPort.interrupt();
		rpcPort.close();

        return true;
    }

    double getPeriod()  { return 1.0;  }
    bool updateModule() { return true; }
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wrechObserver/conf)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                        << endl;
        cout << "\t--rate       rate: the period used by the module. default 100ms (not less than 15ms)"						  << endl;
        cout << "\t--no_legs    this option disables the gravity compensation for the legs joints"								  << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    gravityModuleCompensator gcomp;

    return gcomp.runModule(rf);
}


