/* 
 * Copyright (C) <2010> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Serena Ivaldi
 * email:   serena.ivaldi@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/



#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynFwd.h>
#include <iCub/iDyn/iDynTransform.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "torqueObserver.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace std;
using namespace ctrl;
using namespace iDyn;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//			SENS TO  TORQUES         [RateThread]
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FILE *datas = fopen("ftMethodDiff.txt","w+");
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector SensToTorques::updateVelocity(const Vector &v)
{
	estElement.data=v;
	estElement.time=Time::now();
	return linEst->estimate(estElement);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector SensToTorques::updateAcceleration(const Vector &a)
{
	estElement.data=a;
	estElement.time=Time::now();
	return quadEst->estimate(estElement);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SensToTorques::SensToTorques(int _rate, PolyDriver *_dd, PolyDriver *_tt, string _type, string _name)
:RateThread(_rate), dd(_dd), tt(_tt)
{
	// attach ports
    string fwdSlash = "/";
    string port = fwdSlash+_name;
    port += (fwdSlash+_type).c_str();
	// input port:  /moduleName/robotPart/FT:i
	port_FT = new BufferedPort<Vector>;	  
	port_Wrench = new BufferedPort<Vector>;	  
	port_Torques = new BufferedPort<Vector>;	  
	// output ports: /moduleName/robotPart/torques:o  /moduleName/robotPart/FTendeff:o
	port_FT->open((port+"/FT:i").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/FT:i").c_str()<<endl;
	port_Wrench->open((port+"/wrench:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/wrench:o").c_str()<<endl;
	port_Torques->open((port+"/torques:o").c_str());
	cout<<"TorqueObserver: created port: "<<(port+"/torques:o").c_str()<<endl;

	// initialize estimators of velocity and acceleration
	// they are necessary to avoid noise in this components
	linEst =new AWLinEstimator(16,1.0);
	quadEst=new AWQuadEstimator(25,1.0);

	// Checking device
	dd->view(iencs);
	if (tt)
		tt->view(iencs_torso);
	// check leg type
	if(tt)
	{
		if(_type == "left_arm")
		{
			limb = new iCubArmDyn("left");
			limbInv = new iCubArmDyn("left");
		}
		else 
		{
			limb = new iCubArmDyn("right");
			limbInv = new iCubArmDyn("left");
		}
		sens = new iDynSensorArm(dynamic_cast<iCubArmDyn *>(limb),DYNAMIC,VERBOSE);
		sensInv = new iDynInvSensorArm(dynamic_cast<iCubArmDyn *>(limbInv),DYNAMIC,VERBOSE);
	}
	else
	{
		if(_type == "left_leg")
		{
			limb = new iCubLegDyn("left");
			limbInv = new iCubLegDyn("left");
		}
		else
		{
			limb = new iCubLegDyn("right");
			limbInv = new iCubLegDyn("right");
		}
		sens = new iDynSensorLeg(dynamic_cast<iCubLegDyn *>(limb),DYNAMIC,VERBOSE);
		sensInv = new iDynInvSensorLeg(dynamic_cast<iCubLegDyn *>(limbInv),DYNAMIC,VERBOSE);
	}
	
	// the sensor solver for the leg
	int jnt1=0;
    int jnt2=0;

    iencs->getAxes(&jnt1);
    encoders.resize(jnt1);
	encoders.zero();

    if (tt)
    {
        iencs_torso->getAxes(&jnt2);
        encodersTorso.resize(jnt2);
		encodersTorso.zero();
    }

	// init all variables
	// first the unused ones
	// joints var
	int jnt=jnt1+jnt2;
    q.resize(jnt,0.0);
    dq.resize(jnt,0.0);
    d2q.resize(jnt,0.0);

	q.zero(); dq.zero(); d2q.zero();
	// init Newt-Eul
	w0.resize(3);dw0.resize(3);d2p0.resize(3);Fend.resize(3);Mend.resize(3);
	w0.zero(); dw0.zero(); d2p0.zero();Fend.zero();Mend.zero();
	d2p0(2)=9.81;
	// forces moments torques
	FTsensor.resize(6); FTsensor.zero();
	FTendeff.resize(6); FTendeff.zero();
	sensorOffset.resize(6); sensorOffset.zero();

	//set joints pos/vel/acc
	limb->setAng(q);
	limb->setDAng(dq);
	limb->setD2Ang(d2q);
	limb->prepareNewtonEuler(DYNAMIC);
	limb->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);
	
	limbInv->setAng(q);
	limbInv->setDAng(dq);
	limbInv->setD2Ang(d2q);
	limbInv->prepareNewtonEuler(DYNAMIC);
	limbInv->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);

	//see encoders
	int Njoints;
	iencs->getAxes(&Njoints);
	encoders.resize(Njoints);

	cout<<"SensToTorques: leg="<<endl;

	//other
	FTmeasure = NULL;

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool SensToTorques::threadInit()
{		 
	Time::delay(2.0);
	calibrateOffset(200);
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::run()
{	  
	
	// read encoders values, update limb model with pos/vel/acc
	// read FT sensor measurements
	readAndUpdate(false);
  	
	// estimate sensor FT
	//limbInv->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
	//sensInv->computeSensorForceMoment();
	//Vector sensEstim = -1.0*sensInv->getSensorForceMoment();

	//compute torques
	sens->computeFromSensorNewtonEuler(-1.0*(FTsensor  - sensorOffset));
	//fprintf(stderr,".");
	
	//get torques and FTendeff
	Tau = sens->getTorques();
	//Vector Tinv = limbInv->getTorques();
	FTendeff = -1.0*sens->getForceMomentEndEff();
		
	//prepare ports with datas
	port_Torques->prepare() = Tau;
	port_Wrench->prepare() = FTendeff;

	//send data
	port_Torques->write();
	port_Wrench->write();

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::threadRelease()
{
	fclose(datas);
	cerr<<"SensToTorques: deleting dynamic structures ..";
	if(sens)		{delete sens;		sens = NULL; cout<<"sens ";}
	if(sensInv)		{delete sensInv;	sensInv = NULL; cout<<"sensInv ";}
	if(linEst)		{delete linEst;		linEst = NULL; cout<<"linEst ";}
	if(quadEst)		{delete quadEst;	quadEst = NULL; cout<<"quadEst ";}
	//if(FTmeasure)	{delete FTmeasure;	FTmeasure = NULL; cout<<"FTmeasure "; }
	if(limb)		{delete limb;		limb=NULL; cout<<"limb ";}
	if(limbInv)		{delete limbInv;	limbInv=NULL; cout<<"limbInv ";}
	if(port_FT)		{delete port_FT; port_FT = NULL; cout<<"port_FT ";}	  
	if(port_Wrench)	{delete port_Wrench; port_Wrench = NULL; cout<<"port_Wrench ";}	  
	if(port_Torques){delete port_Torques; port_Torques = NULL; cout<<"port_Torques ";}	  
	cout<<" .. done "<<endl;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::calibrateOffset(const unsigned int Ntrials)
{
	cout<<"SensToTorques: starting sensor offset calibration .."<<endl;

	sensorOffset.zero();
	// N trials to get a more accurate estimation
	for(unsigned int i=0; i<Ntrials; i++)
	{
		//read joints and ft sensor
		readAndUpdate(true);
		// compute forces/moments/torques along the chain
		limbInv->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
		// compute on sensor
		sensInv->computeSensorForceMoment();
		// get an estimate of the force/moment in the sensor
		sensorOffset =  sensorOffset + FTsensor - (-1.0*sensInv->getSensorForceMoment());
	}

	sensorOffset = sensorOffset * (1.0/(double)(Ntrials));

	cout<<"SensToTorques: sensor offset calibration completed"<<endl
		<<"               offset forces: "
		<<sensorOffset[0]<<"; "<<sensorOffset[1]<<"; "<<sensorOffset[2]<<"; "<<endl
		<<"               offset moment: "
		<<sensorOffset[3]<<"; "<<sensorOffset[4]<<"; "<<sensorOffset[5]<<"; "<<endl;

	Time::delay(5.0);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::readAndUpdate(bool waitMeasure)
{
	int i;
	//read encoders values
	iencs->getEncoders(encoders.data());
	if(part=="arm")
	{
		iencs_torso->getEncoders(encodersTorso.data());
		for( i=0;i<3;i++ )
			q(i) = encodersTorso(2-i);
		for( i=3;i<q.length();i++ )
			q(i) = encoders(i-3);
	} 
	else
	{
		for(i=0;i<q.length();i++)
			q(i) = encoders(i);
	}
	//estimate velocity and accelerations
	dq = updateVelocity(q);
	d2q = updateAcceleration(q);

	// update joints
	limb->setAng(M_PI/180.0 * q);
	limb->setDAng(M_PI/180.0 * dq);
	limb->setD2Ang(M_PI/180.0 * d2q);

	limbInv->setAng(M_PI/180.0 * q);
	limbInv->setDAng(M_PI/180.0 * dq);
	limbInv->setD2Ang(M_PI/180.0 * d2q);

	//read FT sensor measurements
	FTmeasure = port_FT->read(waitMeasure);
	if((FTmeasure!=0))
	{
		FTsensor = *FTmeasure;
	} 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	  

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//			TORQUE OBSERVER            [RFModule]
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TorqueObserver::TorqueObserver()
{
	sens2torques= NULL;
	dd		= NULL;
	tt	= NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::checkDriver(PolyDriver *_dd)
{
	// check driver
    if(!_dd || !(_dd->isValid()))
	{
		cerr<<"TorqueObserver: error: could not instantiate the device driver. Closing."<<endl;
		return false;
	}
	//check encoders
	IEncoders *encs;
	if(!_dd->view(encs))
	{
		cerr<<"TorqueObserver: error: could not view encoders properly. Closing."<<endl;
		return false;
	}
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::configure(ResourceFinder &rf)
{
	string moduleName;	// ftObs
	string robotName;	// icub
	string robotPart;	// left_arm, right_leg
	string limb;		// leg, arm
	string type;		// right, left
	int rate;			// rate

	// getting parameters from RF: name, robot, part, rate
	if(rf.check("name")) 		moduleName = rf.find("name").asString();
	else 		moduleName = "ftObs";
	if (rf.check("robot"))		robotName = rf.find("robot").asString().c_str();
    else		robotName = "icub";
	if (rf.check("rate"))		rate = rf.find("rate").asInt();
    else		rate = 100;
	if (rf.check("part"))		robotPart = rf.find("part").asString().c_str();
    else
	{
		cerr<<"TorqueObserver: error in configuration: missing robot part in configuration parameters. Impossible to proceed."<<endl;
		Time::delay(3.0);
        return false;
	}

	//summary of config param 
	cout<<"TorqueObserver: module = "<<moduleName<<endl
		<<"                robot   = "<<robotName<<endl
		<<"                part   = "<<robotPart<<endl
		<<"                rate   = "<<rate<<" ms"<<endl;
	
	// now create the devices
	//device driver of the limb
	cout<<"TorqueObserver: creating "<<robotPart<<" polyDriver"<<endl;
	OptionsLimb.put("robot",robotName.c_str());
	OptionsLimb.put("part",robotPart.c_str());
	OptionsLimb.put("device","remote_controlboard");
	OptionsLimb.put("local",("/"+robotName+"/"+robotPart+"/client").c_str());
	OptionsLimb.put("remote",("/"+robotName+"/"+robotPart).c_str());
	dd = new PolyDriver(OptionsLimb);
	if(!checkDriver(dd)) 
	{
		cerr<<"TorqueObserver: error: unable to create /"<<robotName<<"/"<<robotPart<<" device driver...quitting"<<endl;
		return false;
	}


	// note: the arm needs the torso
	if(robotPart=="left_arm" || robotPart=="right_arm")
	{
		//torso
		cout<<"TorqueObserver: creating torso polyDriver"<<endl;
		OptionsTorso.put("robot",robotName.c_str());
		OptionsTorso.put("part","torso");
		OptionsTorso.put("device","remote_controlboard");
		OptionsTorso.put("local",("/"+robotName+"/torso/client").c_str());
		OptionsTorso.put("remote",("/"+robotName+"/torso").c_str());
		tt = new PolyDriver(OptionsTorso);
		if(!checkDriver(tt)) 
		{
			cerr<<"TorqueObserver: error: unable to create /"<<robotName<<"/torso device driver...quitting"<<endl;
			return false;
		}
		//arm with torso	
	}
	else 
	{
		tt = NULL;
	}
	sens2torques = new SensToTorques(rate, dd, tt, robotPart, moduleName);
	
	//now the thread can start
	sens2torques->start();
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double TorqueObserver::getPeriod()	
{ 
	return 1; 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::updateModule() 
{ 
	return true; 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::close()
{
	cout<<"TorqueObserver is closing: "<<endl;
	if (sens2torques) 
	{
		sens2torques->stop();
		//delete sens2torques; 
		//sens2torques=NULL;
	}

	if (dd) {delete dd; dd=NULL;}
	if (tt) {delete tt; tt=NULL;}

	Time::delay(1.0);

	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::interruptModule()
{
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::respond(const Bottle& command, Bottle& reply)
{
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

