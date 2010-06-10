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
SensToTorques::SensToTorques(int _rate, PolyDriver *_pd_leg, BufferedPort<Vector> *port_FT, BufferedPort<Vector> *port_Torques, BufferedPort<Vector> *port_FTendef, string _type)
:RateThread(_rate), pd_limb(_pd_leg)
{
	// attach ports
	// input port:  /moduleName/robotPart/FT:i
	port_i_FT = port_FT;	  
	// output ports: /moduleName/robotPart/torques:o  /moduleName/robotPart/FTendeff:o
	port_o_Torques = port_Torques;
	port_o_FTendef = port_FTendef;

	// initialize estimators of velocity and acceleration
	// they are necessary to avoid noise in this components
	linEst =new AWLinEstimator(16,1.0);
	quadEst=new AWQuadEstimator(25,1.0);

	// Checking device
	pd_limb->view(iencs);

	// check leg type
	part = "leg";
	type = _type;
	if((type!="left")&&(type!="right"))
	{
		type="left";
		cerr<<"SensToTorques: error: leg type was "<<_type<<", now is "<<type<<" by default."<<endl;
	}

	// new icub leg
	limb = new iCubLegDyn(type);
	
	// the sensor solver for the leg
	sens = new iDynSensorLeg(dynamic_cast<iCubLegDyn *>(limb),DYNAMIC,VERBOSE);

	// init all variables
	// first the unused ones
	iencs_torso = NULL;
	pd_torso = NULL;
	encodersTorso.resize(1); encodersTorso.zero();
	// joints var
	q.resize(6); dq.resize(6); d2q.resize(6);
	q.zero(); dq.zero(); d2q.zero();
	// init Newt-Eul
	w0.resize(3);dw0.resize(3);d2p0.resize(3);Fend.resize(3);Mend.resize(3);
	w0.zero(); dw0.zero(); d2p0.zero();Fend.zero();Mend.zero();
	d2p0(0)=9.81;
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

	//see encoders
	int Njoints;
	iencs->getAxes(&Njoints);
	encoders.resize(Njoints);

	cout<<"SensToTorques: leg="<<endl;

	//other
	FTmeasure = NULL;

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SensToTorques::SensToTorques(int _rate, PolyDriver *_pd_arm, PolyDriver *_pd_torso, BufferedPort<Vector> *port_FT, BufferedPort<Vector> *port_Torques, BufferedPort<Vector> *port_FTendef, string _type)
:RateThread(_rate), pd_limb(_pd_arm), pd_torso(_pd_torso)
{

	// attach ports
	cout<<"SensToTorques: attaching ports"<<endl;
	// input port:  /moduleName/robotPart/FT:i
	port_i_FT = port_FT;	  
	// output ports: /moduleName/robotPart/torques:o  /moduleName/robotPart/FTendeff:o
	port_o_Torques = port_Torques;
	port_o_FTendef = port_FTendef;

	// initialize estimators of velocity and acceleration
	// they are necessary to avoid noise in this components
	linEst =new AWLinEstimator(16,1.0);
	quadEst=new AWQuadEstimator(25,1.0);

	// Checking device
	cout<<"SensToTorques: view device"<<endl;
	pd_limb->view(iencs);
	pd_torso->view(iencs_torso);

	// check arm type
	part = "arm";
	type = _type;
	if((type!="left")&&(type!="right"))
	{
		type="left";
		cerr<<"SensToTorques: error: arm type was "<<_type<<", now is "<<type<<" by default."<<endl;
	}

	// new icub arm
	limb = new iCubArmDyn(type);
	limbInv = new iCubArmDyn(type);
	
	// the sensor solver for the arm

	sens = new iDynSensorArm(dynamic_cast<iCubArmDyn *>(limb),DYNAMIC,VERBOSE);
	sensInv = new iDynInvSensorArm(dynamic_cast<iCubArmDyn *>(limbInv),DYNAMIC,VERBOSE);


	cout<<"SensToTorques: arm and iDynSensor created"<<endl;

	// init all variables
	// joints var
	q.resize(10); dq.resize(10); d2q.resize(10);
	q.zero(); dq.zero(); d2q.zero();
	// init Newt-Eul
	w0.resize(3);dw0.resize(3);d2p0.resize(3);Fend.resize(3);Mend.resize(3);
	w0.zero(); dw0.zero(); d2p0.zero();Fend.zero();Mend.zero();
	d2p0(0)=9.81;
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

	//see encoders
	int Njoints, Ntorso;
	iencs->getAxes(&Njoints);
	iencs_torso->getAxes(&Ntorso);
	encoders.resize(Njoints);
	encodersTorso.resize(Ntorso); 

	cout<<"SensToTorques: arm="<<Njoints<<" torso="<<Ntorso<<endl;

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
  
	// subtracts sensor offset and model
	//FTsensor = FTsensor - sensorOffset;
	
	// estimate sensor FT
	limbInv->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
	sensInv->computeSensorForceMoment();
	Vector sensEstim = -1.0*sensInv->getSensorForceMoment();

	//compute torques
	Vector sensFTestim = FTsensor  - sensorOffset - sensEstim;
	sens->computeFromSensorNewtonEuler(sensFTestim);
	
	//get torques and FTendeff
	Tau = sens->getTorques();
	FTendeff = sens->getForceMomentEndEff();
	cout<<Time::now()-time0<<endl;
	time0 = Time::now();

	//

	//if(part=="arm")
	//{
	//	Vector tau(7);
	//	for(int i=0;i<7;i++)
	//		tau[i]=Tau[i+3];
	//	cout<<tau.toString()<<endl;
	//}
	//else
	//{
	//	cout<<Tau.toString()<<endl;
	//}

	//cout<<sensFTestim.toString()<<endl;
	
	//prepare ports with datas
	port_o_Torques->prepare() = Tau;
	port_o_FTendef->prepare() = FTendeff;

	//send data
	port_o_Torques->write();
	port_o_FTendef->write();

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::threadRelease()
{
	cerr<<"SensToTorques: deleting dynamic structures ..";
	if(sens)		{delete sens;		sens = NULL; cout<<"sens ";}
	if(sensInv)		{delete sensInv;	sensInv = NULL; cout<<"sensInv ";}
	if(linEst)		{delete linEst;		linEst = NULL; cout<<"linEst ";}
	if(quadEst)		{delete quadEst;	quadEst = NULL; cout<<"quadEst ";}
	//if(FTmeasure)	{delete FTmeasure;	FTmeasure = NULL; cout<<"FTmeasure "; }
	if(limb)		{delete limb;		limb=NULL; cout<<"limb ";}
	if(limbInv)		{delete limbInv;	limbInv=NULL; cout<<"limbInv ";}
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
	FTmeasure = port_i_FT->read(waitMeasure);
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
	pd_limb		= NULL;
	pd_torso	= NULL;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::checkDriver(PolyDriver *_pd_limb)
{
	// check driver
    if(!_pd_limb || !(_pd_limb->isValid()))
	{
		cerr<<"TorqueObserver: error: could not instantiate the device driver. Closing."<<endl;
		return false;
	}
	//check encoders
	IEncoders *encs;
	if(!_pd_limb->view(encs))
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
		<<"                robot  = "<<robotName<<endl
		<<"                part   = "<<robotPart<<endl
		<<"                rate   = "<<rate<<" ms"<<endl;
	  
	//creating ports
	// input port:  /moduleName/robotPart/FT:i
	string port_ft = "/"+moduleName+"/"+robotPart+"/FT:i";
	port_i_FT.open(port_ft.c_str());
	cout<<"TorqueObserver: created port: "<<port_ft<<endl;

	// output ports: /moduleName/robotPart/torques:o  /moduleName/robotPart/FTendeff:o
	string port_tau = "/"+moduleName+"/"+robotPart+"/torques:o";
	string port_eef = "/"+moduleName+"/"+robotPart+"/FTendeff:o";
	port_o_Torques.open(port_tau.c_str());
	cout<<"TorqueObserver: created port: "<<port_tau<<endl;
	port_o_FTendef.open(port_eef.c_str());
	cout<<"TorqueObserver: created port: "<<port_eef<<endl;

	// now creating device drivers
	if(robotPart=="left_arm")
	{limb="arm"; type="left";}
	else if(robotPart=="right_arm")
	{limb="arm"; type="right";}
	else if(robotPart=="left_leg")
	{limb="leg"; type="left";}
	else if(robotPart=="right_leg")
	{limb="leg"; type="right";}
	else
	{
		cerr<<"TorqueObserver: error: robotPart is unknown: please use leg/arm and right/left"<<endl;
		return false;
	}
	

	// now create the devices
	//device driver of the limb
	cout<<"TorqueObserver: creating "<<robotPart<<" polyDriver"<<endl;
	OptionsLimb.put("robot",robotName.c_str());
	OptionsLimb.put("part",robotPart.c_str());
	OptionsLimb.put("device","remote_controlboard");
	OptionsLimb.put("local",("/"+robotName+"/"+robotPart+"/client").c_str());
	OptionsLimb.put("remote",("/"+robotName+"/"+robotPart).c_str());
	pd_limb = new PolyDriver(OptionsLimb);
	if(!checkDriver(pd_limb)) 
	{
		cerr<<"TorqueObserver: error: unable to create /"<<robotName<<"/"<<robotPart<<" device driver...quitting"<<endl;
		return false;
	}


	// note: the arm needs the torso
	if(limb=="arm")
	{
		//torso
		cout<<"TorqueObserver: creating torso polyDriver"<<endl;
		OptionsTorso.put("robot",robotName.c_str());
		OptionsTorso.put("part","torso");
		OptionsTorso.put("device","remote_controlboard");
		OptionsTorso.put("local",("/"+robotName+"/torso/client").c_str());
		OptionsTorso.put("remote",("/"+robotName+"/torso").c_str());
		pd_torso = new PolyDriver(OptionsTorso);
		if(!checkDriver(pd_torso)) 
		{
			cerr<<"TorqueObserver: error: unable to create /"<<robotName<<"/torso device driver...quitting"<<endl;
			return false;
		}
		//arm with torso
		sens2torques = new SensToTorques(rate, pd_limb, pd_torso, &port_i_FT, &port_o_Torques, &port_o_FTendef, type);
	
	}
	else 
	{
		//leg
		sens2torques = new SensToTorques(rate, pd_limb, &port_i_FT, &port_o_Torques, &port_o_FTendef,type);
	}
	
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

	if (pd_limb) {delete pd_limb; pd_limb=NULL;}
	if (pd_torso) {delete pd_torso; pd_torso=NULL;}

	//finally closing ports
	cout<<"TorqueObserver: closing ports ..."<<endl;
	port_o_Torques.close(); cout<<"                output torques closed"<<endl;
	port_o_FTendef.close(); cout<<"                output end-eff closed"<<endl;
	port_i_FT.close();      cout<<"                input measures closed"<<endl;
	cout<<"TorqueObserver: all ports closed ..."<<endl;

	Time::delay(5.0);

	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::interruptModule()
{
	port_o_Torques.interrupt(); 
	port_o_FTendef.interrupt();
	port_i_FT.interrupt();
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::respond(const Bottle& command, Bottle& reply)
{
	return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

