/**
* @ingroup icub_module
*
* \defgroup zeroForceControl zeroForceControl
*
*
* Perform zero force control on the iCub arms.
*
* \author Matteo Fumagalli
*
* Copyright (C) 2010 RobotCub Consortium
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* This file can be edited at src/myModule/main.cpp.
**/

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "filter.h"
#include "iCub/iFC.h"
#include "iCub/iKinFwd.h"

const int SAMPLER_RATE = 10;
const int FT_VALUES = 6;

const bool verbose = false;
const int CPRNT = 100;
const int CALIBRATION_OK = true; // should be true when FT calibration will be ok
enum {ZEROFORCECONTROL=0, IMPEDANCE=1};
int   control_mode=ZEROFORCECONTROL;
bool  filter_enabled=true;

#define CONNECTION_ERROR 0
#define CONNECTION_OK	 1

#define CONTROL_ON  1

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
//using namespace ctrl;
using namespace std;

using namespace iFC;
using namespace iKin;

/**
*
* A class for defining the 4-DOF iCub Arm (3 of the shoulder and one for the elbow. The end effector is placed on the wrist)
*/
class iCubArm4DOF : public iKinLimb
{
protected:
    virtual void _allocate_limb(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubArm4DOF();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" arm
    */
    iCubArm4DOF(const std::string &_type);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubArm4DOF(const iCubArm4DOF &arm);
};


/**
* \ingroup iKinFwd
*
* A class for defining the 6-DOF iCub Leg
*/
class iCubLeg4DOF : public iKinLimb
{
protected:
    virtual void _allocate_limb(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubLeg4DOF();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" leg
    */
    iCubLeg4DOF(const std::string &_type);

    /**
    * Creates a new Leg from an already existing Leg object.
    * @param leg is the Leg to be copied.
    */
    iCubLeg4DOF(const iCubLeg4DOF &leg);
};


// class dataCollector: class for reading from Vrow and providing for FT on an output port
class ftControl: public RateThread
{
private:
	//device:
	PolyDriver *dd;
	IPositionControl *ipos;
	IEncoders *iencs;
	IPidControl *ipids;
	IVelocityControl *ivels;
	IAmplifierControl *iamps;

	// Pids
	Pid *iCubPid; //needed when the program stops
	Pid *FTPid; // to be set to zero

	Vector encoders;
	Vector angs;
	Vector angs_old;
	Vector speeds;
	Vector initPosition;
	Vector desPosition;

	Vector maxJntLimits;
	Vector minJntLimits;

	Vector FTs;
	Vector FTs_init;
	Vector FT;
	Vector FTj;
	int count;
	//iCubArm *arm;
	iKinLimb *iCubLimb;
	//iCubArm4DOF *arm;
	iKinChain *chain;
	
	int limbJnt;

	iFB *FTB;
	iFTransform *sensor;
	int sensorLink;

	Matrix Rs;
	Vector ps;

	Matrix T_all;

	BufferedPort<Vector> *port_FT;
	Vector Datum;
	bool first;

	Vector tau;
	Vector tauDes;
	Vector tauSafe;
	Vector tauFilt;

	int watchDog;
	Stamp info;
	double time, time0;
	int countTime, countTime0;

	Vector *datas;
    Vector kp;
	Vector kspr;



	void shoulderCouplingInit()
	{
		  double R0, R1, R2, K0, K1, K2, Jm0, Jm1, Jm2, a;
		  Matrix T; 
	
		//Shoulder motors parameters:
		  R0 = 0.8967; K0 = 0.05; Jm0 = 8.47E-6;
		  R1 = R2 = 0.8363; K1 = K2 = 0.0280; Jm1 = Jm2 = 5.15E-6;
		  a = 40.0/65.0;
		  T.resize(3,3);
		  T_all.resize(limbJnt,limbJnt);
		  T = 0.0; //T = Tvt*Tjm'
		  T_all = 0.0;
		  T(0,0) = R0/K0; 
		  T(0,1) = R0/K0; 
		  T(1,1) = R1/K1*a; 
		  T(2,0) = 0; 
		  T(2,1) = -R1/K1*a;
		  T(2,2) = R2/K2*a;
		  T = 0.056*T;//pinv(T)*T.transposed();

		  for(int i=0;i<3;i++)
			  for(int j=0;j<3;j++)
				  T_all(i,j) = T(i,j);
		  if(limbJnt>3)
		  {
		  for(int i=3;i<limbJnt;i++)
			  for(int j=3;j<limbJnt;j++)
				  T_all(i,j) = 1.0; //added elbow
		  }
	}

	void initializePositionAndLimits(string limb)
	{
		if(limbJnt!=0)
		{
			if (strcmp(limb.c_str(), "left_arm")==0)
			{
				initPosition.resize(limbJnt);
				desPosition.resize(limbJnt);
				initPosition = 0.0;
				initPosition(0) = -10.0; initPosition(1) = 20.0; initPosition(2) = 15.0; initPosition(3) = 15.0;
				desPosition=initPosition;
				maxJntLimits.resize(4);
				maxJntLimits(0) = 2.0; maxJntLimits(1) = 120.0; maxJntLimits(2) = 90.0; maxJntLimits(3) = 95.0;
				minJntLimits.resize(4);
				minJntLimits(0) = -95.0; minJntLimits(1) = 0.0; minJntLimits(2) = -20.0; minJntLimits(3) = 10.0;
			}
			else if (strcmp(limb.c_str(), "right_arm")==0)
			{
				initPosition.resize(limbJnt);
				desPosition.resize(limbJnt);
				initPosition = 0.0;
				initPosition(0) = -10.0; initPosition(1) = 20.0; initPosition(2) = 15.0; initPosition(3) = 15.0;
				desPosition=initPosition;
				maxJntLimits.resize(4);
				maxJntLimits(0) = 2.0; maxJntLimits(1) = 120.0; maxJntLimits(2) = 90.0; maxJntLimits(3) = 95.0;
				minJntLimits.resize(4);
				minJntLimits(0) = -95.0; minJntLimits(1) = 0.0; minJntLimits(2) = -20.0; minJntLimits(3) = 10.0;
			}
			else if (strcmp(limb.c_str(), "left_leg")==0)
			{
				initPosition.resize(limbJnt);
				desPosition.resize(limbJnt);
				initPosition = 0.0;
				initPosition(0) = 15.0; initPosition(1) = 15.0; initPosition(2) = 0.0; initPosition(3) = -20.0;
				desPosition=initPosition;
				maxJntLimits.resize(4);
				maxJntLimits(0) = 130.0; maxJntLimits(1) = 100.0; maxJntLimits(2) = 30.0; maxJntLimits(3) = -10.0;
				minJntLimits.resize(4);
				minJntLimits(0) = -30.0; minJntLimits(1) = 0.0; minJntLimits(2) = -30.0; minJntLimits(3) = -110.0;
			}
			else if (strcmp(limb.c_str(), "right_leg")==0)
			{
				initPosition.resize(limbJnt);
				desPosition.resize(limbJnt);
				initPosition = 0.0;
				initPosition(0) = 15.0; initPosition(1) = 15.0; initPosition(2) = 0.0; initPosition(3) = -20.0;
				desPosition=initPosition;
				maxJntLimits.resize(4);
				maxJntLimits(0) = 130.0; maxJntLimits(1) = 100.0; maxJntLimits(2) = 30.0; maxJntLimits(3) = -10.0;
				minJntLimits.resize(4);
				minJntLimits(0) = -30.0; minJntLimits(1) = 0.0; minJntLimits(2) = -30.0; minJntLimits(3) = -110.0;
			}
		}
	}

	void initLimb(string limb)
	{
		if (strcmp(limb.c_str(), "left_arm")==0)
		  {			  
			  limbJnt = 4;
			  sensorLink = 2;
			  shoulderCouplingInit();
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubArm4DOF("left");
              Rs(0,0) = Rs(2,1) = 1.0;  Rs(1,2) = -1.0;
              ps(1) = 0.10;
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.15;
			  if (filter_enabled)
			  {
				kp(0) = -62.0;	kp(1) = -50.0;	kp(2) = -55.0;	kp(3) = -100; //kp(2) was 62
			  }
			  else
			  {
				kp(0) = -25.0;	kp(1) = -25.0;	kp(2) = -25.0;	kp(3) = -30.0;
			  }
              fprintf(stderr, "Opening left arm ... \n");
		  }
		  else if (strcmp(limb.c_str(), "right_arm")==0)
		  {  
			  limbJnt = 4;
			  sensorLink = 2;
			  shoulderCouplingInit();
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubArm4DOF("right");
              Rs(0,0) = -1.0; Rs(2,1) = 1.0;  Rs(1,2) = 1.0;
              ps(1) = -0.10;
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.15;
			  if (filter_enabled)
			  {
				kp(0) =  62.0;	kp(1) =  50.0;	kp(2) =  55.0;	kp(3) =  100;
			  }
			  else
			  {
				kp(0) =  25.0;	kp(1) =  25.0;	kp(2) =  25.0;	kp(3) =  30.0;
			  }
              fprintf(stderr, "Opening right arm ... \n");
		  } 
		  else if (strcmp(limb.c_str(), "left_leg")==0)
		  {
			  limbJnt = 4;
			  sensorLink = 1;
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubLeg4DOF("left");
              Rs(1,0) = -1.0; Rs(0,1) = -1.0;  Rs(2,2) = -1.0;
              ps(2) = -0.10;
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  //GAINS gains
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.3;
			  if (filter_enabled)
			  {
				kp(0) =  20.0;	kp(1) =  -14.0;	kp(2) =  94.0;	kp(3) =  -94.0; 
			  }
			  else
			  {
				 kp(0) =  10;	kp(1) =  -5;	kp(2) =  40;	kp(3) =  -50; 
			  }
			  T_all=eye(limbJnt,limbJnt);
			  fprintf(stderr, "Opening left leg ... \n");
		  }
		  else if (strcmp(limb.c_str(), "right_leg")==0)
		  {
			  limbJnt = 4;
			  sensorLink = 1;
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubLeg4DOF("right");
              Rs(1,0) = -1.0; Rs(0,1) = 1.0;  Rs(2,2) = 1.0;
              ps(2) = 0.10;
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  //GAINS gains
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.3;
			  if (filter_enabled)
			  {
				kp(0) = -20.0;	kp(1) =  14.0;	kp(2) =  -94.0;	kp(3) =  94.0; 
			  }
			  else
			  {
				 kp(0) =  -10;	kp(1) =   5;	kp(2) =  -40;	kp(3) =  50; 
			  }
			  T_all=eye(limbJnt,limbJnt);
			  fprintf(stderr, "Opening right leg ... \n");
		  }
		  else
          {
              fprintf(stderr, "nothing will be opened ... \n");
          }
		  initializePositionAndLimits(limb);
	}

    
public:
	void setDesiredPositions()			
	{
		for (int i=0; i<limbJnt; i++)
			desPosition(i)=encoders(i);
	}

	ftControl(int _rate, PolyDriver *_dd, BufferedPort<Vector> *_port_FT, ResourceFinder &_rf, string limb):	  
	  RateThread(_rate), dd(_dd) 
	  {
		  iCubPid = 0;
		  FTPid = 0;
		  port_FT = _port_FT;
		  dd->view(ipos);
		  dd->view(ivels);
		  dd->view(iencs);
		  dd->view(ipids);
		  dd->view(iamps);

		  int nJnt;
		  iencs->getAxes(&nJnt);
		  encoders.resize(nJnt);
		  iencs->getEncoders(encoders.data());

		  // Elbow to FT sensor variables
		  Rs.resize(3,3);     Rs=0.0;
		  ps.resize(3);		  ps=0.0;
          kp=0.0;
		  kspr=0.0;

		  initLimb(limb);
		  sensor = new iFTransform(Rs,ps);
		  chain = iCubLimb->asChain();
		  iCubLimb->setAllConstraints(false);

		  FTB = new iFB(sensorLink);
		  FTB->attach(iCubLimb);
		  FTB->attach(sensor);

		  first = true;
		  FTs.resize(FT_VALUES);
		  FTs_init.resize(FT_VALUES);
		  FT.resize(FT_VALUES);
		  FTj.resize(limbJnt);

		  angs.resize(limbJnt);
		  angs_old.resize(limbJnt);
		  speeds.resize(limbJnt);
		  angs=0.0;
		  angs_old=0.0;
		  speeds=0.0;
		  for(int i=0;i<limbJnt;i++)
		  {
			  // Get a copy of iCub Pid values...
			  ipids->getPid(i,iCubPid+i);
			  // Set the Pids for force control mode:
			  ipids->getPid(i,FTPid+i);
			  FTPid[i].setKd(0.0);
			  FTPid[i].setKp(0.0);
			  FTPid[i].setKi(0.0);	
			  FTPid[i].setOffset(0.0);	
			  // Setting the FTPid, iCub is controllable using setOffset
		  }
		  tau.resize(limbJnt);
		  tauDes.resize(limbJnt);
		  tauSafe.resize(limbJnt);
		  tauFilt.resize(limbJnt);
		  tau=0.0;
		  tauDes=0.0;
		  tauSafe=0.0;
		  tauFilt=0.0;

		  watchDog = 0;
		  time = time0 = 0.0;
		  countTime = countTime0 = 0;
	  }
	  bool threadInit()
	  {
		  FT.zero();
		  FTs.zero();
		  FTj.zero();
		  count = 0;

#ifdef CONTROL_ON		  
		  // Put the robot in the starting position
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipos->positionMove(i,initPosition[i]);
		  }
#endif			  

		  bool check = false;
		  //int count;
		  for(int i=0;i<limbJnt;i++)
		  {
			  check=false;
			  count=0;
			  while(!check && count < 10)
			  {
				  ipos->checkMotionDone(i,&check);
				  count++;
				  Time::delay(0.1);
			  }
		  }
		  
		  //Time::delay(1.0);
#ifdef CONTROL_ON
		  // Set the Pids to zero in order to control using setOffset		  
		  for(int i=0;i<limbJnt;i++)
		  {
			  	  ipids->setPid(i,FTPid[i]);  
		  }
//		  ipids->setPid(3,FTPid[3]); // use this on a single joint
//		  ipids->setPid(2,FTPid[2]); // use this on a single joint
//		  ipids->setPid(1,FTPid[1]); // use this on a single joint
//		  ipids->setPid(0,FTPid[0]); // use this on a single joint
#endif		  
		  count =0;
		  return true;
	  }
	  void run()
	  {
		  static double told=0;
		  static double t=0;
		  told=t;
		  t=Time::now();
          double tdiff=t-told;

		  datas=port_FT->read(false);
		  
		  //if(iencs->getEncoders(encoders.data()))
		  iencs->getEncoders(encoders.data());
		  //fprintf(stderr,"encoders length = %d\n", encoders.length());
		  for(int i=0; i<limbJnt;i++)
			{
				angs_old(i) = angs(i);
				angs(i) = encoders(i)*M_PI/180;
			}

		  iCubLimb->setAng(angs);
		  //else if(verbose) fprintf(stderr,"ERROR: no read from encoders\n");

		  port_FT->getEnvelope(info);
		  //time = info.getTime();
		  countTime = info.getCount();

		  int connected = CONNECTION_OK;

		  if(countTime - countTime0 > 0.0) 
		  {
			  connected = CONNECTION_OK;
			  countTime0 = countTime;
			  watchDog = 0;
		  }
		  else   
		  {
			  watchDog+=1;
		  }

		  if(watchDog>=20) 
		  {
			  connected = CONNECTION_ERROR;
		  }

		  //switch(connected)
		  //{ 
			//  case CONNECTION_ERROR:
			//	  FT=0.0;
			//	  fprintf(stderr,"ERROR: connection lost\n\n");
			//	  break;
			//  case CONNECTION_OK:
				  if(count>=CPRNT && verbose)
					  fprintf(stderr,"Connection ok...\n\n");
				  if(datas!=0)  
				  { 
					  if(CALIBRATION_OK)  FTs = *datas;
					  else  FTs = readFT();

					  if(first) FTs_init = FTs;
					  FT = FTB->getFB(FTs-FTs_init);

					  	
					  first = false;
				  }
				  else
				  {
					  if(!first)
					  {
						  FT = FTB->getFB();
					  }
					  else 
					  {
						  FT=0.0;
						  FTs=0.0;
						  FTj=0.0;
						  FTs_init=0.0;
					  }
				  }
				 // break;
			  
		  //}
				  
		  Vector Fe=FTB->getFe();

		  //gains: to be tuned
		  Matrix K;
		  Matrix Kspring;
		  K=eye(limbJnt,limbJnt);
		  Kspring=eye(limbJnt,limbJnt);
		  for(int i=0;i<limbJnt;i++) {K(i,i) = kp(i); Kspring(i,i) = kspr(i);}

		  //control: to be checked
		  Matrix J = iCubLimb->GeoJacobian();
         // const double Kspringc=0.3;
		  Vector tau(4);
		  tau=0.0;
		  tauDes=Kspring*((180.0/M_PI)*angs-desPosition); 
		  FTj=J.transposed()*FT;
		  if (control_mode==IMPEDANCE)
			{tau = K*(FTj-tauDes);} //USE THIS FOR IMPEDANCE CONTROL
		  else 
			{tau = K*(FTj);}        //USE THIS FOR ZERO FORCE CONTROL
		  Vector tauC= T_all*tau;

		  //filtering
		  for(int i=0;i<limbJnt;i++)
			  {
				  tauFilt(i) = lpf_ord1_3hz(tauC(i), i);
			  }
		  if (filter_enabled)
		  {
			  for(int i=0;i<limbJnt;i++)
			  {
				  tauC(i) = tauFilt(i);
			  }
		  }

		  //Limits check
		  for (int i=0; i<limbJnt; i++)
		  {
			  speeds(i) = (angs(i) - angs_old(i))/tdiff;
		  }
		  tauSafe = checkLimits(encoders, tauC, speeds); 

		  //saturation
		  Vector sat(4);
		  sat(0) = 300; sat(1) = 300; sat(2) = 300; sat(3) = 300;
		  for(int i=0;i<limbJnt;i++)
		  {
			  tauSafe(i)=(tauSafe(i)>sat(i))?sat(i):tauSafe(i);
			  tauSafe(i)=(tauSafe(i)<-sat(i))?-sat(i):tauSafe(i);
		  }

#ifdef CONTROL_ON
		  //Set the control offsets to the motors
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipids->setOffset(i,tauSafe(i));
		  }	
	//	  ipids->setOffset(3,tauSafe(3));  // use this on single joint
	//	  ipids->setOffset(2,tauSafe(2));  // use this on single joint
	//	  ipids->setOffset(1,tauSafe(1));  // use this on single joint
	//	  ipids->setOffset(0,tauSafe(0));  // use this on single joint
#endif		  

		  

		  /*if(count>=CPRNT)
		  {
			  Matrix He = arm->getH();
			  Matrix Hg = arm->getH(2);
			  Matrix Hs = FTB->getHs();
			  Matrix He2 = FTB->getHe();

			  fprintf(stderr,"He = ");
			  for(int i=0;i<3;i++)
			  {
				  for(int j=0;j<4;j++)
					  fprintf(stderr,"%.3lf\t", He(i,j));
				  fprintf(stderr,"\n");
			  }
			  fprintf(stderr,"\n\n");

			  fprintf(stderr,"He2 = ");
			  for(int i=0;i<3;i++)
			  {
				  for(int j=0;j<4;j++)
					  fprintf(stderr,"%.3lf\t", He2(i,j));
				  fprintf(stderr,"\n");
			  }
			  fprintf(stderr,"\n\n");

			  fprintf(stderr,"Hg = ");
			  for(int i=0;i<3;i++)
			  {
				  for(int j=0;j<4;j++)
					  fprintf(stderr,"%.3lf\t", Hg(i,j));
				  fprintf(stderr,"\n");
			  }
			  fprintf(stderr,"\n\n");

			  fprintf(stderr,"Hs = ");
			  for(int i=0;i<3;i++)
			  {
				  for(int j=0;j<4;j++)
					  fprintf(stderr,"%.3lf\t", Hs(i,j));
				  fprintf(stderr,"\n");
			  }
			  fprintf(stderr,"\n\n");

			  fprintf(stderr,"encs = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%.3lf\t", encoders(i));
			  fprintf(stderr,"\n");
			  fprintf(stderr,"ang = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%.3lf\t", arm->getAng(i+3)*180/M_PI);

			  fprintf(stderr,"DOF = %d \n", arm->getDOF());

			  count = 0;
		  }

		  */
		  if(count>=CPRNT)
		  {
			  fprintf(stderr,"Cycle duration=%f s, watchdog:%d \n", tdiff, watchDog);
#ifdef CONTROL_ON
			  fprintf(stderr,"Control ON:");
			  if (control_mode==IMPEDANCE) fprintf(stderr,"Impedance\n");
			  else fprintf(stderr,"Zero Force Control\n");
#else
			  fprintf(stderr,"Debug Mode, CONTROL_ON macro is not defined \n",tdiff);
			  if (control_mode==IMPEDANCE) fprintf(stderr," (impedance off)\n");
			  else fprintf(stderr,"(Zero Force Control off)\n");
#endif
			  fprintf(stderr,"FT = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", FT(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"FTs = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", FTs(i)-FTs_init(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"FTe = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", Fe(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"FTj = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", FTj(i));
			  fprintf(stderr,"\n");

			   fprintf(stderr,"tauD = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tauDes(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"encs = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", angs(i)*180/M_PI);
			  fprintf(stderr,"\n");

			  fprintf(stderr,"encd = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", desPosition(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"spds = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", speeds(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"tau  = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tau(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"tauF = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tauFilt(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"tauM = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tauSafe(i));
			  fprintf(stderr,"\n");
			  /*
			  // debug only
              fprintf(stderr,"J:\n");
			  for(int i=0;i<J.rows();i++)
			  {
				  for(int j=0;j<J.cols();j++)
					  fprintf(stderr,"%+.3lf\t", J(i,j));
				  fprintf(stderr,"\n");
			  }
			  fprintf(stderr,"\n\n\n");
			  */

			  fprintf(stderr,"\n\n");
			 
			  count = 0;
		  }
		  count++;
	  }

	  void threadRelease()
	  {

		  fprintf(stderr,"disabling amps...\n");
		  for(int i=0;i<limbJnt;i++)
          	  iamps->disableAmp(i);
		  fprintf(stderr,"disabling pids...\n");
		  for(int i=0;i<limbJnt;i++)
          	  ipids->disablePid(i);
		  fprintf(stderr,"setting old PIDS...\n");

#ifdef CONTROL_ON
		  //set again the original saved PIDS to the ICub
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipids->setPid(i,iCubPid[i]);
		  }
	//	  ipids->setPid(3,iCubPid[3]); //use this on single joint
	//	  ipids->setPid(2,iCubPid[2]); //use this on single joint
	//	  ipids->setPid(1,iCubPid[1]); //use this on single joint
	//	  ipids->setPid(0,iCubPid[0]); //use this on single joint
#endif
		  fprintf(stderr,"enabling amps...\n");
		  for(int i=0;i<limbJnt;i++)
          	  iamps->enableAmp(i);
		  fprintf(stderr,"enabling pids...\n");
		  for(int i=0;i<limbJnt;i++)
          	  ipids->enablePid(i);

	//	  if(datas) delete datas;

		  if(FTB) delete FTB;
		  if(iCubPid) delete[] iCubPid;
		  if(FTPid) delete[] FTPid;
	  }

	  /*bool checkSinglePosition(double qd, double q)
	  {
		  if(abs(qd-q)<=1.0) return true;
		  else return false;
	  }*/
	  Vector checkLimits(Vector q, Vector TAO, Vector dir)
	  {
		  Vector t = TAO;
		  for(int i=0;i<limbJnt;i++)
		  {
			  if(q(i)<=minJntLimits[i])
			  {
				  //if(verbose) 
				  {
					  fprintf(stderr,"J%d over limits %.2lf (%.2lf - %.2lf) ", i, q(i), minJntLimits[i], maxJntLimits[i]);
					  if (dir(i)>0)
					  {
						  //t(i) = t(i)/2; //for safety
						  fprintf(stderr,"Dir %.3lf>0,safe\n",dir(i));
					  }
					  else
					  {
						  t(i) = 0.0;
						  fprintf(stderr,"Dir %.3lf<0,STOPPING\n",dir(i));
					  }
				  }
			  }
			  if(q(i)>=maxJntLimits[i])
			  {
				  //if(verbose) 
				  {
					  fprintf(stderr,"J%d over limits %.2lf (%.2lf - %.2lf) ", i, q(i), minJntLimits[i], maxJntLimits[i]);
					  if (dir(i)<0)
					  {
						  //t(i) = t(i)/2; //for safety
						  fprintf(stderr,"Dir %.3lf>0,safe\n",dir(i));
					  }
					  else
					  {
						  t(i) = 0.0;
						  fprintf(stderr,"Dir %.3lf<0,STOPPING\n",dir(i));
					  }
				  }
			  }
		  }
		  return t;
	  }

	  Vector readFT()
	  {
		  Vector FTtmp = *datas;
		  FTtmp(1) = -FTtmp(1);
		  FTtmp(4) = -FTtmp(4);
		  FTtmp(5) = -FTtmp(5);
		  return FTtmp;
	  }
		  
};


class ft_ControlModule: public RFModule
{
private:
	Property Options;
	PolyDriver *dd;
	ftControl *ft_control;
	BufferedPort<Vector>* port_FT;
    int mod_count;
	string handlerPortName;
    Port handlerPort;      //a port to handle messages 
public:
	ft_ControlModule()
	{
		dd         = 0;
		ft_control = 0;
		mod_count  = 0;
	}

	virtual bool createDriver(PolyDriver *_dd)
	{
	    if(!dd || !(dd->isValid()))
		{
			fprintf(stderr,"It is not possible to instantiate the device driver\nreturning...");
			return 0;
		}

		IPositionControl *pos;
		IEncoders *encs;
		IPidControl *pids;
		IVelocityControl *vels;
		IAmplifierControl *amps;

		bool ok = true;
		ok = ok & dd->view(pos);
		ok = ok & dd->view(vels);
		ok = ok & dd->view(encs);
		ok = ok & dd->view(pids);
		ok = ok & dd->view(amps);
		if(!ok)
		{
			fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
			return false;
		}

		int jnts;
		pos->getAxes(&jnts);

		int i;
		for (i = 0; i < jnts; i++) {
			amps->enableAmp(i);
			pids->enablePid(i);
		}
		return true;
	}

	bool respond(const Bottle& command, Bottle& reply) 
	{
	  string helpMessage =  string(getName().c_str()) + 
							" commands are: \n" +  
							"help        to display this message\n" + 
							"set zfc     to set the zfc behaviour \n" + 
							"set imp     to set the impedance behaviour) \n";

	  reply.clear(); 

	   if (command.get(0).asString()=="help")
	   {
		  cout << helpMessage;
		  reply.addString(helpMessage.c_str());
	   }
   	   /*else if (command.get(0).asString()=="quit")
	   {
		   reply.addString("quitting");
		   return false;     
	   }*/
	   else if (command.get(0).asString()=="set")
	   {
		  //int jnt = command.get(2).asInt(); 
		  if (command.get(1).asString()=="imp")
		  {
		     control_mode= IMPEDANCE;
			 reply.addString("ok, setting impedance mode");
			 ft_control->setDesiredPositions();
		  }
		  else if (command.get(1).asString()=="zfc")
		  {
			 control_mode= ZEROFORCECONTROL;
			 reply.addString("ok, setting zero force control mode");
		  }
	   }
	   return true;
	}
	bool configure(ResourceFinder &rf)
	{
		string PortName;
		string part;
		string robot;
		string fwdSlash = "/";
		PortName = fwdSlash;
		port_FT= 0;

		ConstString robotName=rf.find("robot").asString();
		if (rf.check("robot"))
		{
			PortName=fwdSlash+rf.find("robot").asString().c_str();
			robot = rf.find("robot").asString().c_str();
		}
        else
		{
			fprintf(stderr,"Device not found\n");
            PortName=fwdSlash+"icub";
			robot = "icub";
		}
		
		ConstString partName=rf.find("part").asString();
		if (rf.check("part"))
		{
			PortName=PortName+fwdSlash+rf.find("part").asString().c_str();
			part = rf.find("part").asString().c_str();
		}
        else
		{
			fprintf(stderr,"Could not find part in the config file\n");
            return false;
		}

		// Create the terminal
		handlerPortName = "/zfcTerminal/";
		handlerPortName += partName;
		if (!handlerPort.open(handlerPortName.c_str())) {           
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
		}
		attach(handlerPort);                  // attach to port
		attachTerminal();                     // attach to terminal
		Options.put("robot",robot.c_str());
		Options.put("part",part.c_str());
		Options.put("device","remote_controlboard");
		Options.put("local",((fwdSlash+robot+fwdSlash+part)+"/ftControl/client").c_str());
		Options.put("remote",(fwdSlash+robot+fwdSlash+part).c_str());

		dd = new PolyDriver(Options);
		if(!createDriver(dd)) 
		{
			fprintf(stderr,"ERROR: unable to create device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");
		
		port_FT=new BufferedPort<Vector>;
		port_FT->open((PortName+"/FT:i").c_str());
		fprintf(stderr,"input port opened...\n");
		ft_control = new ftControl(SAMPLER_RATE, dd, port_FT, rf, part);
		fprintf(stderr,"ft thread istantiated...\n");
		ft_control->start();
		fprintf(stderr,"thread started\n");
		return true;
	}

	
	double getPeriod()	{ return 1; }
	bool updateModule() { 
		mod_count++;
        	//fprintf(stderr,"[%d] updateModule... ",mod_count);
		return true; 
		}
	
	
	bool close()
	{
		fprintf(stderr,"closing...don't know why :S \n");
		handlerPort.close();
		if (ft_control) ft_control->stop();
		if (ft_control) {delete ft_control; ft_control=0;}
		if (dd) {delete dd; dd=0;}
		if (port_FT) {delete port_FT; port_FT=0;}
		//port_FT.interrupt();
		//port_FT.close();
		return true;
	}
};



int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;
	
    //create your module
    ft_ControlModule* ft_controlmodule = new ft_ControlModule;

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose();
	rf.setDefaultContext("zeroForceControl/conf");
	rf.setDefaultConfigFile("defautFT.ini");

    rf.configure("ICUB_ROOT", argc, argv);

	if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
		cout << "\t--context   context: where to find the called resource (referred to $ICUB_ROOT\\app: default zeroForceControl\\conf)"                << endl;
        cout << "\t--from      from: The name of the file.ini to be used for calibration"          << endl;
        return 0;
    }

    cout<<"Configure module..."<<endl;
    bool ret = ft_controlmodule->configure(rf);
    
	if (ret)
	{
		cout<<"Start module..."<<endl;
		ft_controlmodule->runModule();
	}

    cout<<"Main returning..."<<endl;
	if (ft_controlmodule) ft_controlmodule->close();
	delete ft_controlmodule;
	ft_controlmodule=0;



	// Connect the ports so that anything written from /out arrives to /in
		  

    return 0;
}

/************************************************************************/
iCubArm4DOF::iCubArm4DOF()
{
    _allocate_limb("right");
}


/************************************************************************/
iCubArm4DOF::iCubArm4DOF(const string &_type)
{
    _allocate_limb(_type);
}


/************************************************************************/
iCubArm4DOF::iCubArm4DOF(const iCubArm4DOF &arm)
{
    _copy_limb(arm);
}


/************************************************************************/
void iCubArm4DOF::_allocate_limb(const string &_type)
{
    iKinLimb::_allocate_limb(_type);

    H0.zero();
    H0(0,1)=-1;
    H0(1,2)=-1;
    H0(2,0)=1;
    H0(3,3)=1;

    linkList.resize(8);
  //  linkList.resize(7);

    if (type=="right")
    {
        linkList[0]=new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2]=new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3]=new iKinLink(       0.0, -0.10774,  M_PI/2.0,         -M_PI/2.0, -95.5*M_PI/180.0,   0.0*M_PI/180.0);
        linkList[4]=new iKinLink(       0.0,      0.0, -M_PI/2.0,         -M_PI/2.0,              0.0, 160.8*M_PI/180.0);
        linkList[5]=new iKinLink(       0.0, -0.15228, -M_PI/2.0, -105.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6]=new iKinLink(     0.015,      0.0,  M_PI/2.0,               0.0,   0.0*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7]=new iKinLink(       0.0,  -0.1373,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
        }//
    else
    {

		linkList[0]=new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2]=new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3]=new iKinLink(       0.0,  0.10774, -M_PI/2.0,          M_PI/2.0, -95.5*M_PI/180.0,   5.0*M_PI/180.0);
        linkList[4]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0,              0.0, 160.8*M_PI/180.0);
        linkList[5]=new iKinLink(       0.0,  0.15228, -M_PI/2.0,   75.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6]=new iKinLink(    -0.015,      0.0,  M_PI/2.0,               0.0,   5.5*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7]=new iKinLink(       0.0,   0.1373,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);

	}//

    for (unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);
    blockLink(7,0.0);//
}




//*******************************************************/
//ICUB LEG 4 DOF


/************************************************************************/
iCubLeg4DOF::iCubLeg4DOF()
{
    _allocate_limb("right");
}


/************************************************************************/
iCubLeg4DOF::iCubLeg4DOF(const string &_type)
{
    _allocate_limb(_type);
}


/************************************************************************/
iCubLeg4DOF::iCubLeg4DOF(const iCubLeg4DOF &leg)
{
    _copy_limb(leg);
}


/************************************************************************/
void iCubLeg4DOF::_allocate_limb(const string &_type)
{
    iKinLimb::_allocate_limb(_type);

    H0.zero();
    H0(0,0)=1;
    H0(1,2)=1;    
    H0(2,1)=-1;
    H0(2,3)=-0.1199;
    H0(3,3)=1;

    linkList.resize(6);

    if (type=="right")
    {
        H0(1,3)=0.0681;

        linkList[0]=new iKinLink(   0.0,     0.0,  M_PI/2.0,  M_PI/2.0,  -44.0*M_PI/180.0, 132.0*M_PI/180.0);
        linkList[1]=new iKinLink(   0.0,     0.0,  M_PI/2.0,  M_PI/2.0, -119.0*M_PI/180.0,  17.0*M_PI	/180.0);
        linkList[2]=new iKinLink(   0.0,  0.2236, -M_PI/2.0, -M_PI/2.0,  -79.0*M_PI/180.0,  79.0*M_PI/180.0);
        linkList[3]=new iKinLink(-0.213,     0.0,      M_PI,  M_PI/2.0, -125.0*M_PI/180.0,  23.0*M_PI/180.0);
        linkList[4]=new iKinLink(   0.0,     0.0,  M_PI/2.0,       0.0,  -42.0*M_PI/180.0,  21.0*M_PI/180.0);
        linkList[5]=new iKinLink(-0.041,     0.0,      M_PI,       0.0,  -24.0*M_PI/180.0,  24.0*M_PI/180.0);
    }
    else
    {
        H0(1,3)=-0.0681;

        linkList[0]=new iKinLink(   0.0,     0.0, -M_PI/2.0,  M_PI/2.0,  -44.0*M_PI/180.0, 132.0*M_PI/180.0);
        linkList[1]=new iKinLink(   0.0,     0.0, -M_PI/2.0,  M_PI/2.0, -119.0*M_PI/180.0,  17.0*M_PI/180.0);
        linkList[2]=new iKinLink(   0.0, -0.2236,  M_PI/2.0, -M_PI/2.0,  -79.0*M_PI/180.0,  79.0*M_PI/180.0);
        linkList[3]=new iKinLink(-0.213,     0.0,      M_PI,  M_PI/2.0, -125.0*M_PI/180.0,  23.0*M_PI/180.0);
        linkList[4]=new iKinLink(   0.0,     0.0, -M_PI/2.0,       0.0,  -42.0*M_PI/180.0,  21.0*M_PI/180.0);
        linkList[5]=new iKinLink(-0.041,     0.0,       0.0,       0.0,  -24.0*M_PI/180.0,  24.0*M_PI/180.0);
    }

    for (unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

	blockLink(4,0.0);
	blockLink(5,0.0);
}

