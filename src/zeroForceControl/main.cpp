/**
@ingroup icub_module

\defgroup zeroForceControl zeroForceControl
 
Perform zero force control on the iCub limbs. 
Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 27/05/2010 

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module performs zero force control and joint impedance control 
of the iCub limbs. No model based compensation of the 6-axis force/torque 
(FT) sensor's measurements is done. FT are acquired through an input YARP 
port.
The intrinsic offsets of the sensors are defined by the first FT data. 
Internal dynamic has not yet been compensated through model based 
compensation of the force/torque measurements. 

\section lib_sec Libraries 
- YARP libraries. 
- ctrlLib library. 
- iKin library.
- iDyn library.  

\section parameters_sec Parameters

--name \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix <name>/. If not
  specified \e /zeroForceControl is assumed.
 
--context
- The parameter \e context identifies the location of the configuration files,
  referred to the path $ICUB_ROOT/app

--from
- The parameter \e from identifies the configuration files, located in the context
  directory, specific for the part to use.
  
--robot
- The parameter \e robot identifies the robot that is used. This parameter defines the
  prefix of the ports of the device. As default \e icub is used.

--part  
- The parameter \e part identifies the part of the robot which is used. All the opened 
  ports will deal with the part which is defined. the default value is \e left_arm

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e <name>/<part>/FT:i (e.g. /zfc/right_arm/FT:i) receives the input data 
  vector.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
-leftArmFT.ini
-rightArmFT.ini
-leftLegFT.ini
-rightLegFT.ini
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
zeroForceControl --name zfc --context zeroForceControl/conf --from rightArmFT.ini  
\endcode 
 
the module will create the listening port /zfc/right_arm/FT:i for 
the acquisition of data vector coming for istance from the right arm analog port. 
 
Try now the following: 
 
\code 
yarp connect /icub/right_arm/analog:o /zfc/right_arm/FT:i
\endcode 
 
\author Matteo Fumagalli

This file can be edited at src/zeroForceControl/main.cpp.
*/ 


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

bool prntData = false;
bool verbose = false;
const int CPRNT = 100;
const int CALIBRATION_OK = true; // should be true when FT calibration will be ok
enum {ZEROFORCECONTROL=0, IMPEDANCE=1};
int   control_mode=ZEROFORCECONTROL;
bool  filter_enabled=true;
enum{FILT1HZ = 1, FILT2HZ = 2, FILT3HZ = 3};
int filter_order = FILT3HZ;

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

using namespace iKin;
using namespace iDyn;

/**
*
* A class for defining the 4-DOF iCub Arm (3 of the shoulder and one for the elbow. The end effector is placed on the wrist)
*/
class iCubArm4DOF : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type);

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
    virtual void allocate(const std::string &_type);

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

FILE* fid;
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
	Vector Fe;
	int count;
	iKinLimb *iCubLimb;
	iKinChain *chain;
	
	int limbJnt;
	string limb;

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
	
	Vector sat;
	Matrix K;
	Matrix Kspring;
    Vector kp;
	Vector kspr;
	double gain;

	Vector *datas;



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
		  T(1,2) = -R1/K1*a; 
		  T(2,2) = R2/K2*a;
		  T = 0.056*T;

		  for(int i=0;i<3;i++)
			  for(int j=0;j<3;j++)
				  T_all(i,j) = T(i,j);
		  if(limbJnt>3)
		  {
		  for(int i=3;i<limbJnt;i++)
			  for(int j=3;j<limbJnt;j++)
				  T_all(i,j) = 1.0;
		  }
	}

	void initLimb(string limb)
	{
		if (strcmp(limb.c_str(), "left_arm")==0)
		  {			  
		      //---------------------------------------------
			  //           SETTING UP PART TO CONTROL
		      //---------------------------------------------
			  limbJnt = 4;
			  sensorLink = 2;
			  shoulderCouplingInit();
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubArm4DOF("left");
              Rs(0,0) = Rs(2,1) = 1.0;  Rs(1,2) = -1.0;
              ps(1) = 0.10;
		      //---------------------------------------------
			  //           SETTING GAINS
		      //---------------------------------------------
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.15;
			  if (filter_enabled)
			  {
				  switch(filter_order)
				  {
				  case FILT1HZ:
					  kp(0) = -62.0;	kp(1) = -50.0;	kp(2) = -55.0;	kp(3) = -100; 
					  break;
				  case FILT2HZ:
					  kp(0) = -62.0;	kp(1) = -50.0;	kp(2) = -55.0;	kp(3) = -100; 
					  break;
				  case FILT3HZ:
					  kp(0) = -62.0;	kp(1) = -50.0;	kp(2) = -55.0;	kp(3) = -100; 
					  break;
				  default:
					  kp(0) = -25.0;	kp(1) = -25.0;	kp(2) = -25.0;	kp(3) = -30.0;
					  break;
				  }
			  }
			  else{ kp(0) = -25.0;	kp(1) = -25.0;	kp(2) = -25.0;	kp(3) = -30.0;}
              fprintf(stderr, "Opening left arm ... \n");
		  }
		  else if (strcmp(limb.c_str(), "right_arm")==0)
		  {  
		      //---------------------------------------------
			  //           SETTING UP PART TO CONTROL
		      //---------------------------------------------
			  limbJnt = 4;
			  sensorLink = 2;
			  shoulderCouplingInit();
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubArm4DOF("right");
              Rs(0,0) = -1.0; Rs(2,1) = 1.0;  Rs(1,2) = 1.0;
              ps(1) = -0.10;
		      //---------------------------------------------
			  //           SETTING GAINS
		      //---------------------------------------------
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.30;	kspr(3) = 0.15;
			  if (filter_enabled)
			  {
				kp(0) =  62.0;	kp(1) =  50.0;	kp(2) =  55.0;	kp(3) =  100;
				switch(filter_order)
				  {
				  case FILT1HZ:
					  kp(0) =  62.0;	kp(1) =  50.0;	kp(2) =  55.0;	kp(3) =  100;
					  break;
				  case FILT2HZ:
					  kp(0) =  62.0;	kp(1) =  50.0;	kp(2) =  55.0;	kp(3) =  100;
					  break;
				  case FILT3HZ:
					  kp(0) =  62.0;	kp(1) =  50.0;	kp(2) =  55.0;	kp(3) =  100;
					  break;
				  default:
					  kp(0) =  25.0;	kp(1) =  25.0;	kp(2) =  25.0;	kp(3) =  30.0;
					  break;
				  }
			  }
			  else{ kp(0) =  25.0;	kp(1) =  25.0;	kp(2) =  25.0;	kp(3) =  30.0; }
              fprintf(stderr, "Opening right arm ... \n");
		  } 
		  else if (strcmp(limb.c_str(), "left_leg")==0)
		  {
			  
		      //---------------------------------------------
			  //           SETTING UP PART TO CONTROL
		      //---------------------------------------------
			  limbJnt = 4;
			  sensorLink = 1;
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubLeg4DOF("left");
              Rs(1,0) = -1.0; Rs(0,1) = -1.0;  Rs(2,2) = -1.0;
              ps(2) = -0.10;
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
			  
		      //---------------------------------------------
			  //           SETTING GAINS
		      //---------------------------------------------
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.3;
			  if (filter_enabled)
			  {
				kp(0) =  20.0;	kp(1) =  -14.0;	kp(2) =  94.0;	kp(3) =  -94.0;
				switch(filter_order)
				  {
				  case FILT1HZ:
					  kp(0) =  20.0;	kp(1) =  -14.0;	kp(2) =  94.0;	kp(3) =  -94.0;
					  break;
				  case FILT2HZ:
					  kp(0) =  20.0;	kp(1) =  -14.0;	kp(2) =  94.0;	kp(3) =  -94.0;
					  break;
				  case FILT3HZ:
					  kp(0) =  20.0;	kp(1) =  -14.0;	kp(2) =  94.0;	kp(3) =  -94.0;
					  break;
				  default:
					  kp(0) =  10;	kp(1) =  -5;	kp(2) =  40;	kp(3) =  -50; 
					  break;
				  }

			  }
			  else{ kp(0) =  10;	kp(1) =  -5;	kp(2) =  40;	kp(3) =  -50;}
			  T_all=eye(limbJnt,limbJnt);
			  fprintf(stderr, "Opening left leg ... \n");
		  }
		  else if (strcmp(limb.c_str(), "right_leg")==0)
		  {
			  
		      //---------------------------------------------
			  //           SETTING UP PART TO CONTROL
		      //---------------------------------------------
			  limbJnt = 4;
			  sensorLink = 1;
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubLeg4DOF("right");
              Rs(1,0) = -1.0; Rs(0,1) = 1.0;  Rs(2,2) = 1.0;
              ps(2) = 0.10;
			  kp.resize(limbJnt);
			  kspr.resize(limbJnt);
		      //---------------------------------------------


			  
		      //---------------------------------------------
			  //           SETTING GAINS
		      //---------------------------------------------
			  kspr(0) = 0.3;	kspr(1) = 0.3;	kspr(2) = 0.3;	kspr(3) = 0.3;
			  if (filter_enabled)
			  {
				kp(0) = -20.0;	kp(1) =  14.0;	kp(2) =  -94.0;	kp(3) =  94.0; 
				switch(filter_order)
				  {
				  case FILT1HZ:
					  kp(0) = -20.0;	kp(1) =  14.0;	kp(2) =  -94.0;	kp(3) =  94.0; 
					  break;
				  case FILT2HZ:
					  kp(0) = -20.0;	kp(1) =  14.0;	kp(2) =  -94.0;	kp(3) =  94.0; 
					  break;
				  case FILT3HZ:
					  kp(0) = -20.0;	kp(1) =  14.0;	kp(2) =  -94.0;	kp(3) =  94.0; 
					  break;
				  default:
					  kp(0) =  -10;	kp(1) =   5;	kp(2) =  -40;	kp(3) =  50; 
					  break;
				  }
			  }
			  else{ kp(0) =  -10;	kp(1) =   5;	kp(2) =  -40;	kp(3) =  50;}
			  T_all=eye(limbJnt,limbJnt);
			  fprintf(stderr, "Opening right leg ... \n");
		  }
		  else fprintf(stderr, "nothing will be opened ... \n");
		      //---------------------------------------------
	}

    
public:
	void setDesiredPositions()			
	{
		for (int i=0; i<limbJnt; i++)
			desPosition(i)=encoders(i);
	}

	ftControl(int _rate, PolyDriver *_dd, BufferedPort<Vector> *_port_FT, ResourceFinder &_rf, string tmplimb):	  
	  RateThread(_rate), dd(_dd) 
	  {
		  
		  //------------------------------------------
		  //             STAT VARIABLES
		  //------------------------------------------
		  watchDog = 0;
		  time = time0 = 0.0;
		  countTime = countTime0 = 0;
          if(prntData) fid=fopen("impedanceData.dat","a+");
		  first = true;
		  //------------------------------------------



		  //------------------------------------------
		  //           FT SENSOR DEFAULT
		  //------------------------------------------
		  Rs.resize(3,3);     Rs=0.0;
		  ps.resize(3);		  ps=0.0;
          kp=0.0;
		  kspr=0.0;
		  //------------------------------------------
		  
		  


		  //------------------------------------------
		  //           DEVICES
		  //------------------------------------------
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
		  limb = tmplimb;
		  initLimb(limb);
		  chain = iCubLimb->asChain();
		  iCubLimb->setAllConstraints(false);
		  //------------------------------------------




		  
		  //------------------------------------------
		  //             ICUB PID
		  //------------------------------------------
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
		  //------------------------------------------






		  //------------------------------------------
		  //         VARIABLES FOR CONTROL
		  //------------------------------------------
		  FT.resize(FT_VALUES);
		  FTs.resize(FT_VALUES);
		  FTs_init.resize(FT_VALUES);
		  FTj.resize(limbJnt);
		  Fe.resize(FT_VALUES);
		  FT.zero();
		  FTs.zero();
		  FTj.zero();

		  angs.resize(limbJnt);
		  angs_old.resize(limbJnt);
		  speeds.resize(limbJnt);
		  angs=0.0;
		  angs_old=0.0;
		  speeds=0.0;
		  K=eye(limbJnt,limbJnt);
		  Kspring=eye(limbJnt,limbJnt);
		  tau.resize(limbJnt);
		  tauDes.resize(limbJnt);
		  tauSafe.resize(limbJnt);
		  tauFilt.resize(limbJnt);
		  tau=0.0;
		  tauDes=0.0;
		  tauSafe=0.0;
		  tauFilt=0.0;
          desPosition.resize(limbJnt);
		  //------------------------------------------





		  
		  //------------------------------------------
		  //           FT SENSOR
		  //------------------------------------------
		  sensor = new iFTransform(Rs,ps);

		  FTB = new iFB(sensorLink);
		  FTB->attach(chain);
		  FTB->attach(sensor);
		  //------------------------------------------






		  //------------------------------------------
		  //             SATURATION
		  //------------------------------------------ 
		  Bottle tmp;
		  tmp = 0;
		  sat.resize(4);
		  if(_rf.check("saturation"))
		  {
			  tmp = _rf.findGroup("saturation");
			  Vector sat_tmp(tmp.size()-1);
			  if(sat_tmp.length()<(tmp.size()-1))
			  {
				  fprintf(stderr,"warning: check saturation length in config file.");
				  for(int i = 0;i<(tmp.size()-1);i++)
				  {
					  sat(i) = tmp.get(i+1).asDouble();
				  }
				  for(int i = (tmp.size()-1);i<sat.length();i++)
					  sat(i) = 0.0;
			  }
			  else
			  {
				  for(int i = 0;i<sat.length();i++)
				  {
					  sat(i) = tmp.get(i+1).asDouble();
				  }
			  }
			  
		  }
		  //-------------------------------------------


		  //------------------------------------------
		  //             GAIN
		  //------------------------------------------ 
		  tmp = 0;
		  if(_rf.check("gain"))
		  {
			  tmp = _rf.findGroup("gain");
			  gain = tmp.get(1).asDouble();
			  if(gain>1.0) 
			  {
				  fprintf(stderr,"cannot use gains greater than 1.0. Setting gain = 0.7\n");
				  gain = 0.7;
			  }
		  }
		  else  gain = 0.5;
		  //-------------------------------------------
	  }


	  bool threadInit()
	  {
		  
		  count = 0;

		  //------------------------------------------
		  //       START POSITION CHECK
		  //------------------------------------------
#ifdef CONTROL_ON		  
		  for(int i=0;i<limbJnt;i++)
			  ipos->positionMove(i,initPosition[i]);
#endif			  
		  
		  bool check = false;
		  for(int i=0;i<limbJnt;i++)
		  {
			  check=false;
			  count=0;
			  while(!check && count < 50)
			  {
				  ipos->checkMotionDone(i,&check);
				  count++;
				  Time::delay(0.1);
			  }
		  }
		  //------------------------------------------
		  



		  
		  //------------------------------------------
		  //         SETTING PID FOR CONTROL
		  //------------------------------------------
#ifdef CONTROL_ON
		  // Set the Pids to zero in order to control using setOffset		  
		  for(int i=0;i<limbJnt;i++)
			  	  ipids->setPid(i,FTPid[i]);  
#endif		  
		  //------------------------------------------
		  count =0;
		  return true;
	  }


	  void run()
	  {
		  //------------------------------------------
		  //          STATISTIC VARIABLES
		  //------------------------------------------
		  static double told=0;
		  static double t=0;
		  told=t;
		  t=Time::now();
          double tdiff=t-told;
		  //------------------------------------------



		  //------------------------------------------
		  //          ROBOT POSTURE
		  //------------------------------------------
		  iencs->getEncoders(encoders.data());
		  for(int i=0; i<limbJnt;i++)
		  {
			  angs_old(i) = angs(i);
			  angs(i) = encoders(i)*M_PI/180;
		  }

		  iCubLimb->setAng(angs);
		  //------------------------------------------



		  
		  //------------------------------------------
		  //     FT PORT CHECK AND DATA RETRIEVING
		  //------------------------------------------
		  datas=port_FT->read(false);
		  port_FT->getEnvelope(info);
		  countTime = info.getCount();

		  int connected = CONNECTION_OK;

		  if(countTime0>=32000 && countTime<16000)  countTime0=0;
		  if(countTime - countTime0 > 0) 
		  {
			  connected = CONNECTION_OK;
			  countTime0 = countTime;
			  watchDog = 0;
		  }
		  else watchDog+=1;
		  if(watchDog>=20)connected = CONNECTION_ERROR;
		  if(count>=CPRNT)
		  {
			  fprintf(stderr,"Cycle duration=%f s, watchdog:%d ",tdiff, watchDog);
			  if (verbose) fprintf(stderr,"c:%d c0:%d", countTime, countTime0);
			  fprintf(stderr,"\n");
			  if (connected == CONNECTION_ERROR) fprintf(stderr,"Connection error with the sensor\n");
#ifdef CONTROL_ON
			  fprintf(stderr,"Control ON:");
			  if (control_mode==IMPEDANCE) fprintf(stderr,"Impedance\n");
			  else fprintf(stderr,"Zero Force Control\n");
#else
			  fprintf(stderr,"Debug Mode, CONTROL_ON macro is not defined \n",tdiff);
			  if (control_mode==IMPEDANCE) fprintf(stderr," (impedance off)\n");
			  else fprintf(stderr,"(Zero Force Control off)\n");
#endif
		  }
		  //------------------------------------------


		  
		  //------------------------------------------
		  //   FORCE ACQUISITION AND TRANSFORMATION
		  //------------------------------------------
		  if(datas!=0)  
		  { 
			  FTs = *datas;
			  if(first) 
			  {
				  FTs_init = FTs;
				  setDesiredPositions();	
				  control_mode=IMPEDANCE;
			  }
			  FT = FTB->getFB(FTs-FTs_init);			  	
			  first = false;
		  }
		  else
		  {
			  if(!first)  FT = FTB->getFB();
			  else {FT=0.0; FTs=0.0; FTj=0.0; FTs_init=0.0;}
		  }
		  Fe=FTB->getFe();
		  //------------------------------------------




		  //------------------------------------------
		  //          DEFINING CONTROL OUTPUT
		  //------------------------------------------
		  for(int i=0;i<limbJnt;i++) {K(i,i) = kp(i); Kspring(i,i) = kspr(i);}
		  Matrix J = iCubLimb->GeoJacobian();
		  Vector tau(limbJnt);
		  tau=0.0;
		  tauDes=Kspring*((180.0/M_PI)*angs-desPosition);
		  FTj=J.transposed()*FT;
		  if (control_mode==IMPEDANCE)
			{tau = K*(FTj-tauDes);} //USE THIS FOR IMPEDANCE CONTROL
		  else 
			{tau = K*(FTj);}        //USE THIS FOR ZERO FORCE CONTROL
		  Vector tauC= T_all*tau;
		  //------------------------------------------




		  //------------------------------------------
		  //              FILTERING
		  //------------------------------------------
		  for(int i=0;i<limbJnt;i++)
			  {
				  tauFilt(i) = 0.0;//lpf_ord1_3hz(tauC(i), i);
				  switch(filter_order)
				  {
				  case FILT1HZ:
					  tauFilt(i) = lpf_ord1_1hz(tauC(i), i);
					  break;
				  case FILT2HZ:
					  tauFilt(i) = lpf_ord1_2hz(tauC(i), i);
					  break;
				  case FILT3HZ:
					  tauFilt(i) = lpf_ord1_3hz(tauC(i), i);
					  break;
				  default:
					  tauFilt(i) = lpf_ord1_3hz(tauC(i), i);
					  break;
				  }
			  }
		  if (filter_enabled)
		  {
			  for(int i=0;i<limbJnt;i++)
			  {
				  tauC(i) = tauFilt(i);
			  }
		  }
		  //--------------------------------------------



		  //-------------------------------------------
		  //             LIMIT CHECK
		  //-------------------------------------------

		  for (int i=0; i<limbJnt; i++)
		  {
			  speeds(i) = (angs(i) - angs_old(i))/tdiff;
		  }
		  tauSafe = checkLimits(encoders, gain * tauC, speeds);
		  //--------------------------------------------



		  //--------------------------------------------
		  //                SATURATION
		  //--------------------------------------------

		  for(int i=0;i<limbJnt;i++)
		  {
			  tauSafe(i)=(tauSafe(i)>sat(i))?sat(i):tauSafe(i);
			  tauSafe(i)=(tauSafe(i)<-sat(i))?-sat(i):tauSafe(i);
		  }
		  //--------------------------------------------



		  //--------------------------------------------
		  //            COMMANDING TORQUES
		  //--------------------------------------------
#ifdef CONTROL_ON
		  //Set the control offsets to the motors
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipids->setOffset(i,tauSafe(i));
		  }	
#endif		  
		  //--------------------------------------------




		  //----------------------------------------------
		  //               PRINT
		  //----------------------------------------------
          if(prntData)
		  {
			  for(int i=0;i<limbJnt;i++)
					   fprintf(fid,"%.4lf\t",encoders(i));
			  for(int i=0;i<limbJnt;i++)
					   fprintf(fid,"%.4lf\t",tauDes(i));
			  for(int i=0;i<limbJnt;i++)
					   fprintf(fid,"%.4lf\t",FTj(i));
			  for(int i=0;i<limbJnt;i++)
					   fprintf(fid,"%.4lf\t",tauSafe(i));
			  for(int i=0;i<6;i++)
					   fprintf(fid,"%.4lf\t",FTs(i)-FTs_init(i));
			  fprintf(fid,"\n");
          }
	 

		  if(count>=CPRNT)
		  {
			  if (verbose)
			  {fprintf(stderr,"FT = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", FT(i));
			  fprintf(stderr,"\n");}

			  //if (verbose)
			  {fprintf(stderr,"FTs = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", FTs(i)-FTs_init(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"FTe = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", Fe(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"FTj = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", FTj(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"tauD = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tauDes(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"encs = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", angs(i)*180/M_PI);
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"encd = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", desPosition(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"spds = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", speeds(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"tau  = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tau(i));
			  fprintf(stderr,"\n");}

			  if (verbose)
			  {fprintf(stderr,"tauF = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tauFilt(i));
			  fprintf(stderr,"\n");}

			  //if (verbose)
			  {fprintf(stderr,"tauM = ");
			  for(int i=0;i<limbJnt;i++)
				  fprintf(stderr,"%+.3lf\t", tauSafe(i));
			  fprintf(stderr,"\n");}
			  
			  fprintf(stderr,"\n\n");
			 
			  count = 0;
		  }
		  //---------------------------------------------
		  count++;
	  }

	  void threadRelease()
	  {

		  //---------------------------------------------
		  //            DISABLING
		  //---------------------------------------------
		  fprintf(stderr,"disabling amps...\n");
		  for(int i=0;i<limbJnt;i++)
          	  iamps->disableAmp(i);
		  fprintf(stderr,"disabling pids...\n");
		  for(int i=0;i<limbJnt;i++)
          	  ipids->disablePid(i);
		  fprintf(stderr,"setting old PIDS...\n");
		  //---------------------------------------------


		  
		  //---------------------------------------------
		  //         SETTING BACK ICUB PID
		  //---------------------------------------------
#ifdef CONTROL_ON
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipids->setPid(i,iCubPid[i]);
			  fprintf(stderr,"setting back iCub PID(%d)\n",i);
		  }
#endif
		  //---------------------------------------------


		  
		  //---------------------------------------------
		  //       ENABLING THE ROBOT
		  //---------------------------------------------
		  fprintf(stderr,"enabling amps...\n");
		  for(int i=0;i<limbJnt;i++)
          	  iamps->enableAmp(i);
		  fprintf(stderr,"enabling pids...\n");
		  for(int i=0;i<limbJnt;i++)
          	  ipids->enablePid(i);
		  //---------------------------------------------


		  
		  //---------------------------------------------
		  //       DESTROING VARIABLES
		  //---------------------------------------------
          if(prntData) fclose(fid);
		  if(FTB) delete FTB;
		  if(iCubPid) delete[] iCubPid;
		  if(FTPid) delete[] FTPid;
		  //---------------------------------------------
	  }

	  Vector checkLimits(Vector q, Vector TAO, Vector dir)
	  {
		  Vector t = TAO;
		  for(int i=0;i<limbJnt;i++)
		  {
			  if(q(i)<=minJntLimits[i])
			  {
                  if(count>=CPRNT)  fprintf(stderr,"J%d over limits %.2lf (%.2lf ; %.2lf) ", i, q(i), minJntLimits[i], maxJntLimits[i]);
				  if (dir(i)>0){ if(count>=CPRNT) fprintf(stderr,"Dir %.3lf > 0.0,safe\n",dir(i));}
				  else
				  {
					  t(i) = 0.0;
					  if(count>=CPRNT) fprintf(stderr,"Dir %.3lf < 0.0,STOPPING\n",dir(i));
				  }			  
			  }
			  if(q(i)>=maxJntLimits[i])
			  {
                  if(count>=CPRNT) fprintf(stderr,"J%d over limits %.2lf (%.2lf ; %.2lf) ", i, q(i), minJntLimits[i], maxJntLimits[i]);
				  if (dir(i)<0){ if(count>=CPRNT) fprintf(stderr,"Dir %.3lf > 0.0, safe\n",dir(i));}
				  else
				  {
					  t(i) = 0.0;
                      if(count>=CPRNT) fprintf(stderr,"Dir %.3lf < 0.0, STOPPING\n",dir(i));
				  }
			  }
		  }
		  return t;
	  }

	  bool setInitialPosition(Vector initPos)
	  {
		  if(initPos.length()<limbJnt)
			  return false;
		  initPosition.resize(initPos.length());
		  for(int i = 0;i<limbJnt;i++)
			  initPosition(i) =	initPos(i);
		  fprintf(stderr,"initial position set to: %.2lf, %.2lf, %.2lf, %.2lf\n", initPosition(0), initPosition(1), initPosition(2), initPosition(3));
		  return true;
	  }

	  bool setLimits(Vector maxLimit, Vector minLimit)
	  {
		  if(maxLimit.length()<limbJnt || minLimit.length()<limbJnt) return false;

		  maxJntLimits.resize(maxLimit.length());
		  minJntLimits.resize(minLimit.length());
		  for(int i = 0;i<limbJnt;i++)
		  {
			  maxJntLimits(i) = maxLimit(i);
			  minJntLimits(i) = minLimit(i);
		  }
		  fprintf(stderr,"max limits set to: %.2lf, %.2lf, %.2lf, %.2lf\n", maxJntLimits(0), maxJntLimits(1), maxJntLimits(2), maxJntLimits(3));
		  fprintf(stderr,"max limits set to: %.2lf, %.2lf, %.2lf, %.2lf\n", minJntLimits(0), minJntLimits(1), minJntLimits(2), minJntLimits(3));
		  return true;
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
							"verbose     to display debug variables\n" + 
							"set zfc     to set the zfc behaviour \n" + 
							"set imp     to set the impedance behaviour) \n";

	  reply.clear(); 

	   if (command.get(0).asString()=="help")
	   {
		  cout << helpMessage;
		  reply.addString(helpMessage.c_str());
	   }
   	   else if (command.get(0).asString()=="verbose")
	   {
		  if (verbose) reply.addString("setting verbose mode OFF");
		  else  reply.addString("ok, setting verbose mode ON");
		  verbose = !(verbose);
	   }
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
		PortName = "/zfc/";
		port_FT= 0;

		ConstString robotName=rf.find("robot").asString();
		if (rf.check("robot"))
		{
			//PortName=PortName+rf.find("robot").asString().c_str();
			robot = rf.find("robot").asString().c_str();
		}
        else
		{
			fprintf(stderr,"Device not found\n");
            //PortName=PortName+"icub";
			robot = "icub";
		}
		
		ConstString partName=rf.find("part").asString();
		if (rf.check("part"))
		{
			PortName=PortName+rf.find("part").asString().c_str();
			part = rf.find("part").asString().c_str();
		}
        else
		{
			fprintf(stderr,"Could not find part in the config file\n");
		  Time::delay(3.1);
            return false;
		}
		//port_FT=new BufferedPort<Vector>;
		//port_FT->open((PortName+"/FT:i").c_str());
		Bottle tmp;
		tmp=0;
		Vector initPos;
		if(rf.check("initPosition"))
		{
		  tmp = rf.findGroup("initPosition");
		  initPos.resize(tmp.size()-1);
		  for(int i = 0;i<initPos.length();i++)
		  {
			  initPos(i) = tmp.get(i+1).asDouble();
		  }
		}
		else 
		{
		  fprintf(stderr,"error: initial position not defined in configuration file!!!\n returning;");
		  Time::delay(3.0);
		  return false;
		}
		tmp=0;
		Vector maxLim;
		if(rf.check("MAX_LIMITS"))
		{
		  tmp = rf.findGroup("MAX_LIMITS");
		  maxLim.resize(tmp.size()-1);
		  for(int i = 0;i<maxLim.length();i++)
		  {
			  maxLim(i) = tmp.get(i+1).asDouble();
		  }
		}
		else 
		{
		  fprintf(stderr,"error: max limits not defined in configuration file!!!\n returning;");
		  Time::delay(3.0);
		  return false;
		}

		tmp=0;
		Vector minLim;
		if(rf.check("MIN_LIMITS"))
		{
		  tmp = rf.findGroup("MIN_LIMITS");
		  minLim.resize(tmp.size()-1);
		  for(int i = 0;i<minLim.length();i++)
		  {
			  minLim(i) = tmp.get(i+1).asDouble();
		  }
		}
		else 
		{
		  fprintf(stderr,"error: min limits not defined in configuration file!!!\n returning;");
		  Time::delay(3.0);
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
		fprintf(stderr,"setting initial position\n");
		ft_control->setInitialPosition(initPos);
		fprintf(stderr,"setting limits\n");
		ft_control->setLimits(maxLim,minLim);
		fprintf(stderr,"initial position and limits set...\n");

        fprintf(stderr,"starting thread\n");
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
	rf.setDefaultConfigFile("leftArmFT.ini");

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
    allocate("right");
}


/************************************************************************/
iCubArm4DOF::iCubArm4DOF(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
iCubArm4DOF::iCubArm4DOF(const iCubArm4DOF &arm)
{
    clone(arm);
}


/************************************************************************/
void iCubArm4DOF::allocate(const string &_type)
{
    iKinLimb::allocate(_type);

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
    allocate("right");
}


/************************************************************************/
iCubLeg4DOF::iCubLeg4DOF(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
iCubLeg4DOF::iCubLeg4DOF(const iCubLeg4DOF &leg)
{
    clone(leg);
}


/************************************************************************/
void iCubLeg4DOF::allocate(const string &_type)
{
    iKinLimb::allocate(_type);

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

