/**
* @ingroup icub_module
*
* \defgroup ftIdSweep ftIdSweep
*
*
* Perform sweep PWM over one motor.
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

const int SWEEP_TIME = 4000; //time of sweep@w 

bool verbose = false;
const int CPRNT = 100;

#define CONNECTION_ERROR 0
#define CONNECTION_OK	 1

#define SWEEP_OFF	 0
#define SWEEP_ON	 1

#define CONTROL_ON  1

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
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

FILE* fid = fopen("ftSweepData.dat","a+");
// class dataCollector: class for reading from Vrow and providing for FT on an output port
class ftSweep: public RateThread
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
	Pid *FTPid; //needed when the program stops
	Pid *SweepPid; // to be set to zero

	Vector encoders;
	Vector angs;
	Vector initPosition;

	Vector maxJntLimits;
	Vector minJntLimits;

	Vector FTs;
	Vector FTs_init;
	Vector FT;
	Vector FTj;
	int count;

	iKinLimb *iCubLimb;
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
	bool sweep;

	int ctrlJnt;
	Vector tau;
	Vector tauDes;
	Vector tauSafe;

	int watchDog;
	Stamp info;
	double time, time0;
	int countTime, countTime0, sweepCount;
	double wt,f,w0,A;

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

	
	void initLimb(string limb)
	{
		if (strcmp(limb.c_str(), "left_arm")==0)
		  {			  
			  limbJnt = 4;
			  sensorLink = 2;
			  shoulderCouplingInit();
			  iCubPid = new Pid[limbJnt];
			  FTPid = new Pid[limbJnt];
			  iCubLimb = new iCubArm("left");
			  iCubLimb = new iCubArm4DOF("left");
              Rs(0,0) = Rs(2,1) = 1.0;  Rs(1,2) = -1.0;
              ps(1) = 0.10;
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
			  T_all=eye(limbJnt,limbJnt);
			  fprintf(stderr, "Opening right leg ... \n");
		  }
		  else
          {
              fprintf(stderr, "nothing will be opened ... \n");
          }
		  //initializePositionAndLimits(limb);
	}
	
	  Vector checkLimits(Vector q, Vector TAO)
	  {
		  Vector t = TAO;
		  for(int i=0;i<limbJnt;i++)
		  {
			  if(q(i)<=minJntLimits[i])
			  {
                  t(i) = 0.0;
				  fprintf(stderr,"jnt %d is over inferior limit : %.3lf (%.3lf - %.3lf)\n", i, q(i), minJntLimits[i], maxJntLimits[i]);
					  
			  }
			  if(q(i)>=maxJntLimits[i])
			  {
				  t(i) = 0.0;
				  fprintf(stderr,"jnt %d is over inferior limit : %.3lf (%.3lf - %.3lf)\n", i, q(i), minJntLimits[i], maxJntLimits[i]);
			  }
		  }
		  return t;
	  }

public:
	

	ftSweep(int _rate, PolyDriver *_dd, BufferedPort<Vector> *_port_FT, ResourceFinder &_rf, string limb):	  
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
		  angs=0.0;

		  ctrlJnt = 3;
		  for(int i=0; i<limbJnt;i++)
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
		  tau=0.0;
		  tauDes=0.0;
		  tauSafe=0.0;
		  
		  watchDog = 0;
		  time = time0 = 0.0;
		  countTime = countTime0 = 0;
		  sweep = SWEEP_OFF;
		  sweepCount = 0;
		  wt=0;
		  f=1;
		  w0=0;
		  A=50;
	  }
	  bool threadInit()
	  {
		  FT.zero();
		  FTs.zero();
		  FTj.zero();
		  count = 0;
	  
		  // Put the robot in the starting position
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipos->positionMove(i,initPosition[i]);
		  }

		  bool check = false;
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
			fprintf(stderr,"Reached position %.1lf [deg] on jnt: %d\n",initPosition[i], i);
		  }
		  ipids->setPid(ctrlJnt,FTPid[ctrlJnt]);  
		  	  
		  Time::delay(2.0);
		  count =0;
		  return true;
	  }
		void run()
		{
			static double told=0;
			static double t=0;
			told=t;
			t=Time::now();
		
			datas=port_FT->read(false);

			//if(iencs->getEncoders(encoders.data()))
			iencs->getEncoders(encoders.data());
			//fprintf(stderr,"encoders length = %d\n", encoders.length());
			for(int i=0; i<limbJnt;i++)
			{
				angs(i) = encoders(i)*M_PI/180;
			}

			iCubLimb->setAng(angs);
			//else if(verbose) fprintf(stderr,"ERROR: no read from encoders\n");

			port_FT->getEnvelope(info);
			//time = info.getTime();
			countTime = info.getCount();

			int connected = CONNECTION_OK;

			if(countTime0>=32000 && countTime<16000)
			{
			  countTime0=0;
			}
			if(countTime - countTime0 > 0) 
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

		  if(count>=CPRNT)
		  {
			  if (verbose) {fprintf(stderr," s, watchdog:%d ", watchDog);
			  fprintf(stderr,"c:%d c0:%d", countTime, countTime0);
			  fprintf(stderr,"\n");}
			  if (connected == CONNECTION_ERROR) fprintf(stderr,"Connection error with the sensor\n");
		  }

		  if(datas!=0)  
		  { 
			FTs = *datas;
			if(first) 
			{
				FTs_init = FTs;
		  	    sweep=SWEEP_ON;
				for(int kk=0;kk<100;kk++)
					fprintf(stderr,"Sweep signal is starting now!!!\n");
			}
			FT = FTB->getFB(FTs-FTs_init);
			first = false;
		  }
		  else
	  	  {
	  	    if(!first)  FT = FTB->getFB();
		    else 
		    {
	  		  FT=0.0;
	  		  FTs=0.0;
	  		  FTj=0.0;
		  	  FTs_init=0.0;
		    }
		  }

				  
		  Vector Fe=FTB->getFe();
		  Vector FSweep=FTs-FTs_init;

		  Matrix J = iCubLimb->GeoJacobian();
		  Vector tau(4);
		  tau=0.0;
		  FTj=J.transposed()*FT;
		  if(sweep==SWEEP_ON)
		  {
			  fprintf(stderr,"sono in sweep!!!\n");
			  if(sweepCount<=SWEEP_TIME/SAMPLER_RATE)
			  {
				  sweepCount++;
				  wt=2*M_PI*f*sweepCount*SAMPLER_RATE/1000+w0;
				  fprintf(stderr,"sono qui!!!\n");
			  }else
			  {
				  sweepCount=1;
				  f=f+1;
				  w0=wt;
				  wt=2*M_PI*f*sweepCount*SAMPLER_RATE/1000+w0;
				  for(int kk=0;kk<100;kk++)
					  fprintf(stderr,"sono qui!!!\n");
			  }
			  tau=A*sin(wt);
		  }  else
		  {
			  fprintf(stderr,"sono fuori da sweep!!!\n");
			  tau=0.0;
		  }
		  fprintf(stderr,"f = %.2lf; \t wt = %.2lf; \t sweepCount = %.d; \t w0 = %.2lf;\n", f, wt, sweepCount, w0);
			  
		  tauSafe = checkLimits(encoders, tau); 
		  fprintf(fid, "%.3lf\t", tau);
		  fprintf(fid, "%.3lf\t%.3lf\t%.3lf\t%.3lf\t", FTj(0), FTj(1), FTj(2), FTj(3));
		  fprintf(fid, "%.3lf\t%.3lf\t%.3lf\t%.3lf\t%.3lf\t%.3lf\n", FSweep(0), FSweep(1), FSweep(2), FSweep(3), FSweep(4), FSweep(5));


		  //saturation
		  ipids->setOffset(ctrlJnt,tauSafe(ctrlJnt));
		  
		fprintf(stderr,"tau(%d) = %.3lf\n", ctrlJnt, tauSafe(ctrlJnt));

		  if(count>=CPRNT)
		  {
			  
			  if (verbose)
			  {fprintf(stderr,"FTs = ");
			  for(int i=0;i<6;i++)
				  fprintf(stderr,"%+.3lf\t", FTs(i)-FTs_init(i));
			  fprintf(stderr,"\n");}


			  count = 0;
		  }
		  count++;
		  fprintf(stderr,"counter: %d\n", count);
		  fprintf(stderr,"sweep: %d\n", sweep);
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

		  //set again the original saved PIDS to the ICub
		  for(int i=0;i<limbJnt;i++)
		  {
			  ipids->setPid(i,iCubPid[i]);
		  }
		  fprintf(stderr,"set Pid on jnt %d:\n", ctrlJnt);
		  Time::delay(3);
		  fprintf(stderr,"enabling amps...\n");
		  for(int i=0;i<limbJnt;i++)
          	  iamps->enableAmp(i);
		  fprintf(stderr,"enabling pids...\n");
		  for(int i=0;i<limbJnt;i++)
          	  ipids->enablePid(i);

		  if(FTB) delete FTB;
		  if(iCubPid) delete[] iCubPid;
		  if(FTPid) delete[] FTPid;
	  }


	  bool setInitialPosition(Vector initPos)
	  {
		  if(initPos.length()<limbJnt)
			  return false;

		  initPosition.resize(initPos.length());
		  for(int i = 0;i<limbJnt;i++)
			  initPosition(i) =	initPos(i);
		  return true;
	  }

	  bool setLimits(Vector maxLimit, Vector minLimit)
	  {
		  if(maxLimit.length()<limbJnt || minLimit.length()<limbJnt)
			  return false;

		  maxJntLimits.resize(maxLimit.length());
		  minJntLimits.resize(minLimit.length());
		  for(int i = 0;i<limbJnt;i++)
		  {
			  maxJntLimits(i) = maxLimit(i);
			  minJntLimits(i) = minLimit(i);
		  }
		  return true;
	  }

	  
    void setDesiredPositions()			
	{
//		for (int i=0; i<limbJnt; i++)
//			desPosition(i)=encoders(i);
	}
};


class ftSweepModule: public RFModule
{
private:
	Property Options;
	PolyDriver *dd;
	ftSweep *ft_sweep;
	BufferedPort<Vector>* port_FT;
    int mod_count;
	string handlerPortName;
    Port handlerPort;      //a port to handle messages 
public:
	ftSweepModule()
	{
		dd         = 0;
		ft_sweep = 0;
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
							"verbose     to display debug variables\n";

	  reply.clear(); 

	   if (command.get(0).asString()=="help")
	   {
		  cout << helpMessage;
		  reply.addString(helpMessage.c_str());
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

		Bottle tmp;
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
		  return false;
		}
		  

		// Create the terminal
		handlerPortName = "/ftIdTerminal/";
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
		Options.put("local",((fwdSlash+robot+fwdSlash+part)+"/ftIdSweep/client").c_str());
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
		ft_sweep = new ftSweep(SAMPLER_RATE, dd, port_FT, rf, part);
		fprintf(stderr,"id thread istantiated...\n");
		ft_sweep->setInitialPosition(initPos);
		ft_sweep->setLimits(maxLim,minLim);
		fprintf(stderr,"initial position and limits set...\n");
		ft_sweep->start();
		fprintf(stderr,"thread started successfully\n");
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
		if (ft_sweep) ft_sweep->stop();
		if (ft_sweep) {delete ft_sweep; ft_sweep=0;}
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
    ftSweepModule* ftsweepmodule = new ftSweepModule;

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose();
	rf.setDefaultContext("ftIdSweep/conf");
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
    bool ret = ftsweepmodule->configure(rf);
    
	if (ret)
	{
		cout<<"Start module..."<<endl;
		ftsweepmodule->runModule();
	}

    cout<<"Main returning..."<<endl;
	if (ftsweepmodule) ftsweepmodule->close();
	delete ftsweepmodule;
	ftsweepmodule=0;



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

