 /**
   * @ingroup icub_module
   *
   * \defgroup zeroForceControl zeroForceControl
   *
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
#include <string>

#include "iFC.h"
#include "iCub/iKinFwd.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
//using namespace ctrl;
using namespace std;

using namespace iFC;
using namespace iKin;


const int SAMPLER_RATE = 50;
const int FT_VALUES = 6;
const int ARM_JNT = 4;

const bool verbose = 1;
const int CPRNT = 10;
const int CALIBRATION_OK = false; // should be true when FT calibration will be ok

const double initPosition[4] = {0.0, 0.0, 0.0, 0.0};

const double MAX_JNT_LIMITS[4] = {2.0, 120.0, 90.0, 90.0};
const double MIN_JNT_LIMITS[4] = {-85.0, 0.0, -20.0, 10.0};

#define CONNECTION_ERROR 0
#define CONNECTION_OK	 1


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
	Pid iCubPid[ARM_JNT]; //needed when the program stops
	Pid FTPid[ARM_JNT]; // to be set to zero

	Vector encoders;

	Vector FTs;
	Vector FTs_init;
	Vector FT;
	int count;
	iCubArm4DOF *arm;
	iKinChain *chain;

	iFB *FTB;
	iFTransform *sensor;

	Matrix Rs;
	Vector ps;

	BufferedPort<Vector> port_FT;
	Vector Datum;
	bool first;

	Vector tau;
	Vector tauSafe;

	int watchDOG;
	Stamp info;
	double time, time0;
	int countTime, countTime0;

	Vector *datas;
    

public:
	ftControl(int _rate, PolyDriver *_dd, BufferedPort<Vector> &_port_FT, ResourceFinder &_rf):	  
	  RateThread(_rate), dd(_dd)
	  {
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
		  Rs.resize(3,3);
		  Rs=0.0;
		  Rs(0,0) = Rs(2,1) = 1.0;
		  Rs(1,2) = -1.0;
		  ps.resize(3);
		  ps=0.0;
		  ps(1) = 0.10;

		  arm= new iCubArm4DOF("Left");
		  sensor = new iFTransform(Rs,ps);
		  chain = arm->asChain();

		  FTB = new iFB(2);
		  FTB->attach(arm);
		  FTB->attach(sensor);

		  first = true;
		  FTs.resize(FT_VALUES);
		  FTs_init.resize(FT_VALUES);
		  FT.resize(FT_VALUES);

		  for(int i=0;i<ARM_JNT;i++)
		  {
			  // Get a copy of iCub Pid values...
			  ipids->getPids(&iCubPid[i]);
			  // Set the Pids for force control mode:
			  ipids->getPids(&FTPid[i]);
			  FTPid[i].setKd(0.0);
			  FTPid[i].setKp(0.0);
			  FTPid[i].setKi(0.0);	
			  FTPid[i].setOffset(0.0);	
			  // Setting the FTPid, iCub is controllable using setOffset
		  }
		  tau.resize(4);
		  tauSafe.resize(4);
		  tau=0.0;
		  tauSafe=0.0;

		  watchDOG = 0;
		  time = time0 = 0.0;
		  countTime = countTime0 = 0;
	  }
	  bool threadInit()
	  {
		  FT.zero();
		  FTs.zero();
		  count = 0;

		  /* Decommentare...
		  
		  for(int i=0;i<ARM_JNT;i++)
			  ipos->positionMove(i,initPosition[i]);

		  bool check = false;
		  //int count;
		  for(int i=0;i<ARM_JNT;i++)
		  {
			  check=false;
			  count=0;
			  while(!check && count < 100)
			  {
				  ipos->checkMotionDone(i,&check);
				  count++;
				  Time::delay(0.1);
			  }
		  }
		  */
		  
		  Time::delay(1.0);

		  for(int i=0;i<ARM_JNT;i++)
		  {
			  //ipids->setPid(i,FTPid[i]);  // iCub is now controllable using setOffset
		  }
		  
		  count =0;
		  return true;
	  }
	  void run()
	  {
		  datas=port_FT.read(false);
		  
		  if(iencs->getEncoders(encoders.data()))
		          arm->setAng(encoders);
		  else if(verbose) fprintf(stderr,"ERROR: no read from encoders\n");

		  port_FT.getEnvelope(info);
		  //time = info.getTime();
		  countTime = info.getCount();

		  int connected = CONNECTION_OK;

		  if(countTime - countTime0 > 0.0) 
		  {
			  connected = CONNECTION_OK;
			  countTime0 = countTime;
			  watchDOG = 0;
		  }
		  else   
		  {
			  watchDOG+=1;
		  }

		  if(watchDOG>=20) 
		  {
			  connected = CONNECTION_ERROR;
			  if (verbose) fprintf(stderr,"WARNING: possible connection problem. watchdog:%d\n\n",watchDOG);
		  }

		  switch(connected)
		  { 
			  case CONNECTION_ERROR:
				  FT=0.0;
				  fprintf(stderr,"ERROR: connection lost\n\n");
				  break;
			  case CONNECTION_OK:
				  if(count>=CPRNT && verbose)
					  fprintf(stderr,"Connection ok...\n\n");
				  if(datas!=0)  
				  { 
					  if(CALIBRATION_OK)
					  {
						  Vector Datum = *datas;
						  FTs = *Datum.data();
					  } else
						  FTs = readFT();


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
						  FTs_init=0.0;
					  }
				  }
				  break;
			  
		  }
		  

		  double k=1.0; //to be tuned
		  Matrix K;
		  K=k*eye(ARM_JNT,ARM_JNT);

		  Vector tau = K*(arm->GeoJacobian(encoders).transposed())*FT;
		  tauSafe = tau;
		  tauSafe = checkLimits(encoders, tau); 

		  if(count>=CPRNT)
		  {
			  fprintf(stderr,"tau = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%.3lf\t", tau(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"safeTau = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%.3lf\t", tauSafe(i));
			  fprintf(stderr,"\n\n\n");

			  count = 0;
		  }
		  count++;
	  }

	  void threadRelease()
	  {
		  for(int i=0;i<ARM_JNT;i++)
			  ipids->disablePid(i);
		  for(int i=0;i<ARM_JNT;i++)
			  ipids->resetPid(i);
			
		  delete sensor;
		  delete FTB;
		  delete arm;
	  }

	  /*bool checkSinglePosition(double qd, double q)
	  {
		  if(abs(qd-q)<=1.0) return true;
		  else return false;
	  }*/
	  Vector checkLimits(Vector q, Vector TAO)
	  {
		  Vector t = TAO;
		  for(int i=0;i<ARM_JNT;i++)
		  {
			  if((q(i)<=MIN_JNT_LIMITS[i]) || (q(i)>=MAX_JNT_LIMITS[i]))
			  {
				  t(i) = 0.0;
				  if(verbose) fprintf(stderr,"Joint %d over limits %.2lf (%.2lf - %.2lf)\n", i, q(i), MIN_JNT_LIMITS[i], MAX_JNT_LIMITS[i]);
			  }
		  }
		  return t;
	  }

	  Vector readFT()
	  {
		  Vector Datum = *datas;
		  Vector FTtmp = *Datum.data();
		  FTtmp(2) = -FTtmp(2);
		  FTtmp(4) = -FTtmp(4);
		  return FTtmp;
	  }
		  
};


class ft_ControlModule: public RFModule
{
private:
	Property Options;
	ftControl *ft_control;
	BufferedPort<Vector> port_FT;
public:
	ft_ControlModule()
	{
	}

	bool createDriver(PolyDriver &_dd)
	{
		if(!(_dd).isValid())
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
		ok = ok & _dd.view(pos);
		ok = ok & _dd.view(vels);
		ok = ok & _dd.view(encs);
		ok = ok & _dd.view(pids);
		ok = ok & _dd.view(amps);
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
	virtual bool configure(ResourceFinder &rf)
	{
		string PortName;
		string part;
		string robot;
		string fwdSlash = "/";
		PortName = fwdSlash;

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
			fprintf(stderr,"Device not found\n");
            PortName=PortName+"/left_arm";
			part = "left_arm";
		}

		Options.put("robot",robot.c_str());
		Options.put("part",part.c_str());
		Options.put("device","remote_controlboard");
		Options.put("local",((fwdSlash+robot)+"/ftControl/client").c_str());
		Options.put("remote",(fwdSlash+robot+fwdSlash+part).c_str());

		PolyDriver dd(Options);
		if(!createDriver(dd)) 
		{
			fprintf(stderr,"ERROR: unable to create device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");

		port_FT.open((PortName+"/FT:i").c_str());
		ft_control = new ftControl(SAMPLER_RATE, &dd, port_FT, rf);
		ft_control->start();
		return true;
	}

	
	virtual double getPeriod()	{ return 1.0; }
	virtual bool updateModule() { return true; }
	
	
	virtual bool close()
	{
		ft_control->stop();
		port_FT.interrupt();
		port_FT.close();

		delete ft_control;
		return true;
	}
};



int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose();
	rf.setDefaultContext("ftSensorCalibration/conf");
	rf.setDefaultConfigFile("leftArmFTCal.ini");

    rf.configure("ICUB_ROOT", argc, argv);

	if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
		cout << "\t--context   context: where to find the called resource (referred to $ICUB_ROOT\\app: default FTSensorCalibration\\conf)"                << endl;
        cout << "\t--from      from: The name of the file.ini to be used for calibration"          << endl;
        return 0;
    }

    //create your module
    ft_ControlModule ft_controlmodule;

    cout<<"Configure module..."<<endl;
    ft_controlmodule.configure(rf);
    cout<<"Start module..."<<endl;
    ft_controlmodule.runModule();

    cout<<"Main returning..."<<endl;
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
        }
    else
    {
        linkList[0]=new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2]=new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3]=new iKinLink(       0.0,  0.10774, -M_PI/2.0,          M_PI/2.0, -95.5*M_PI/180.0,   0.0*M_PI/180.0);
        linkList[4]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0,              0.0, 160.8*M_PI/180.0);
        linkList[5]=new iKinLink(       0.0,  0.15228, -M_PI/2.0,   75.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6]=new iKinLink(    -0.015,      0.0,  M_PI/2.0,               0.0,   0.0*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7]=new iKinLink(       0.0,   0.1373,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
        }

    for (unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);
    blockLink(7,0.0);
}


