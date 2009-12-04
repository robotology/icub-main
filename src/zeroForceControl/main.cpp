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
//#include <yarp/dev/GenericSensorInterfaces.h>

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

#define c0_1000 0
#define c1000_0 1

const int SAMPLER_RATE = 50;
const int FT_VALUES = 6;
const int ARM_JNT = 4;

const double initPosition[4] = {0.0, 0.0, 0.0, 0.0};

const double MAX_JNT_LIMITS[4] = {0.0, 0.0, 0.0, 0.0};
const double MIN_JNT_LIMITS[4] = {0.0, 0.0, 0.0, 0.0};

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
	iCubArm *arm;
	iKinChain *chain;

	iFB *FTB;
	iFTransform *sensore;

	Matrix Rs;
	Vector ps;

	BufferedPort<Vector> port_FT;
	Vector Datum;
	bool first;

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
		  Rs=eye(3,3);
		  ps.resize(3,1);
		  ps=0.0;

		  arm= new iCubArm("Left");
		  sensore = new iFTransform(Rs,ps);
		  chain = arm->asChain();

		  FTB = new iFB(2);
		  FTB->attach(arm);
		  FTB->attach(sensore);

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
	  }
	  bool threadInit()
	  {
		  FT.zero();
		  FTs.zero();
		  count = 0;

		  for(int i=0;i<ARM_JNT;i++)
			  ipos->positionMove(i,initPosition[i]);

		  bool check = false;
		  while(!check)
		  {
			  check=true;
			  for(int i=0;i<ARM_JNT;i++)
				  check &= checkSinglePosition(initPosition[i],encoders(i));
		  }
		  Time::delay(1.0);

		  for(int i=0;i<ARM_JNT;i++)
			  ipids->setPid(i,FTPid[i]);  // iCub is now controllable using setOffset
		  return true;
	  }
	  void run()
	  {
		  Vector *tmp=port_FT.read(false);
		  iencs->getEncoders(encoders.data());
		  arm->setAng(encoders);
		
		  if(tmp!=0)  
		  {
			  Vector Datum = *tmp;
			  FTs = *Datum.data();
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

		

		  double k=1.0; //to be tuned
		  Matrix K;
		  K=k*eye(ARM_JNT,ARM_JNT);

		  Vector tao = checkLimits(encoders,K*arm->GeoJacobian(encoders)*FT); 

		  fprintf(stderr,"tao = ");
		  for(int i=0;i<4;i++)
			  fprintf(stderr,"%.3lf\t", tao(i));
		  fprintf(stderr,"\n\n\n");


	  }

	  void threadRelease()
	  {
		  for(int i=0;i<ARM_JNT;i++)
			  ipids->disablePid(i);
		  for(int i=0;i<ARM_JNT;i++)
			  ipids->resetPid(i);
			
		  delete sensore;
		  delete FTB;
		  delete arm;
	  }

	  bool checkSinglePosition(double qd, double q)
	  {
		  if(abs(qd-q)<=1.0) return true;
		  else return false;
	  }
	  Vector checkLimits(Vector q, Vector TAO)
	  {
		  Vector t = TAO;
		  for(int i=0;i<ARM_JNT;i++)
		  {
			  if((q(i)<=MIN_JNT_LIMITS[i]) || (q(i)>=MAX_JNT_LIMITS[i])) t(i) = 0.0;
		  }
		  return t;
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
			return 0;
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
            PortName=fwdSlash+"iCub";
			robot = "iCub";
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
		if(!createDriver(dd)) return 0;

		port_FT.open(("/" + PortName+"/FT:i").c_str());
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




