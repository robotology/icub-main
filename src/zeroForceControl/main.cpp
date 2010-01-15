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
#include <string.h>

#include "iCub/iFC.h"
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


const int SAMPLER_RATE = 20;
const int FT_VALUES = 6;
const int ARM_JNT = 4;

const bool verbose = false;
const int CPRNT = 100;
const int CALIBRATION_OK = true; // should be true when FT calibration will be ok

const double initPosition[4] = {-10.0, 20.0, 15.0, 15.0};

const double MAX_JNT_LIMITS[4] = {2.0, 120.0, 90.0, 95.0};
const double MIN_JNT_LIMITS[4] = {-95.0, 0.0, -20.0, 10.0};

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
	//iCubArm *arm;
	iCubArm4DOF *arm;
	iKinChain *chain;

	iFB *FTB;
	iFTransform *sensor;

	Matrix Rs;
	Vector ps;

	double R0, R1, R2, K0, K1, K2, Jm0, Jm1, Jm2, a;
	Matrix T, T_all;

	BufferedPort<Vector> *port_FT;
	Vector Datum;
	bool first;

	Vector tau;
	Vector tauSafe;

	int watchDOG;
	Stamp info;
	double time, time0;
	int countTime, countTime0;

	Vector *datas;
    Vector kp;
    

public:
	ftControl(int _rate, PolyDriver *_dd, BufferedPort<Vector> *_port_FT, ResourceFinder &_rf, string limb):	  
	  RateThread(_rate), dd(_dd) 
	  {
		  port_FT = _port_FT;
		  dd->view(ipos);
		  dd->view(ivels);
		  dd->view(iencs);
		  dd->view(ipids);
		  dd->view(iamps);

		  int nJnt;
		  iencs->getAxes(&nJnt);
		  encoders.resize(nJnt);
		  //fprintf(stderr, "number of joints: %d", nJnt);
		  iencs->getEncoders(encoders.data());


		  // Elbow to FT sensor variables
		  Rs.resize(3,3);     Rs=0.0;
		  ps.resize(3);		  ps=0.0;
          kp.resize(4);       kp=0.0;


		  //Shoulder motors parameters:
		  R0 = 0.8967; K0 = 0.05; Jm0 = 8.47E-6;
		  R1 = R2 = 0.8363; K1 = K2 = 0.0280; Jm1 = Jm2 = 5.15E-6;
		  a = 40.0/65.0;
		  T.resize(3,3);
		  T_all.resize(4,4);
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
		  T_all(3,3) = 1.0; //added elbow

		  //arm= new iCubArm("left");
          if (strcmp(limb.c_str(), "left_arm")==0)
              {
                  Rs(0,0) = Rs(2,1) = 1.0;  Rs(1,2) = -1.0;
                  ps(1) = 0.10;
                  kp(0) = -25;	kp(1) = -25;	kp(2) = -25;	kp(3) = -50;
                  arm= new iCubArm4DOF("left");
                  fprintf(stderr, "Opening left arm ... \n");

              }
          else
              if (strcmp(limb.c_str(), "right_arm")==0)
                  {
                      Rs(0,0) = -1.0; Rs(2,1) = 1.0;  Rs(1,2) = 1.0;
                      ps(1) = -0.10;
                      kp(0) =  25;	kp(1) =  25;	kp(2) =  25;	kp(3) =  50;
                      arm= new iCubArm4DOF("right");
                      fprintf(stderr, "Opening right arm ... \n");
                  }
              else
                  {
                      fprintf(stderr, "nothing will be opened ... \n");
                  }

		  sensor = new iFTransform(Rs,ps);
		  chain = arm->asChain();
		  arm->setAllConstraints(false);

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
			  ipids->getPid(i,iCubPid+i);
			  // Set the Pids for force control mode:
			  ipids->getPid(i,FTPid+i);
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
		  
		  for(int i=0;i<ARM_JNT;i++)
		  {
			  ipos->positionMove(i,initPosition[i]);
		  }
			  

		  bool check = false;
		  //int count;
		  for(int i=0;i<ARM_JNT;i++)
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

		  for(int i=0;i<ARM_JNT;i++)
		  {
			  ipids->setPid(i,FTPid[i]);  // iCub is now controllable using setOffset
			  //ipids->setPid(i,FTPid[i]);  // iCub is now controllable using setOffset
		  }
		  
		  count =0;
		  return true;
	  }
	  void run()
	  {
		  datas=port_FT->read(false);
		  
		  //if(iencs->getEncoders(encoders.data()))
		  iencs->getEncoders(encoders.data());
		  //fprintf(stderr,"encoders length = %d\n", encoders.length());
		  Vector angs(4);
		  for(int i=0; i<4;i++)
			  angs(i) = encoders(i)*M_PI/180;

		  arm->setAng(angs);
		  //else if(verbose) fprintf(stderr,"ERROR: no read from encoders\n");

		  port_FT->getEnvelope(info);
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
			  //if (verbose) fprintf(stderr,"WARNING: possible connection problem. watchdog:%d\n\n",watchDOG);
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
						  FTs_init=0.0;
					  }
				  }
				 // break;
			  
		  //}
				  
		  Vector Fe=FTB->getFe();

		  //GAINS: to be tuned
		  Matrix K;
		  K=eye(ARM_JNT,ARM_JNT);
		  for(int i=0;i<4;i++) K(i,i) = kp(i);

		  //CONTROL: to be checked
		  Matrix J = arm->GeoJacobian();
		  Vector tau = K*(J.transposed())*FT;
		  tauSafe = T_all*tau;
		  tauSafe = checkLimits(encoders, T_all*tau); /**/
		  const int sat=50;
		  for(int i=0;i<ARM_JNT;i++)
		  {
			  tauSafe(i)=(tauSafe(i)>sat)?sat:tauSafe(i);
			  tauSafe(i)=(tauSafe(i)<-sat)?-sat:tauSafe(i);
		  }

		  for(int i=0;i<4;i++)
		  {
			  ipids->setOffset(i,tauSafe(i));
		  }
		  

		  

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

			  fprintf(stderr,"encs = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%+.3lf\t", angs(i)*180/M_PI);
			  fprintf(stderr,"\n");
			  
			  fprintf(stderr,"tau = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%+.3lf\t", tau(i));
			  fprintf(stderr,"\n");

			  fprintf(stderr,"tauM = ");
			  for(int i=0;i<4;i++)
				  fprintf(stderr,"%+.3lf\t", tauSafe(i));
			  fprintf(stderr,"\n");

              fprintf(stderr,"J:\n %s \n", J.toString().c_str());
			  fprintf(stderr,"\n\n\n");
			  fprintf(stderr,"\n\n");



			  count = 0;
		  }
		  count++;
	  }

	  void threadRelease()
	  {
		  fprintf(stderr,"disabling amps...\n");
		  for(int i=0;i<ARM_JNT;i++)
          	  iamps->disableAmp(i);
		  fprintf(stderr,"disabling pids...\n");
		  for(int i=0;i<ARM_JNT;i++)
          	  ipids->disablePid(i);
		  fprintf(stderr,"setting old PIDS...\n");
		  for(int i=0;i<ARM_JNT;i++)
			  ipids->setPid(i,iCubPid[i]);
		  fprintf(stderr,"enabling amps...\n");
		  for(int i=0;i<ARM_JNT;i++)
          	  iamps->enableAmp(i);
		  fprintf(stderr,"enabling pids...\n");
		  for(int i=0;i<ARM_JNT;i++)
          	  ipids->enablePid(i);

	//	  if(datas) delete datas;
		  if(FTB) delete FTB;
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
public:
	ft_ControlModule()
	{
		dd         = 0;
		ft_control = 0;
		mod_count  = 0;
	}

	virtual bool createDriver(PolyDriver *_dd)
	{
		/*********************************************/
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
		/***********************************************/
		return true;
	}

	bool configure(ResourceFinder &rf)
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
			fprintf(stderr,"Could not find part in the config file\n");
            return false;
		}

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
    ft_controlmodule->configure(rf);
    cout<<"Start module..."<<endl;
    ft_controlmodule->runModule();

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


