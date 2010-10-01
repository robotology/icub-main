/**
@ingroup icub_module

\defgroup demoForceControl demoForceControl
 
Starts zero force control and impedance control interface on the iCub limbs. 
Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 27/05/2010 

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This module starts torqueControl and joint impedance position control 
of the iCub limbs, using icub joint torque and impedance interface. Joint torques 
are estimated using dynamic model of the icub body (by the module "wholeBodyTorqueObserver" which is required
for torque control) exploiting 6-axis FT sensors.
The intrinsic offsets of the sensors are defined by the first FT data. 

<b>NOTE 1: before launching the demo (or before using any kind of force control interface), be sure that the firmware 
of the boards is greater (or equal) to the version 1.50 for BLL boards and 1.38 for MC4 boards. You can find instructions
to update the firmware of your robot in the page: http://eris.liralab.it/wiki/Firmware
</b>

<b>NOTE 2: before launching the application (or before using any kind of force control interface), be sure that the configuration
files of your robot support force control. To see whether your robot is properly configured, send an email to the rkhackers mailing list (robotcub-hackers@lists.sourceforge.net)</b>


\section lib_sec Libraries 
None.

\section dep_sec Dependencies
- wholeBodyTorqueObserver

\section parameters_sec Parameters

--robot
- The parameter \e robot identifies the robot that is used. This parameter defines the
  prefix of the ports of the device. As default \e icub is used.

--part  
- The parameter \e part identifies the part of the robot which is used. All the opened 
  ports will deal with the part which is defined. the default value is \e left_arm

--name
- The parameter \e name identifies the name of the ports which are opened. All the opened 
  ports will deal with the name which is defined. the default value is \e demoForceControl

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e /demoForceControl/<part>/rpc:i receives the input data to play with this module. (e.g. position commands, control mode commands...).
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching for example the following command: 
\code 
demoForceControl --robot icub --part left_arm
\endcode 
left arm joints can be controlled.

By typing:
\code 
yarp rpc /demoForceControl/left_arm/rpc:i
\endcode 
on an opened shell, it is possible to send commands to the module

\section command_sec RPC commands
Commands on the rpc port are

1 - set/get

2 - cmod/pos/iimp/pid (control mode, position, impedance, pid)

3 - specifies the joint, while commands from 4 to end define the values;

4 and more - depend on the previous commands:
- cmp/cmip/cmt if command 1 is cmod (to set the control mode: cmp for position, cmip for impedance position, and cmt for torque mode)
e.g. set cmod 3 cmt (sets the modality of jnt 3 to "torque").
- value1 and value2 if command 2 is iimp. this command sets the stiffness and damping of the joint
e.g. set iimp 3 0.16 0.08
- value1 if command 2 is pos; this command sets the reference position for the specified joint
e.g. set pos 3 45
- value1 value2 value3 (+value4 value5 value6 value7) set respectively the kp, kd and ki of the torque regulator.
value4->value7 are the same of a normal pid for iCub position control.
\section intro_sec Description


\author Matteo Fumagalli

This file can be edited at main/src/modules/demoForceControl/main.cpp.
*/ 


#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>

#include <iostream>
#include <iomanip>
//#include <string>
#include <string>

#define VOCAB_HELP VOCAB4('h','e','l','p')
#define VOCAB_QUIT VOCAB4('q','u','i','t')

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace std;

class ftCommand
{
private:
	PolyDriver *dd;
	IControlMode *mode;
	IEncoders *ienc;
	IPositionControl *ipos;
	ITorqueControl *itqs;
	IImpedanceControl *iimp;
	IPidControl *ipid;
	string part;

	int ctrlJnt;
	int allJnt;
public:
	ftCommand(PolyDriver *_dd, string _part)
	{
		dd = _dd;
		part = _part;
		if( (part=="left_arm") || (part == "right_arm")) 
			ctrlJnt = 5;
		else 
			ctrlJnt = 4;


		bool dd_check;
		dd_check = true;
		dd_check &= dd->view(mode);
		dd_check &= dd->view(ienc);
		dd_check &= dd->view(itqs);
		dd_check &= dd->view(iimp);
		dd_check &= dd->view(ipid);
		dd_check &= dd->view(ipos);

		ienc->getAxes(&allJnt);
		for(int i=0; i<ctrlJnt; i++)
			mode->setPositionMode(i);
		initRobotDemo();
	}

	//init and closing functions:
	~ftCommand()
	{
		stopRobotDemo();
		fprintf(stderr,"here...");
		Bottle p;
		p.addVocab(VOCAB_QUIT);
		command(p);
	}
	
	bool initRobotDemo()
	{
		if( (part=="left_arm") || (part == "right_arm")) 
			ipos->positionMove(3,50.0);
		fprintf(stderr,"moving to start position...\n");
		Time::delay(3.0);
		for(int i=0; i<ctrlJnt; i++)
		{
			Bottle p;
			p.addVocab(VOCAB_SET);
			p.addVocab(VOCAB_IMPEDANCE);
			p.addInt(i);
			p.addDouble(0.08);
			p.addDouble(0.02);
			command(p);
		}
		for(int i=0; i<ctrlJnt; i++)
		{
			Bottle p;
			p.addVocab(VOCAB_SET);
			p.addVocab(VOCAB_CM_CONTROL_MODE);
			p.addInt(i);
			p.addVocab(VOCAB_CM_IMPEDANCE_POS);
			command(p);
		}
		return true;

	}
	bool stopRobotDemo()
	{
		int ctrlJnt = getCtrlJoints();
		for(int i=0; i<ctrlJnt; i++)
		{
			Bottle p;
			p.addVocab(VOCAB_SET);
			p.addVocab(VOCAB_IMPEDANCE);
			p.addInt(i);
			p.addDouble(0.0);
			p.addDouble(0.0);
			command(p);
		}
		for(int i=0; i<ctrlJnt; i++)
		{
			Bottle p;
			p.addVocab(VOCAB_SET);
			p.addVocab(VOCAB_CM_CONTROL_MODE);
			p.addInt(i);
			p.addVocab(VOCAB_CM_POSITION);
			command(p);
		}
		return true;

	}
	// set Functions:
	bool getFunctions(const Bottle &p)
	{
		bool ok = false;
		int cmd = p.get(1).asVocab();
		int checkMode;
		switch(cmd)
		{
		case VOCAB_CM_CONTROL_MODE:
			ok = getMode(p, &checkMode, true);
			break;
		case VOCAB_PID:
			ok = getPid(p);
			break;
		default:
			return false;
			break;
		}
		
		return ok;
	}
	bool setFunctions(const Bottle &p)
	{ 
		bool ok = false;
		int cmd = p.get(1).asVocab();
		switch(cmd)
		{
		case VOCAB_CM_CONTROL_MODE:
			ok = setMode(p);
			break;
		case VOCAB_POSITION_MOVE:
			ok = setPosition(p);
			break;
		case VOCAB_PID:
			ok = setPid(p);
			break;
		case VOCAB_IMPEDANCE:
			ok = setImpedance(p);
			break;
		default:
			return false;
			break;
		}
		
		return ok;

	}
	bool setMode(const Bottle &p)
	{
		int checkMode=0;
		int axis = p.get(2).asInt();
		int cmd = p.get(3).asVocab();
		Bottle res;
		res=0;
		switch(cmd)
		{
		case VOCAB_CM_IDLE:
			printf("not implemented\n");
			break;
		case VOCAB_CM_POSITION:
			mode->setPositionMode(axis);
			mode->getControlMode(axis,&checkMode);
			res.addVocab(checkMode);
			printf("mode set to: %s",res.toString().c_str());
			break;
		case VOCAB_CM_VELOCITY:
			mode->setVelocityMode(axis);
			mode->getControlMode(axis,&checkMode);
			res.addVocab(checkMode);
			printf("mode set to: %s",res.toString().c_str());
			break;
		case VOCAB_CM_OPENLOOP:
			mode->setOpenLoopMode(axis);
			mode->getControlMode(axis,&checkMode);
			res.addVocab(checkMode);
			printf("mode set to: %s",res.toString().c_str());
			break;
		case VOCAB_CM_IMPEDANCE_POS:
			mode->setImpedancePositionMode(axis);
			mode->getControlMode(axis,&checkMode);
			res.addVocab(checkMode);
			printf("mode set to: %s",res.toString().c_str());
			break;
		case VOCAB_CM_IMPEDANCE_VEL:
			mode->setImpedanceVelocityMode(axis);
			mode->getControlMode(axis,&checkMode);
			res.addVocab(checkMode);
			printf("mode set to: %s",res.toString().c_str());
			break;
		case VOCAB_CM_TORQUE:
			mode->setTorqueMode(axis);
			mode->getControlMode(axis,&checkMode);
			res.addVocab(checkMode);
			printf("mode set to: %s",res.toString().c_str());
			break;
		default:
			return false;
			break;
		}
		return true;
	}
	bool getMode(const Bottle &p, int *_checkMode, bool _verbose = false)
	{
		Bottle res;
		int axis = p.get(2).asInt();
		int ctrlMod;//=*_checkMode;
		mode->getControlMode(axis,&ctrlMod);
		res.addVocab(ctrlMod);
		(*_checkMode) = ctrlMod;
		if(_verbose) printf("mode set to: %s",res.toString().c_str());
		return true;
	}
	bool setPosition(const Bottle &p)
	{
		int checkMode;
		getMode(p,&checkMode);
		int axis = p.get(2).asInt();
		double angle = p.get(3).asDouble();
		switch(checkMode)
		{
		case VOCAB_CM_POSITION:
		case VOCAB_CM_VELOCITY:
		case VOCAB_CM_IMPEDANCE_POS:
		case VOCAB_CM_IMPEDANCE_VEL:
			ipos->positionMove(axis,angle);
			break;
		case VOCAB_CM_IDLE:
		case VOCAB_CM_TORQUE:
		case VOCAB_CM_OPENLOOP:
			fprintf(stderr,"wrong modality for position command: message not sent!\n");
			break;
		default:
			fprintf(stderr,"joint %d is in unknown modality\n", axis);
		}
		return true;
	}
	void PrintPid(const Pid &_pid)
	{
		printf("kp %.2f ", _pid.kp);
		printf("kd %.2f ", _pid.kd);
		printf("ki %.2f ", _pid.ki);
		printf("maxi %.2f ", _pid.max_int);
		printf("maxo %.2f ", _pid.max_output);
		printf("off %.2f ", _pid.offset);
		printf("scale %.2f ", _pid.scale);
		printf("\n");
	}
	void setPidValue(const Bottle &p, Pid *_pid)
	{
		int checkMode;
		getMode(p,&checkMode);
		int axis = p.get(2).asInt();
		if(p.size()>=6)
		{
			(*_pid).setKp(p.get(3).asInt());
			(*_pid).setKd(p.get(4).asInt());
			(*_pid).setKi(p.get(5).asInt());
		}
		else if(p.size()>=10)
		{
			(*_pid).setMaxInt(p.get(6).asInt());
			(*_pid).setMaxOut(p.get(7).asInt());
			(*_pid).setScale(p.get(8).asInt());
			(*_pid).setOffset(p.get(9).asInt());
		}
		else fprintf(stderr,"wrong pid size!!!");

		switch(checkMode)
		{
		case VOCAB_CM_POSITION:
		case VOCAB_CM_VELOCITY:
			ipid->setPid(axis,(*_pid));
		case VOCAB_CM_IMPEDANCE_POS:
		case VOCAB_CM_IMPEDANCE_VEL:
		case VOCAB_CM_TORQUE:
			itqs->setTorquePid(axis,(*_pid));
			break;
		case VOCAB_CM_IDLE:
		case VOCAB_CM_OPENLOOP:
			fprintf(stderr,"wrong modality for pid command: message not sent!\n");
			break;
		default:
			fprintf(stderr,"joint %d is in unknown modality\n", axis);
		}
	}
	bool getPid(const Bottle &p, Pid *_pid)
	{
		int checkMode;
		getMode(p,&checkMode);
		int axis = p.get(2).asInt();
		Pid pid;
		Bottle res;
		switch(checkMode)
		{
		case VOCAB_CM_POSITION:
		case VOCAB_CM_VELOCITY:
			ipid->getPid(axis,&pid);
			break;
		case VOCAB_CM_IMPEDANCE_POS:
		case VOCAB_CM_IMPEDANCE_VEL:
		case VOCAB_CM_TORQUE:
			itqs->getTorquePid(axis,&pid);
			break;
		case VOCAB_CM_IDLE:
		case VOCAB_CM_OPENLOOP:
			fprintf(stderr,"wrong modality for pid command: message not sent!\n");
			break;
		default:
			fprintf(stderr,"joint %d is in unknown modality\n", axis);
		}
		(*_pid) = pid;
		return true;
	}
	bool getPid(const Bottle &p)
	{
		Pid pid;
		getPid(p,&pid);
		PrintPid(pid);
		return true;
	}
	bool setPid(const Bottle &p)
	{
		int checkMode;
		getMode(p,&checkMode);
		int axis = p.get(2).asInt();
		Pid pid;
		getPid(p,&pid);
		fprintf(stderr,"setting PID values for joint %d \n",axis);
		printf("from %s: ", Vocab::decode(VOCAB_PID).c_str());
		PrintPid(pid);
		setPidValue(p, &pid);
		printf("to %s: ", Vocab::decode(VOCAB_PID).c_str());
		PrintPid(pid);

		
		return true;
	}
	bool setImpedance(const Bottle &p)
	{
		int checkMode;
		getMode(p,&checkMode);
		int axis = p.get(2).asInt();
		double stiffness =  p.get(3).asDouble();
		double dumping =  p.get(4).asDouble();
		iimp->setImpedance(axis,stiffness,dumping,0.0);		
		return true;
	}
	//bool setTorquePid(int _jnt, double _kp, double _kd, double _ki)
	//{
	//	Pid pid;
	//	pid.setKp(_kp);
	//	pid.setKd(_kd);
	//	_ki = 0;
	//	pid.setKi(_ki);
	//	itqs->setTorquePid(_jnt, pid);
	//	itqs->getTorquePid(_jnt,&pid);
	//	
	//	fprintf(stderr,"Pid of joint %d set to: %.1lf\n", _jnt, 0.3);
	//	return true;
	//}
	//bool setImpedanceGains(int _jnt, double _k, double _d, double _o)
	//{
	//	iimp->setImpedance(_jnt,_k,_d,_o);
	//	return true;
	//}
	//get Functions:

	// cmd function:
	bool command(const Bottle &p)
	{
        Bottle response;
        bool ok=false;
        bool rec=false;

		switch(p.get(0).asVocab()) {      
        case VOCAB_HELP:printf("\n\n");
            printf("Available commands:\n\n");
            break;
		case VOCAB_QUIT:
			stopRobotDemo();            
            break;
		case VOCAB_SET:
			setFunctions(p);
			break;
		case VOCAB_GET:
			getFunctions(p);
			break;
		default:
			printf("error!!!\n");
		}
		return true;
	}

	int getCtrlJoints(){return ctrlJnt;}
};

class ft_ControlModule: public RFModule
{
private:
	Property Options;
	PolyDriver *dd;
	ftCommand *ft_control;
	//string handlerPortName;
    Port rpcPort;      //a port to handle messages 
	
public:
	ft_ControlModule()
	{
		ft_control = 0;
		dd = 0;
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
		Bottle cmd;
		reply.clear(); 
		ft_control->command(command);
		
		return true;
	}
	bool configure(ResourceFinder &rf)
	{
		string PortName;
		string part;
		string robot;
		string fwdSlash = "/";
		PortName = fwdSlash;
		string rpcPortName = fwdSlash;

		ConstString robotName=rf.find("robot").asString();
		if (rf.check("robot"))
		{
			robot = rf.find("robot").asString().c_str();
		}
        else
		{
			fprintf(stderr,"robot not fount...using icub as default\n");
			robot = "icub";
		}
		
		if (rf.check("name"))
		{
			PortName += rf.find("name").asString().c_str();
			PortName += fwdSlash;
		}
        else
		{
			fprintf(stderr,"Name not found...using demoForceControl as port name.\n");
			PortName += "demoForceControl/";
			rpcPortName += "demoForceControl/";
		} 
				
		ConstString partName=rf.find("part").asString();
		if (rf.check("part"))
		{
			part = rf.find("part").asString().c_str();
			PortName+=part;
			rpcPortName+=part;
		}
        else
		{
			
			fprintf(stderr,"Could not find part in the config file\n");
		  Time::delay(3.1);
            return false;
		}
		
		// Create the rpc port
		rpcPortName += "/rpc:i";
		rpcPort.open(rpcPortName.c_str());
		attach(rpcPort);                  // attach to port

		string localPort = PortName;
		localPort += "/client";

		string remotePort = "/";
		remotePort += robot; remotePort += "/"; remotePort += part;
		Options.put("robot",robot.c_str());
		Options.put("part",part.c_str());
		Options.put("device","remote_controlboard");
		Options.put("local",localPort.c_str());
		Options.put("remote",remotePort.c_str());

		dd = new PolyDriver(Options);
		if(!createDriver(dd)) 
		{
			fprintf(stderr,"ERROR: unable to create device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");
		
		fprintf(stderr,"input port opened...\n");
		ft_control = new ftCommand(dd,part);
		fprintf(stderr,"ft client istantiated...\n");
		//ft_control->initRobotDemo();
		return true;
	}
	double getPeriod()	{ return 1; }
	bool updateModule() { return true; }
	
	
	bool close()
	{
		ft_control->stopRobotDemo();
		fprintf(stderr,"closing...don't know why :S \n");
		rpcPort.close();
		//if (ft_control) {delete ft_control; ft_control=0;}
		Time::delay(3.0);
		if (dd) {delete dd; dd=0;}
		return true;
	}
};

int main(int argc, char * argv[])
{
    //initialize yarp network
    Network yarp;
	
    //create your module
    ft_ControlModule* ft_controlmodule = new ft_ControlModule();

    // prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose();

    rf.configure("ICUB_ROOT", argc, argv);

	if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
		cout << "\t--robot   robot: the name of the robot (default icub)"                << endl;
        cout << "\t--part    the name of the part (e.g. left_arm, no default)"          << endl;
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
