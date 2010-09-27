#include "iCub/SkinDriftCompensation.h"


bool SkinDriftCompensation::configure(yarp::os::ResourceFinder &rf)
{    
	/* Process all parameters from both command-line and .ini file */

	/* get the module name which will form the stem of all module port names */
	moduleName			= rf.check("name", Value("SkinDriftCompensation"), "module name (string)").asString();
	robotName			= rf.check("robot", Value("icub"), "name of the robot (string)").asString();
	/* before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name*/
	setName(moduleName.c_str());

	bool rightHand;
	string hand				= rf.check("hand", Value("right"), "Hand to take as reference (string)").asString().c_str();
	if(hand.compare("right")==0){
		rightHand = true;
		compensatedTactileDataPortName		= "/"+ robotName+ "/skin/righthandcomp";
	}else if(hand.compare("left")==0){
		rightHand = false;
		compensatedTactileDataPortName		= "/"+ robotName+ "/skin/lefthandcomp";
	}else{
		return false;
		/*compensatedTactileDataPortName		= "/";
		compensatedTactileDataPortName		+= getName( rf.check("compensatedTactileDataPort", 
			Value(("/"+ robotName+ "/skin/lefthandcomp").c_str()), "Compensated tactile data output port (string)").asString());*/
	}

	/* get some other values from the configuration file */
	float minBaseline			= (float)rf.check("minBaseline", Value(3), 
	   "If the baseline reaches this value then, if allowed, a calibration is executed (float in [0,255])").asDouble();
	
	bool zeroUpRawData = true;
	string zeroUpRawDataStr		= rf.check("zeroUpRawData", Value("true"), 
	   "if true the raw data are considered from zero up, otherwise from 255 down (string)").asString().c_str();
	if(zeroUpRawDataStr.compare("true")==0){
		zeroUpRawData = true;
	}else if(zeroUpRawDataStr.compare("false")==0){
		zeroUpRawData = false;
	}
	 
	/* open ports  */        
	if (!compensatedTactileDataPort.open(compensatedTactileDataPortName.c_str())) {
		cout << getName() << ": unable to open port " << compensatedTactileDataPortName << endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}

	/*
	* attach a port of the same name as the module (prefixed with a /) to the module
	* so that messages received from the port are redirected to the respond method
	*/
	handlerPortName = "/";
	string emptyStr = "";
	handlerPortName += getName(rf.check("handlerPort", Value("/rpc")).asString());
	if (!handlerPort.open(handlerPortName.c_str())) {
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}

	attach(handlerPort);                  // attach to port
	//attachTerminal();                     // attach to terminal
	


	/* create the thread and pass pointers to the module parameters */
	calibrationAllowed = false;		// default -> calibration is not allowed!
	myThread = new MyThread(&compensatedTactileDataPort, robotName, &minBaseline, &calibrationAllowed, &forceCalibration, 
		&zeroUpRawData, &rightHand);
	/* now start the thread to do the work */
	myThread->start(); // this calls threadInit() and it if returns true, it then calls run()

	return true ;      // let the RFModule know everything went well
					  // so that it will then run the module
}


bool SkinDriftCompensation::interruptModule()
{
	compensatedTactileDataPort.interrupt();
	handlerPort.interrupt();

	return true;
}


bool SkinDriftCompensation::close()
{
	/* stop the thread */
	if(myThread)
		myThread->stop();

	compensatedTactileDataPort.close();
	handlerPort.close();   

	return true;
}


bool SkinDriftCompensation::respond(const Bottle& command, Bottle& reply) 
{
	string helpMessage =  string(getName().c_str()) + " commands are: \n" +  "help \n" + "quit\n" + "forbid calibration\n" + 
		"allow calibration\n" + "force calibration";
	reply.clear(); 

	if (command.get(0).asString()=="quit") {
	   reply.addString("quitting");
	   return false;
	}
	else if (command.get(0).asString()=="help") {
		cout << helpMessage;
		reply.addString("ok");
	}
	else if (command.get(0).asString()=="forbid") {
		if (command.get(1).asString()=="calibration") {
			calibrationAllowed = false; // set parameter value
			reply.addString("ok");
		}
	}
	else if (command.get(0).asString()=="allow") {
		if (command.get(1).asString()=="calibration") {
			calibrationAllowed = true; // set parameter value
			reply.addString("ok");
		}
	}
	else if (command.get(0).asString()=="force") {
		if (command.get(1).asString()=="calibration") {
            fprintf(stderr, "force calibration message received\n");
			forceCalibration = true; // set parameter value
			reply.addString("ok");
		}
	}
	return true;
}

bool SkinDriftCompensation::updateModule(){ return true;}
double SkinDriftCompensation::getPeriod(){ return 0.1;}

