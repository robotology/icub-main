// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceControlModule.h"
#include <ace/OS.h>



BalanceControlThread::BalanceControlThread(int period) : RateThread(period)
{
	this->period = ((double)period)/1000.0;
}

BalanceControlThread::~BalanceControlThread()
{
}


bool BalanceControlThread::threadInit()
{
	fprintf(stderr, "BalanceControlThread::thread init>> done\n");
	return true;
}

void BalanceControlThread::threadRelease()
{
	delete myIK;
}


void BalanceControlThread::run()
{
}



bool BalanceControlThread::init(Searchable &s)
{
	Property arguments(s.toString());
	Time::turboBoost();

	Property options;

	//get the conf from file
	if(arguments.check("file")) {
		options.fromConfigFile(arguments.find("file").asString().c_str());
	} else {//get the default conf
		char *cubPath;
		cubPath = getenv("ICUB_DIR");
		if(cubPath == NULL) {
			ACE_OS::printf("BalanceControlThread::init>> ERROR getting env var ICUB_DIR, exiting\n");
			return false;
		}
		yarp::String cubPathStr(cubPath);
		options.fromConfigFile((cubPathStr + "app/Crawling/config/balanceControlConfig.ini").c_str());
	}
	if(options.check("robot")) {
		this->robot = options.find("robot").asString();
	} else {
		robot = "icub";
	}

	myIK = new IKManager;

	////////opening ports to receive commands from the manager/////
	bool ok;
	char tmp1[255], tmp2[255];

}

///////////////////////////////////////////
/////////////////MODULE CODE///////////////
///////////////////////////////////////////
bool BalanceControlModule::close()
{
	theThread->stop();
	delete theThread;
	fprintf(stderr, "BalanceControlModule closed\n");
	return true;
}



bool BalanceControlModule::updateModule()
{
	return true;
}



bool BalanceControlModule::open(yarp::os::Searchable & s)
{
	Property options(s.toString());
	int period = 50; // the default value for the period in ms

	//check if another value was provided
	if(options.check("period"))
	{
		period = options.find("period").asInt();
	}


	theThread = new BalanceControlThread(period);
	if(!theThread->init(s))
	{
		ACE_OS::printf("BalanceControlModule::open>> Failed to initialize the thread\n");
		fflush(stdout);
		return false;
	}

	theThread->start();

	return true;
}

double BalanceControlModule::getPeriod() {
	return 1.0;
}

