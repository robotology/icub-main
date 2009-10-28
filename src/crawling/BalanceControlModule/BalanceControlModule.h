#ifndef BALANCE_CONTROL_MODULE__H
#define BALANCE_CONTROL_MODULE__H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/String.h>

#include "CrawlInvKin.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


#include <stdio.h>
#include <iostream>
#include <vector>



class BalanceControlThread : public yarp::os::RateThread {
public:

	BalanceControlThread(int period); //constructor
	~BalanceControlThread(); //destructor

	void run();
	bool threadInit();
	void threadRelease();

	bool init(Searchable &s);


private:
	double period;
    ConstString robot;

    myIK *IKManager;

};

//THis module is in charge for the balance controller
class BalanceControlModule : public Module
{    
private:
	BalanceControlThread *theThread;


public:
	virtual bool close();
	virtual bool open(yarp::os::Searchable &s);
	virtual double getPeriod();
	virtual bool updateModule();
};



#endif
