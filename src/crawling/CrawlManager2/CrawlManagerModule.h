#ifndef CRAWL_MANAGER_MODULE__H
#define CRAWL_MANAGER_MODULE__H


#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/String.h>


#include <stdio.h>
#include <iostream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


class CrawlManagerModule : public Module
{
 private:        
  //managerThread *theThread;
  
    BufferedPort<Bottle> parts_port[4];
    bool connected_part[4];

    double crawl_parameters[4][2*4];
    double init_parameters[4][2*4];
    double om_swing,om_stance;


    int com;

 public:
    virtual bool close();
    virtual bool open(yarp::os::Searchable &s);
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command, Bottle &reply);
    //virtual int runModule(int argc, char *argv[]);
};


#endif
