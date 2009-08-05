#ifndef CRAWL_MANAGER_MODULE__H
#define CRAWL_MANAGER_MODULE__H


#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/String.h>


#include <stdio.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


class managerThread : public yarp::os::RateThread
{
 public:
    managerThread(int period); //constructor
    ~managerThread(); //destructor
  
    void run(); //the main loop
    bool threadInit();
    void threadRelease();

    bool init(Searchable &s);
 private:
    double period; //in second
    BufferedPort<Bottle> parts_port[4];
    bool connected_part[4];

    double parameters[4][2*4];
    double om_swing,om_stance;
};

class CrawlManagerModule : public Module
{
 private:        
    managerThread *theThread;
  
 public:
    virtual bool close();
    virtual bool open(yarp::os::Searchable &s);
    virtual double getPeriod();
    virtual bool updateModule();
};


#endif
