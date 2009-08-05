#ifndef clockModule__H
#define clockModule__H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>

#include <stdio.h>


using namespace yarp::os;
using namespace yarp::dev;

#define MAX_FREQUENCY 1.5 //in Hz


class clockThread : public yarp::os::RateThread
{
 public:
  clockThread(int period); //constructor
  ~clockThread(); //destructor
  
  void run(); //the main loop
  bool threadInit();
  void threadRelease();

  bool init(Searchable &s);
  void getParameters();
  
 private:
  double period; //in second
  BufferedPort<Bottle> clock_port;
  BufferedPort<Bottle> manager_port;
  BufferedPort<Bottle> beat_port;

  int beat;

  double nu;
  double dt;
  double a,m_on;
  double current_state[2];
  double original_time, theoretical_time, lastBeat_time;
  FILE *debug_file;
};



class clockModule : public Module
{
 private:
  clockThread *theThread;
  

 public:

  virtual bool close();
  virtual bool open(yarp::os::Searchable &s);
  virtual double getPeriod();
  virtual bool updateModule();
};


#endif
