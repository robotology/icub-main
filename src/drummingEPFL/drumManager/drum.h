#ifndef drum_h
#define drum_h

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>

#include<vector>
#include<iostream>
#include<fstream>
#include<sstream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

class drum{

 public:

  drum();
  ~drum();

  char pathToConfig[255];
 
  //ROBOT STUFF
  static const int nbparts = 5; //number of interfaces
  ConstString parts[nbparts];//name of the interfaces
  bool ok[nbparts]; //which interface is active (=1 if parameter port for the part open by DrumGenerator)
  int controlled_dofs[nbparts];//how many dofs are controlled (in config file partConfig.ini)

  BufferedPort<Bottle> param_port[nbparts], check_port[nbparts], score_port[nbparts], phase_port[nbparts];
  BufferedPort<Bottle> interactive_port, clock_port, beat_clock_port;

  int drum_beat[nbparts], current_beat[nbparts];
  int beat[nbparts];
  
  //functions
  void sendNewParameterSet(int id, double phase, double freq, int i);
  int getRhythm(Bottle *Rhythm, int beat);
  void openPort(int i, BufferedPort<Bottle> *port, ConstString port_name, int out, int connect);
 
  //SCORES STUFF
  static const int sizeScore = 16; //typical size of a score
  double rhythmParam[sizeScore]; //frequency
  double phase_shift[nbparts][sizeScore];//phase shift

};
#endif
