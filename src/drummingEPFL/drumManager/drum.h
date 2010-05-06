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
  static const int max_dofs=4; //maximum number of dofs controlled TO DO: deduce from config file

  BufferedPort<Bottle> param_port[nbparts], check_port[nbparts], score_port[nbparts],sound_port[nbparts], phase_port[nbparts];
  BufferedPort<Bottle> interactive_port, midi_port, clock_port, beat_clock_port;

  //FEEDBACK STUFF
  FILE *feedback_file; //file to store at which time sound feedback information has been received
  int notes[nbparts][max_dofs]; //number corresponding to the instrument beaten for the feedback (in partTargets.ini)
  int soundFeedback;


  int drum_beat[nbparts], current_beat[nbparts];
  int beat[nbparts];

  //parameters to be sent
  double m_off;//no oscillations
  double m_on;//oscillations
  
  double **mu;
  //double **mu_on;

  //TO DO deduce from config files
  double **g;
  double ***G;
  double **MU;

  //functions
  void sendNewParameterSet(double *mu, double *g, double phase, double freq, int i, BufferedPort<Bottle> *param_port);
  int getRhythm(Bottle *Rhythm, int beat);
  void openPort(int i, BufferedPort<Bottle> *port, ConstString port_name, int out, int connect);
  void sendSoundFeedback(Bottle *Hit, BufferedPort<Bottle> *sound_port);
  
  //SCORES STUFF
  static const int sizeScore = 16; //typical size of a score  TO DO: Improve this
  int nbDrums[nbparts]; //how many target positions per dof (in config file partTargets.ini)
  static const int max_drums=4; //max number of drums TO DO: deduce from config file
  double rhythmParam[sizeScore];
  double phase_shift[nbparts][sizeScore];

};


#endif
