#ifndef drum_h
#define drum_h

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/os/Vocab.h>
#include <yarp/String.h>

#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <ace/Sched_Params.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <list>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

#define LEFT_ARM 0
#define RIGHT_ARM 1
#define LEFT_LEG 2
#define RIGHT_LEG 3
#define HEAD 4
                            
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
    int init[nbparts];
   
    BufferedPort<Bottle> param_port[nbparts], check_port[nbparts];
    BufferedPort<Bottle> score_port[nbparts],sound_port[nbparts], phase_port[nbparts];
    BufferedPort<Bottle> interactive_port, midi_port, clock_port, beat_clock_port;
    BufferedPort<Bottle> scan_port, head_port;
   
    int drum_beat[nbparts], current_beat[nbparts];
    int beat[nbparts];

    int beat_clock;
    int drum_beat_clock;
    int sizeScore;
    int scan;
    int nbDrums[nbparts]; //how many target positions per dof (in config file partTargets.ini)
    int nbIds[nbparts];

    //parameters to be sent
    double m_off;//no oscillations
    double m_on;//oscillations
    double freqHead;
  
    double **mu;
    double **g;
    double ***G;
    double **MU;
    double *frequency;
    double **phase_shift;
    int **id;
    double **notes;          
    int **Score;
    bool **id_pos_found;

  
    FILE *feedback_file;
    bool soundFeedback;
    bool visualFeedback;
  
    //functions
    void sendNewParameterSet(int i);
    bool getRhythm();
    void openPort(int i, BufferedPort<Bottle> *port, ConstString port_name, int out, int connect);
    void sendSoundFeedback();
    void scanDrumPosition();
    void scanHeadPosition();
    void getConfig();
    void doConnect();
    void getScore();
    void sendScore();
    void getPosition();
    void getBeat();
    void run();
};


#endif
