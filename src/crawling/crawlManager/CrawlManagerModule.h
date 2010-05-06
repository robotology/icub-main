#ifndef CRAWL_MANAGER_MODULE__H
#define CRAWL_MANAGER_MODULE__H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>

using namespace yarp::os;
using namespace yarp::dev;

#include <stdio.h>
#include <iostream>
#include <vector>

using namespace std;

//auxiliary variables

#define LEFT_ARM 1 
#define RIGHT_ARM -1

//behaviors
#define NOT_SET -1
#define INIT_POS 0
#define CRAWL 1
#define REACH 2

#define COMMAND_PORT_NAME "/manager/in" // port to receive command from the reachingManager or planner

#define TURN_INDENT 0.05 //increment in radians
#define STANCE_INDENT 0.05 //increment in radians
#define MAX_TURN_ANGLE 0.5

#define SEB_TURN_COMMAND 77 // added by Seb
#define REACH_COMMAND 66 // added by Seb
#define HEAD_ROT_COMMAND 55 // added by Seb

using namespace std;


///The CrawlManagerModule sends the parameters to the generators according to the selected behavior
class CrawlManagerModule : public Module
{    
    private:
    
        static const int nbParts = 6; //number of limbs   	   
        int nbDOFs[nbParts]; //nb of dofs for each part
        ConstString part_names[nbParts]; // names of the different parts 
        bool connected_part[nbParts]; //true if part is connected, false otherwise

        BufferedPort<Bottle> parts_port[nbParts]; //ports to send the parameters to the generators
        BufferedPort<Bottle> check_port[nbParts]; //ports to check the current status of the cpgs
        BufferedPort<Bottle> commandPort; // ports getting the command from the reachManager and the planner

        int STATE; //current behavior (INIT_POS; CRAWL; NOT_SET)
        int com; //command to switch between ehaviors

        Property options; //getting the info in the config file
        vector<vector<double> > init_parameters, crawl_parameters;//parameters for init pos (on all fours) and crawling
        vector<vector<double> > crawl_left_parameters, crawl_right_parameters; //intermediate pose (left (right) arm lifted) 
        vector<double> om_swing, om_stance; //control the duration of the swing and the stance resp. 
        double turnAngle; //set point of the torso roll (controls the turning)

        void sendCommand(int i, vector<vector<double> > params);//function sending commands to the generator
        int getSwingingArm();//function getting which arm is swinging

		void InitPosition(void);
		void Crawl(double desiredTurnAngle=0,double omstanceIncrement=0);
		void Reach(Bottle *reachingCommand);
		void HeadControl(double pitchAngle, double yawAngle);
   
    public:
        virtual bool close();
        virtual bool open(Searchable &s);
        virtual double getPeriod();
        virtual bool updateModule();
        virtual bool respond(const Bottle &command, Bottle &reply);       

};


/* mvt_parameters 

0 left_arm amplitude
1 left_arm target
2 right_arm amplitude
3 right_arm target
   0 shoulder pitch
   1 shoulder roll 
   2 shoulder yaw
   3 elbow
   4 forearm
   5 wrist pitch
   6 wrist roll
 
4 left_leg amplitude
5 left_leg target
6 right_leg amplitude
7 right_leg target
   0 hip pitch
   1 hip roll 
   2 hip yaw
   3 knee
   4 ankle pitch
   5 ankle yaw
   
8 torso amplitude
9 torso target
   0 torso pitch
   1 torso roll
   2 torso yaw
    
10 head amplitude
11 head target
   0 head pitch
   1 head yaw
   2 head roll

*/

#endif
