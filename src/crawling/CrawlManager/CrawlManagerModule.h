#ifndef CRAWL_MANAGER_MODULE__H
#define CRAWL_MANAGER_MODULE__H


#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/String.h>
using namespace yarp::os;
using namespace yarp::dev;

#include <stdio.h>
#include <iostream>
#include <vector>

#include "CrawlInvKin.h"

using namespace std;

class CrawlManagerModule : public Module
{
 private:
 
   static const int nbParts = 6;
   
   BufferedPort<Bottle> parts_port[nbParts];
   bool connected_part[nbParts];


   double om_swing,om_stance;

   IKManager *myIK;
   
   vector<double> closedLoop;

   vector<vector<vector<double> > > unsit_parameters, sit_parameters;
   vector<vector<double> > init_parameters, crawl_parameters, crawl_init_parameters;
   vector<vector<double> > encoders;

   ConstString part_names[nbParts];

   int com;
   
   int nbDOFs[nbParts];

   double left_turn, right_turn;
   
   Property opt;

   int nbPosSit;
   int nbPosUnsit;
   
   double turnAngle;
   double scaleLeg;
   vector<double> turnParams;

   bool getSequence(vector<vector<vector<double> > > parameters, ConstString task, int& nbPos);
   void sendCommand(int i, vector<vector<double> > params);

 public:

   virtual bool close();
   virtual bool open(Searchable &s);
   virtual double getPeriod();
   virtual bool updateModule();
   virtual bool respond(const Bottle &command, Bottle &reply);
   virtual bool interruptModule();
   
   double scale[nbParts];
   //virtual int runModule(int argc, char *argv[]);
};


#endif
