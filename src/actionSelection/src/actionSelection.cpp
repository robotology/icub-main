/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


/*
 * Audit Trail
 * -----------
 * 08/01/010  Started development.  DV
 */ 

#include "iCub/actionSelection.h"


ActionSelection::ActionSelection() {
   debug = true;
}

bool ActionSelection::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - actionSelection.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("actionSelection"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   stateInputPortName            = "/";
   stateInputPortName           += getName(
                                   rf.check("stateInPort", 
                                   Value("/state:i"),
                                   "affective state input port (string)").asString()
                                   );
   
   modeOutputPortName             = "/";
   modeOutputPortName            += getName(
                                   rf.check("modeOutPort", 
                                   Value("/mode:o"),
                                   "mode - learning, prediction, reconstruction - output port (string)").asString()
                                   );
 
   mode                          = rf.check("mode",
				                   Value(0),
    			                   "default mode (int)").asInt(); 

   if (debug) {
      printf("actionSelection: module name is %s\n",moduleName.c_str());
      printf("actionSelection: parameters are\n%s\n%s\n%d\n\n",   
              stateInputPortName.c_str(),
              modeOutputPortName.c_str(),
              mode
            );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!stateIn.open(stateInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << stateInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!modeOut.open(modeOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << modeOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

 

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

   handlerPortName =  "/";
   handlerPortName += getName();         // use getName() rather than a literal 
 
   if (!handlerPort.open(handlerPortName.c_str())) {           
      cout << getName() << ": Unable to open port " << handlerPortName << endl;  
      return false;
   }

   attach(handlerPort);                  // attach to port
 
   attachTerminal();                     // attach to terminal

   /* create the thread and pass pointers to the module parameters */

   actionSelectionThread = new ActionSelectionThread(&stateIn, 
                                                     &modeOut, 
                                                     &mode);

   /* now start the thread to do the work */

   actionSelectionThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool ActionSelection::interruptModule()
{
   stateIn.interrupt();
   modeOut.interrupt();
   handlerPort.interrupt(); 

   return true;
}


bool ActionSelection::close()
{
   stateIn.close();
   modeOut.close();
   handlerPort.close(); 

   /* stop the thread */

   actionSelectionThread->stop();

   return true;
}


bool ActionSelection::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set mode <n>    ... set current / default mode: 0-3\n" + 
                        "(where <n> is an integer number) \n";


  reply.clear(); 

  if (command.get(0).asString()=="quit") {
       reply.addString("quitting");
       return false;     
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="mode") {
         mode = command.get(2).asInt(); // set parameter value
         reply.addString("mode set ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool ActionSelection::updateModule()
{
   return true;
}



double ActionSelection::getPeriod()
{
   /* module periodicity (seconds), called implicitly by actionSelection */
    
   return 0.1;
}

 

ActionSelectionThread::ActionSelectionThread(BufferedPort<VectorOf<double> > *stateIn,
                                             BufferedPort<VectorOf<double> > *modeOut,
                                             int                             *mode)
{
   stateInPort         = stateIn;
   modeOutPort         = modeOut;
   modeValue           = mode;

   if (debug) {
      cout << "ActionSelectionThread: mode               " << *modeValue << endl;
   }
}

bool ActionSelectionThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

   debug = false;

   curiousityLevel = 0;
   deltaCuriousityLevel = 0; 
   experimentationLevel = 0;
   deltaExperimentationLevel = 0;

   return true;
}

void ActionSelectionThread::run(){

   /* 
    * start the action selection operating 
    */ 
      
   if (debug) {
      cout << "ActionSelectionThread::run: mode               " << *modeValue << endl;
   }

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true

      /* read the affective state levels and determine which mode we are in: learning, exploration, or reconstruction */
  
      stateIn = stateInPort->read(true);       // read a vector containing the state data; do block

      if (stateIn != NULL) {
         curiousityLevel            = (*stateIn)[0];
         deltaCuriousityLevel       = (*stateIn)[1];
         experimentationLevel       = (*stateIn)[2];
         deltaExperimentationLevel  = (*stateIn)[3];

         if (curiousityLevel > experimentationLevel) {
            *modeValue = 0; // learning
         }
         else {
            *modeValue = 1; // prediction
         }
      }
 
      mode = *modeValue;
 
      VectorOf<double>& out = modeOutPort->prepare();
      out.resize(1,0);
      out[0] = mode;
      modeOutPort->write();


      if (debug) {
         if      (mode == LEARNING_MODE)       cout << "ActionSelectionThread::run: Learning mode"    << endl;
         else if (mode == PREDICTION_MODE)     cout << "ActionSelectionThread::run: Prediction mode"  << endl;
         else if (mode == RECONSTRUCTION_MODE) cout << "ActionSelectionThread::run: Explanation mode" << endl;
         cout << "ActionSelectionThread::run: curiousityLevel           " << curiousityLevel           << endl;
         //cout << "ActionSelectionThread::run: deltaCuriousityLevel      " << deltaCuriousityLevel      << endl;
         cout << "ActionSelectionThread::run: experimentationLevel      " << experimentationLevel      << endl;
         //cout << "ActionSelectionThread::run: deltaExperimentationLevel " << deltaExperimentationLevel << endl;
         cout << endl;
      }
   }
}

void ActionSelectionThread::threadRelease() 
{

}

