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
 * 07/01/10  Started development.  DV
 * 15/01/10  Completed version 1.  DV
 */ 

#include "iCub/affectiveState.h"


AffectiveState::AffectiveState() {
   debug = true;
}

bool AffectiveState::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - affectiveState.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("affectiveState"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   imageIdInputPortName          = "/";
   imageIdInputPortName         += getName(
                                   rf.check("imageIdInPort", 
                                   Value("/imageId:i"),
                                   "current image id port (string)").asString()
                                   );
   
   modeInputPortName             = "/";
   modeInputPortName            += getName(
                                   rf.check("modePort", 
                                   Value("/mode:i"),
                                   "mode (corresponding to learning, prediction, reconstruction) port (string)").asString()
                                   );
 
   stateOutputPortName           = "/";
   stateOutputPortName          += getName(
                                   rf.check("stateOutPort", 
                                   Value("/state:o"),
                                   "affective state (string)").asString()
                                   );
   
   duration                      = rf.check("duration",
				                   Value(3),
    			                   "number of time samples for weighted average spiking rate (int)").asInt(); 

   expiryAge                     = rf.check("expiry",
				                   Value(10),
    			                   "number of time samples after which an image expires (int)").asInt(); 

   if (debug) {
      printf("affectiveState: module name is %s\n",moduleName.c_str());
      printf("affectiveState: parameters are\n%s\n%s\n%s\n%d\n%d\n\n",   
              imageIdInputPortName.c_str(),
              modeInputPortName.c_str(),
              stateOutputPortName.c_str(),
              duration,
              expiryAge
            );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!imageIdIn.open(imageIdInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << imageIdInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!modeIn.open(modeInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << modeInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!stateOut.open(stateOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << stateOutputPortName << endl;
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

   affectiveStateThread = new AffectiveStateThread(&imageIdIn, 
                                                   &modeIn, 
                                                   &stateOut, 
                                                   &duration,
                                                   &expiryAge);

   /* now start the thread to do the work */

   affectiveStateThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool AffectiveState::interruptModule()
{
   imageIdIn.interrupt();
   modeIn.interrupt();
   stateOut.interrupt();
   handlerPort.interrupt(); 

   return true;
}


bool AffectiveState::close()
{
   imageIdIn.close();
   modeIn.close();
   stateOut.close();
   handlerPort.close(); 

   /* stop the thread */

   affectiveStateThread->stop();

   return true;
}


bool AffectiveState::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "(where <n> is an real number) \n";

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
      if (command.get(1).asString()=="len") {
         duration = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
      else if (command.get(1).asString()=="exp") {
         expiryAge = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }


   return true;
}


/* Called periodically every getPeriod() seconds */

bool AffectiveState::updateModule()
{
   return true;
}



double AffectiveState::getPeriod()
{
   /* module periodicity (seconds), called implicitly by affectiveState */
    
   return 0.1;
}

 

AffectiveStateThread::AffectiveStateThread(BufferedPort<VectorOf<double> >  *imageIdIn,
                                           BufferedPort<VectorOf<double> >  *modeIn,
                                           BufferedPort<VectorOf<double> >  *stateOut,
                                           int                              *duration,
                                           int                              *expiryAge)
{
   imageIdInPort         = imageIdIn;
   modeInPort            = modeIn;
   stateOutPort          = stateOut;
   durationValue         = duration;
   expiryAgeValue        = expiryAge;

   for (int i=0; i<MAX_DURATION; i++){
      curiousitySpikes[i] = 0;
      experimentationSpikes[i] = 0;
   }


   if (debug) {
      cout << "AffectiveStateThread: duration           " << *durationValue << endl;
   }
}

bool AffectiveStateThread::threadInit() 
{

   int i;

   /* initialize variables and create data-structures if needed */

   debug = true;

   imageIdCurrent = 0;
   imageSimilarityCurrent = 0;
   azimuthCurrent = 0;
   elevationCurrent = 0;
   imageIdPrevious = 0;
   imageSimilarityPrevious = 0;
   azimuthPrevious = 0;
   elevationPrevious = 0;

   curiousityLevel = 0;
   deltaCuriousityLevel = 0; 
   experimentationLevel = 0;
   deltaExperimentationLevel = 0;

   for (i=0; i<MAX_DURATION; i++) {
      curiousitySpikes[i] = 0;
      experimentationSpikes[i] = 0;
   }

   for (i=0; i<MAX_NUMBER_OF_EVENTS; i++) {
      eventHistory[i] = *expiryAgeValue;
   }

   numberOfSpikes = 0;
   lastEvent = 0;

   mode = LEARNING_MODE;

   return true;
}

void AffectiveStateThread::run(){

   int i;

   /* 
    * start the affective state operating 
    */ 
      
   if (debug) {
      cout << "AffectiveStateThread::run: duration           " << *durationValue << endl;
   }

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 

      /* 
       * Step 0: decide on the number of spike intervals that will contribute the curiousity and experimentation levels
       * ==============================================================================================================
       *
       */

      if (*durationValue < MAX_DURATION) {
         numberOfSpikes = *durationValue;
      }
      else {
         numberOfSpikes = MAX_DURATION;
      }

      maxCuriousityLevel = 0;
      for (i=0; i<numberOfSpikes; i++) {
         maxCuriousityLevel += i+1;
      }

      maxExperimentationLevel = 0;
      for (i=0; i<numberOfSpikes; i++) {
         maxExperimentationLevel += i+1;
      }


      /* 
       * Step 1: read the mode, the image id, the action, and the previous image id
       * ==========================================================================
       *
       */

      imageIdCurrent = 0;
      imageSimilarityCurrent = 0;
      azimuthCurrent = 0;
      elevationCurrent = 0;
      imageIdPrevious = 0;
      imageSimilarityPrevious = 0;
      azimuthPrevious = 0;
      elevationPrevious = 0;

      /* Determine which mode we are in: learning, exploration, or reconstruction */

      modeIn = modeInPort->read(false);                // read a vector containing the mode; don't block
      if (modeIn != NULL) {
         mode = (int) (*modeIn)[0];                    // if there is nothing there; assume the same mode as before
      }

      /* get the episodic memory data */

      imageIdIn  = imageIdInPort->read(true);       // read a vector containing the image data; do block
      imageIdCurrent          = (int) (*imageIdIn)[0];
      imageSimilarityCurrent  =       (*imageIdIn)[1];
      azimuthCurrent          =       (*imageIdIn)[2];
      elevationCurrent        =       (*imageIdIn)[3];
      imageIdPrevious         = (int) (*imageIdIn)[4];
      imageSimilarityPrevious =       (*imageIdIn)[5];
      azimuthPrevious         =       (*imageIdIn)[6];
      elevationPrevious       =       (*imageIdIn)[7];
         
      /* 
       * Step 2: Determine whether there is a curiousity spike
       * ==========================================================================
       *
       * A curiousity spike occurs when either a new event is recorded in the episodic memory 
       * or when an event that has not recently been recalled is accessed in the episodic memory.
       */

      cSpike = false;

      if (imageIdCurrent > lastEvent) {

         /* new event */

         if (imageIdCurrent < MAX_NUMBER_OF_EVENTS) {
            lastEvent = imageIdCurrent;
            eventHistory[lastEvent] = 0;  // age of event
            cSpike = true;
         }
      }
      else {

         /*  old event */

         if (eventHistory[imageIdCurrent] == *expiryAgeValue) {

            /* event has expired ... not recently recalled */

            cSpike = true;
         }

         // set new age of event

         eventHistory[imageIdCurrent] = 0;  // age of event
      }

      /* now update the ages of all events */

      for (i=0; i<=lastEvent; i++) {
         if (eventHistory[i] < *expiryAgeValue) {
            eventHistory[i]++;
         }
      }

      if (debug) {
         cout << "AffectiveStateThread::run: eventHistory:      " ;
         for (i=0; i<=lastEvent; i++) {
            cout << eventHistory[i] << " ";
         }
         cout << endl;
      }


      /* insert the spike into the spike stream */

      for (i=0; i<numberOfSpikes-1; i++) {
         curiousitySpikes[i] = curiousitySpikes[i+1]; // shift
      }

      if (cSpike == true) {
         curiousitySpikes[numberOfSpikes-1] = 1;
      }
      else {
         curiousitySpikes[numberOfSpikes-1] = 0;
      }

      if (debug) {
         cout << "AffectiveStateThread::run: curiousitySpikes:      " ;
         for (i=0; i<numberOfSpikes; i++) {
            cout << curiousitySpikes[i] << " ";
         }
         cout << endl;
      }

      /* 
       * Step 3: Determine whether there is an experimentation spike
       * ==========================================================================
       *
       * An experimentation spike occurs when an event / image which is predicted or reconstructed by the procedural memory 
       * and recalled by the episodic memory is subsequently recalled again by the episodic memory, typically as a result of
       * the endogenous salience successfully causing the attention system to attend to the predicted / reconstructed event.
       * This is equivalent to the sequential recall of the same event in episodic memory when in either prediction or
       * reconstruction action mode.
       *  
       */


      eSpike = false;

      if (
          //(mode == LEARNING_MODE) || 
          (mode == PREDICTION_MODE) || 
          (mode == RECONSTRUCTION_MODE)
          ) {
         if (imageIdCurrent == imageIdPrevious) {
            eSpike = true;
         }
      }

      /* insert the spike into the spike stream */

      for (i=0; i<numberOfSpikes-1; i++) {
         experimentationSpikes[i] = experimentationSpikes[i+1]; // shift
      }

      if (eSpike == true) {
         experimentationSpikes[numberOfSpikes-1] = 1;
      }
      else {
         experimentationSpikes[numberOfSpikes-1] = 0;
      }

      if (debug) {
         cout << "AffectiveStateThread::run: experimentationSpikes: " ;
         for (i=0; i<numberOfSpikes; i++) {
            cout << experimentationSpikes[i] << " ";
         }
         cout << endl;
      }
      /* 
       * Step 4: Compute the curiousity and experimentation levels and write out
       * ==========================================================================
       *
       */

      previousCuriousityLevel = curiousityLevel;
      curiousityLevel = 0;
      for (i=0; i<numberOfSpikes; i++) {
         curiousityLevel += curiousitySpikes[i] * (i+1); // most recent spike is most heavily weighted
      }
      curiousityLevel = curiousityLevel/maxCuriousityLevel;
      deltaCuriousityLevel = curiousityLevel - previousCuriousityLevel;

      previousExperimentationLevel = experimentationLevel;
      experimentationLevel = 0;
      for (i=0; i<numberOfSpikes; i++) {
         experimentationLevel += experimentationSpikes[i] * (i+1); // most recent spike is most heavily weighted
      }
      experimentationLevel = experimentationLevel/maxExperimentationLevel;
      deltaExperimentationLevel = experimentationLevel - previousExperimentationLevel;

      VectorOf<double>& out = stateOutPort->prepare();
      out.resize(4,0);
      out[0] = curiousityLevel;
      out[1] = deltaCuriousityLevel; 
      out[2] = experimentationLevel;
      out[3] = deltaExperimentationLevel;
      stateOutPort->write();


      if (debug) {
         if      (mode == LEARNING_MODE)       cout << "AffectiveStateThread::run: Learning mode"    << endl;
         else if (mode == PREDICTION_MODE)     cout << "AffectiveStateThread::run: Prediction mode"  << endl;
         else if (mode == RECONSTRUCTION_MODE) cout << "AffectiveStateThread::run: Explanation mode" << endl;
         cout << "AffectiveStateThread::run: imageIdPrevious           " << imageIdPrevious           << endl;
         cout << "AffectiveStateThread::run: imageIdCurrent            " << imageIdCurrent            << endl;
         cout << "AffectiveStateThread::run: lastEvent                 " << lastEvent                 << endl;
         cout << "AffectiveStateThread::run: eventHistory[]            " << eventHistory[imageIdCurrent]<< endl;
         //cout << "AffectiveStateThread::run: imageSimilarityPrevious   " << imageSimilarityPrevious   << endl;
         //cout << "AffectiveStateThread::run: imageSimilarityCurrent    " << imageSimilarityCurrent    << endl;
         //cout << "AffectiveStateThread::run: azimuthPrevious           " << azimuthPrevious           << endl;
         //cout << "AffectiveStateThread::run: azimuthCurrent            " << azimuthCurrent            << endl;
         //cout << "AffectiveStateThread::run: elevationPrevious         " << elevationPrevious         << endl;
         //cout << "AffectiveStateThread::run: elevationCurrent          " << elevationCurrent          << endl;
         cout << "AffectiveStateThread::run: curiousityLevel           " << curiousityLevel           << endl;
         //cout << "AffectiveStateThread::run: deltaCuriousityLevel      " << deltaCuriousityLevel      << endl;
         cout << "AffectiveStateThread::run: experimentationLevel      " << experimentationLevel      << endl;
         //cout << "AffectiveStateThread::run: deltaExperimentationLevel " << deltaExperimentationLevel << endl;
         cout << endl;
      }
   }
}

void AffectiveStateThread::threadRelease() 
{

}

