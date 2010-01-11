/** 
 * @ingroup icub_module
 *
 * \defgroup icub_affectiveState affectiveState
 *
 * Affective state is a competitive network of three motives: 
 * 
 * - curiosity (dominated by exogenous factors)
 * - experimentation (dominated by endogenous factors)
 * - social engagement (where exogenous and endogenous factors balance)
 * 
 * The outcome of this competition fed to the actionSelection module which produces a cognitive behaviour 
 * that is biased towards learning (motivated by curiousity), and
 * prediction and reconstruction (motivated by curiousity and social engagement).
 * 
 * Both curiousity and experimentation are forms of exploration.   
 * 
 * Social engagement is an exploratory act to but it focusses on the establishment of a common basis 
 * for mutual interations with others rather than on acquiring new knowledge about the world. 
 * In a sense, exploration concerns the acquisition of knowledge, whereas social engagement concerns 
 * the agreement of knowledge.
 * 
 * In the current implementation, we target only simple forms of curiousity and experimentation motives.   
 * 
 * Both motives are modelled as a temporal series of event-related spikes.
 * The current level of a motivation is a weighted sum of recent spiking activity, with weighting being biased 
 * towards most recent spikes. 
 *
 * The number of time intervals over which the sum is taken is provided as a parameter to the module.
 * 
 * Curiousity and experimentation have different spiking functions.
 * 
 * A curiousity spike occurs when either a new event is recorded in the episodic memory 
 * or when an event that has not recently been recalled is accessed in the episodic memory.
 * 
 * An experimentation spike occurs when an event / image which is predicted or reconstructed by the procedural memory 
 * and recalled by the episodic memory is subsequently recalled again by the episodic memory, typically as a result of
 * the endogenous salience successfully causing the attention system to attend to the predicted / reconstructed event.
 * This is equivalent to the sequential recall of the same event in episodic memory when in either prediction or
 * reconstruction action mode.
 *
 * The affectiveState module takes as input the imageId  and the action output 
 * from the actionSelection module.
 * 
 *
 * The affectiveState module has the following inputs: 
 * 
 * - image vector containing eight numbers
 *   -# an image id. number for the matched image
 *   -# a match value r, 0 <= r <= 1, for the matched image
 *   -# the azimuth angle of the gaze at which the image was acquired
 *   -# the elevation angle of the gaze at which the image was acquired
 *   -# an image id. number for the previously matched image
 *   -# a match value r, 0 <= r <= 1, for the previously matched image
 *   -# the azimuth angle of the gaze at which the previously image was acquired
 *   -# the elevation angle of the gaze at which the previously image was acquired
 *
 *   This is typically input from the episodicMemory module.
 *
 * - mode vector containing a single value denoting the mode of operation
 *   - 0 (learning)
 *   - 1 (prediction) 
 *   - 2 (reconstruction) 
 *
 *   This is typically input from the actionSelection module.
 *
 *
 * The affectiveState module has one outputs: affective state vector containing four numbers
 *
 *    -# the current curiousity level 
 *    -# the instantaneous rate of change of curiousity level
 *    -# the current experimentation level 
 *    -# the instantaneous rate of change of experimentation level
 *
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b>
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from \c file.ini ). The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c affectiveState.ini       \n
 *   specifies the configuration file
 *
 * - \c context \c affectiveState/conf   \n 
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c affectiveState \n         
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file e.g. \c affectiveState.ini 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c imageIdInPort       \c /imageId:i       \n  
 *   The input port name for the current image id. number and match value
 *
 * - \c modePort            \c /mode:i          \n  
 *   The input port name for the mode (corresponding to learning, prediction, reconstruction) 
 *
 * - \c stateOutPort        \c /state:o       \n 
 *   The output port name for the affective state value  
 *
 * - \c duration            \c 3              \n
 *   specifies the number of time samples to be used when computing the weighted average spiking rate.
 *
 *
 * \section portsa_sec Ports Accessed
 * 
 * - none
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /affectiveState \n
 *   This port is used to change the parameters of the module at run time or stop the module
 *   The following commands are available
 * 
 *   \c help \n
 *   \c quit
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /rectification
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /affectiveState/imageId:i 
 * - \c /affectiveState/mode:i
 *
 *
 * <b>Output ports</b>
 *
 * - \c /affectiveState 
 * - \c /affectiveState/state:o 
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - \c BufferedPort<VectorOf<double> >  \c imageIdInPort;          \c // \c image_id, \c match_value, \c azimuth, \c elevation, \c previous_image_id, \c match_value, \c azimuth, \c elevation
 * - \c BufferedPort<VectorOf<double> >  \c modePort;               \c // \c mode  
 * - \c BufferedPort<VectorOf<double> >  \c stateOutPort;           \c // \c state 
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c affectiveState.ini  in \c $ICUB_ROOT/app/affectiveState/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>affectiveState --name affectiveState --context affectiveState/conf --from affectiveState.ini</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/affectiveState/include/iCub/affectiveState.h
 * 
 */


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
 * 07/01/10  Began development      DV
 */ 


#ifndef __ICUB_AFFECTIVESTATE_MODULE_H__
#define __ICUB_AFFECTIVESTATE_MODULE_H__


/* System includes */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <string>


/* YARP includes */

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;


/* affectiveState definitions */

#define LEARNING_MODE                          0
#define PREDICTION_MODE                        1
#define RECONSTRUCTION_MODE                    2
#define MAX_DURATION                         100
#define MAX_NUMBER_OF_EVENTS                 300
#define MAX_EVENT_AGE                         10


class AffectiveStateThread : public Thread
{
private:

   /* class variables */

   VectorOf<double>      *imageIdIn;
   VectorOf<double>      *modeIn;

   int                   imageIdCurrent;
   double                imageSimilarityCurrent;
   double                azimuthCurrent;
   double                elevationCurrent;
   int                   imageIdPrevious;
   double                imageSimilarityPrevious;
   double                azimuthPrevious;
   double                elevationPrevious;
   double                azimuthDelta;
   double                elevationDelta;
   int                   mode;
   double                curiousityLevel;
   double                previousCuriousityLevel;
   double                maxCuriousityLevel;
   double                deltaCuriousityLevel;
   double                experimentationLevel;
   double                previousExperimentationLevel;
   double                maxExperimentationLevel;
   double                deltaExperimentationLevel;
   int                   curiousitySpikes[MAX_DURATION];
   int                   experimentationSpikes[MAX_DURATION];
   int                   eventHistory[MAX_NUMBER_OF_EVENTS];
   int                   numberOfSpikes;
   int                   lastEvent;
   bool                  cSpike;
   bool                  eSpike;

   bool debug;
  	    
   /* thread parameters: they are pointers so that they refer to the original variables in rectification */

   BufferedPort<VectorOf<double> >   *imageIdInPort;
   BufferedPort<VectorOf<double> >   *modeInPort;
   BufferedPort<VectorOf<double> >   *stateOutPort;
   int                               *durationValue;

public:

   /* class methods */

   AffectiveStateThread(BufferedPort<VectorOf<double> >  *imgIdIn,
                        BufferedPort<VectorOf<double> >  *modeIn,
                        BufferedPort<VectorOf<double> >  *stateOut,
                        int                              *duration
                       );
   bool threadInit();     
   void threadRelease();
   void run (); 
};


class AffectiveState:public RFModule
{
   /* port names */

   string moduleName;
   string imageIdInputPortName;
   string modeInputPortName;
   string stateOutputPortName;  
   string handlerPortName;

   /* class variables */

   bool debug;

   /* parameters */

   int duration;


   /* ports */

   BufferedPort<VectorOf<double> >   imageIdIn;
   BufferedPort<VectorOf<double> >   modeIn;
   BufferedPort<VectorOf<double> >   stateOut;
   Port                              handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   AffectiveStateThread *affectiveStateThread;


public:
   AffectiveState();
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_AFFECTIVESTATE_MODULE_H__
//empty line to make gcc happy

