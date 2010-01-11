/** 
 * @ingroup icub_module
 *
 * \defgroup icub_actionSelection actionSelection
 *
 * The purpose of action selection is to effect the development of the iCub and specifically to increase 
 * its predictive capability.  In the current context, this means the selection of the iCub's mode of exploration.  
 * At present, only two basic motives drive this iCub development, curiousity and experimentation, both of them exploratory.  
 * The third main motive, social interaction, has not yet been addressed.  
 * Consequently, the present implementation of action selection is based on a very simple (trivial) function of the 
 * levels of curiousity and experimentation.  Specifically, the learning  mode is selected if the curiosity level 
 * is higher than the experimentation level; otherwise the prediction mode is selected. 
 *
 *
 * The actionSelection module has one inputs: an affective state vector containing four numbers
 *
 *    -# the current curiousity level 
 *    -# the instantaneous rate of change of curiousity level
 *    -# the current experimentation level 
 *    -# the instantaneous rate of change of experimentation level
 *
 *    This is typically input from the affectiveState module.
 *
 * The actionSelection module has one outputs: mode vector containing a single value denoting the mode of operation
 *
 *   - 0 (learning)
 *   - 1 (prediction) 
 *   - 2 (reconstruction) 
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
 * - \c from \c actionSelection.ini       \n
 *   specifies the configuration file
 *
 * - \c context \c actionSelection/conf   \n 
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c actionSelection \n         
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file e.g. \c actionSelection.ini 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c stateInPort        \c /state:i          \n 
 *   The input port name for the affective state value  
 *
 * - \c modeOutPort        \c /mode:o          \n  
 *   The input port name for the mode (corresponding to learning, prediction, reconstruction) 
 *
 * - \c mode               \c 0              \n
 *   specifies the initial mode; this is also the default mode should no state be available.
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
 * - \c /actionSelection \n
 *   This port is used to change the parameters of the module at run time or stop the module
 *   The following commands are available
 * 
 *   \c help \n
 *   \c quit
 *   \c set \c mode    \c <n>   ... set the mode  (where \c <n> is an integer number in the range 1-3)
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /rectification
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
  * - \c /actionSelection/state:i 
 *
 *
 * <b>Output ports</b>
 *
 * - \c /actionSelection 
 * - \c /actionSelection/mode:o
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - \c BufferedPort<VectorOf<double> >  \c stateInPort;           \c // \c state 
 * - \c BufferedPort<VectorOf<double> >  \c modeOutPort;            \c // \c mode  
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
 * \c actionSelection.ini  in \c $ICUB_ROOT/app/actionSelection/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>actionSelection --name actionSelection --context actionSelection/conf --from actionSelection.ini</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/actionSelection/include/iCub/actionSelection.h
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
 * 08/01/10  Began development      DV
 */ 


#ifndef __ICUB_ACTIONSELECTION_MODULE_H__
#define __ICUB_ACTIONSELECTION_MODULE_H__


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


/* actionSelection definitions */

#define LEARNING_MODE                          0
#define PREDICTION_MODE                        1
#define RECONSTRUCTION_MODE                    2

class ActionSelectionThread : public Thread
{
private:

   /* class variables */

   VectorOf<double>      *stateIn;

   int                   mode;
   double                curiousityLevel;
   double                deltaCuriousityLevel;
   double                experimentationLevel;
   double                deltaExperimentationLevel;

   bool debug;
  	    
   /* thread parameters: they are pointers so that they refer to the original variables in rectification */

   BufferedPort<VectorOf<double> >   *stateInPort;
   BufferedPort<VectorOf<double> >   *modeOutPort;
   int                               *modeValue;

public:

   /* class methods */

   ActionSelectionThread(BufferedPort<VectorOf<double> >  *stateIn,
                         BufferedPort<VectorOf<double> >  *modeOut,
                         int                              *mode
                       );
   bool threadInit();     
   void threadRelease();
   void run (); 
};


class ActionSelection:public RFModule
{
   /* port names */

   string moduleName;
   string stateInputPortName;
   string modeOutputPortName;
   string handlerPortName;

   /* class variables */

   bool debug;

   /* parameters */

   int mode;


   /* ports */

   BufferedPort<VectorOf<double> >   stateIn;
   BufferedPort<VectorOf<double> >   modeOut;
   Port                              handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   ActionSelectionThread *actionSelectionThread;


public:
   ActionSelection();
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_ACTIONSELECTION_MODULE_H__
//empty line to make gcc happy

