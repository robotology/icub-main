// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup icub_module
 * \defgroup icub_attentionNetworkTest attentionNetworkTest
 *
 * The attentionNetworkTest module generates a sequence of images that implement the Attention Network Test (ANT) 
 * developed by Michael Posner and his coworkers (see Fan et al. 2002, below).
 * 
 * J. Fan, B. D. McCandliss, T. Sommer, A. Raz, and M. I. Posner (2002). 
 * Testing the Efficiency and Independence of Attentional Networks, Journal of Cognitive Neuroscience, 14:3, 340 – 347.
 * 
 * The current version has some restrictions:
 * - The module doesn't provide a facility for interaction with a participant and, specifically, for signalling when
 *   the orientation of the target is recognized.  Consequently, the current sequence displays the target and flanker image 
 *   for the full allowed reaction time. ANT should only present this image until the participant responds after 
 *   which a post-target fixation image is shown.  
 * - There are 48 distinct trials (4 cue conditions x 2 target lcoations x 2 target directions x 3 flanker conditions).
 *   ANT presents these trials in random order, with two repetitions (96 trials in total).
 *   However, currently, the modules presents the 48 trials in a fixed order and it repeats this set of trial continuously.
 *   The pattern of presentation cycles though each variable as follows:  
 *   -# for each cue condition
 *   -# for each location
 *   -# for each direction
 *   -# vary flanker condition
 *   
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * <b>Command Line Parameters </b> 
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from \c file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c from \c attentionNetworkTest.ini       \n     
 *   specifies the configuration file
 * 
 * - \c context \c attentionNetworkTest/conf   \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c attentionNetworkTest           \n                              
 *   specifies the name of the module (used to form the stem of module port names)
 * 
 * - \c robot \c icub                    \n                         
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters </b> 
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c ANTimageOutPort \c /ANTimage:o  \n
 *   specifies the image output port
 *
 * - \c d1           \c 1600  \n
 *   specifies the duration in milliseconds of the fixation image
 *
 * - \c d2           \c 100  \n
 *   specifies the duration in milliseconds of the cue image
 *
 * - \c d3           \c 400  \n
 *   specifies the duration in milliseconds of the fixation image
 *
 * - \c d4           \c 1700  \n
 *   specifies the duration in milliseconds of the target image
 *
 * - \c d5           \c 0  \n
 *   specifies the duration in milliseconds of the fixation image  (4000 - d1 - d2 - d3 - d4); 
 *   note that d3 is maximum reaction time RT
 *
 * - \c database              \c ANTdatabase  \n
 *   specifies the directory name in which the database of images will be stored
 * 
 * - \c path                  \c ~/iCub/app/attentionNetworkTest  \n
 *   specifies the path the to directory in which the database of images will be stored
 *
 * The database parameter specifies the name of the directory in which the database of images is stored. 
 * This directory must be created before running the module. 
 *  
 * The path parameter specifies the full path to the database directory. 
 * This is where the where the database of image files is stored. 
 *
 * For example, if the configuration file \c attentionNetworkTest.ini is located in \c C:/iCub/app/attentionNetworkTest/conf 
 * and the database is \c C:/iCub/app/attentionNetworkTest/ANTdatabase then 
 *
 * - \c attentionNetworkTest module must be invoked with \c --context \c attentionNetworkTest/conf 
 * - \c attentionNetworkTest.ini must contain \c "path C:/iCub/app/attentionNetworkTest"
 * - the directory \c C:/iCub/app/attentionNetworkTest/ANTdatabase must exist. 
 *
 * The database must comprise the following ANT images 
 *
 * - ANT00   fixation
 * - ANT01   cue centre
 * - ANT02   cue top and bottom
 * - ANT03   cue top
 * - ANT04   cue bottom
 * - ANT05   target neutral top right
 * - ANT06   target neutral bottom right
 * - ANT07   target neutral top left
 * - ANT08   target neutral bottom left
 * - ANT09   target congurent top right
 * - ANT10   target congurent bottom right
 * - ANT11   target congurent top left
 * - ANT12   target congurent bottom left
 * - ANT13   target incongurent top right
 * - ANT14   target incongurent bottom right
 * - ANT15   target incongurent top left
 * - ANT16   target incongurent bottom left
 *
 * \section portsa_sec Ports Accessed
 * 
 * None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b> 
 *
 *  None
 *
 * <b>Output ports</b> 
 *
 *  - \c /attentionNetworkTest/ANTimage:o
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 *
 * <b>I/O Port Types & Naming</b> 
 *
 * - \c BufferedPort<ImageOf<PixelRgb> > \c ANTimageOutPort;
 *
 * \section in_files_sec Input Data Files
 *
 * \c ANTdatabase  (see above)
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c attentionNetworkTest.ini (see above)
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * \c attentionNetworkTest \c --context \c attentionNetworkTest/conf  \c --from attentionNetworkTest.ini
 *
 * \author David Vernon 
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/attentionNetworkTest/include/iCub/attentionNetworkTestModule.h
**/
  
/*
 * Audit Trail
 * -----------
 *
 * 15/01/10  Initial version DV
 */ 

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   <david.vernon>@robotcub.org
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


#ifndef __ICUB_attentionNetworkTest_MODULE_H__
#define __ICUB_attentionNetworkTest_MODULE_H__

//yarp
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <sys/types.h>
#include <sys/timeb.h>


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


void pause(int ms); // defined in fourierVision.lib

#define FPS 10 // frames per second; the number of images sent per second

/*
 * Histogram Matching data holder class. Stores information that will be used by the ANT implementation of the attentionNetworkTest. 
 */

class ANTData
{
private:
    std::vector<ImageOf<PixelRgb> > imgs; //image database (see attentionNetworkTest specification)
    string databaseName;                  //name of the database folder in which memory images will be stored
    string databaseContext;               //database context

public:
    Semaphore thrMutex;                   //threshold semaphore
    Semaphore imgMutex;                   //image semaphore

    /** ANTData constructor */  
    ANTData();
    /** ANTData destructor */
    ~ANTData();

    
    vector<ImageOf<PixelRgb> >& images();

    void setDatabaseContext(string);
    string getDatabaseContext();
    void setDatabaseName(string);
    string getDatabaseName();

    void loadDatabase();

};


class AttentionNetworkTestThread : public Thread
{
private:

   /* class variables */

   bool debug;

   std::vector<ImageOf<PixelRgb> >::iterator it;
   ImageOf<PixelRgb> ANTimage;
   bool              found;
   int               imageId; 
   int               cueCondition;
   int               location;
   int               direction;
   int               flankerCondition;
   ANTData           data;


   /* thread parameters: they are pointers so that they refer to the original variables in imageSource */

   BufferedPort<ImageOf<PixelRgb> > *ANTimageOutPort;
   string                           *databaseNameValue;
   string                           *pathValue;
   int                              *d1Value;
   int                              *d2Value;
   int                              *d3Value;
   int                              *d4Value;
   int                              *d5Value;

public:

   /* class methods */

   AttentionNetworkTestThread (BufferedPort<ImageOf<PixelRgb> > *ANTimageOut,
                               string                           *databaseName,
                               string                           *path,
                               int                              *d1,
                               int                              *d2,
                               int                              *d3,
                               int                              *d4,
                               int                              *d5);

   bool threadInit();     
   void threadRelease();
   void run(); 
};


class AttentionNetworkTest : public RFModule
{
private:
   /* class variables */

   bool debug; 

   /* port names */

   string ANTimageOutPortName;
   string moduleName;

   /* parameters */

   string databaseName;
   string path;
   int d1;
   int d2;
   int d3;
   int d4;
   int d5;


   // ports

   BufferedPort<ImageOf<PixelRgb> > ANTimageOut;

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   AttentionNetworkTestThread *attentionNetworkTestThread;

public:
   virtual bool configure(yarp::os::ResourceFinder &rf);
   virtual bool updateModule();
   virtual bool interruptModule();
   virtual bool close();
   virtual double getPeriod();
};

#endif // __ICUB_attentionNetworkTest_MODULE_H__
//empty line to make gcc happy

