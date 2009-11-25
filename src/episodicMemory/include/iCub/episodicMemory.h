// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup icub_module
 * \defgroup icub_episodicMemory episodicMemory
 *
 * The episodicMemory module effects the following functionality: 
 *
 * -  when an image is presented to the memory, it attempts to recall that image; 
 * -  if a previously-stored image matches the presented image sufficiently well, the stored image is recalled.
 * -  if no previously-stored image matches sufficiently well, the presented image is stored; 
 * -  in both cases, the module outputs:
 *    -# the recalled or stored image;
 *    -# an image identification number and the match value for the presented image  
 *    -# an image identification number and the match value for whatever image was previously presented 
 * -  Alternatively, when an image identification number is presented to the memory, 
 *    the associated image is recalled and all the above four outputs are generated.
 *    If both image and image identification number are presented, the image identification number takes precedence.
 *   
 * Since images are streamed continuously, the module reads an image with a pre-set but definable frequency.
 * In addition, before reading the module checks to ensure that the iCub head is not in motion so that
 * the eyes are fixating on something.
 * 
 * The frequency at which images are read and the threshold defining whether or not an input image adequately matches a stored image
 * are both provided as module parameters, set in the episodicMemory configuration file. 
 * The thresold can also be set interactively via the episodicMemory port.
 * 
 * The episodicMemory module has the following inputs: 
 * 
 * - an input image 
 * - an input image identification number  
 * 
 *
 * The episodicMemory has the following outputs: 
 * 
 * - recalled/stored image
 * - recalled/stored image tuple (containing an image id. number(int) and a match value r (double), 0 <= r <= 1.   
 * - previously recalled/stored image tuple (containing an image id. number(int) and a match value r (double), 0 <= r <= 1. 
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
 * - \c from \c episodicMemory.ini       \n     
 *   specifies the configuration file
 * 
 * - \c context \c episodicMemory/conf   \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c episodicMemory           \n                              
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
 * - \c imageInPort         \c /image:i  \n
 *   specifies the port for input of an image
 *
 * - \c imageIdInPort       \c /imageId:i  \n
 *   specifies the port for input of an image identification number
 *
 * - \c headPort            \c /head:i   \n  
 *   specifies the input port name for the head encoder values used to check that the iCub head has stopped moving
 * 
 * - \c imageOutPort        \c /image:o  \n
 *   specifies the image output port
 *
 * - \c imageIdOutPort      \c /imageId:o  \n
 *   specifies the port for output of the image identification number corresponding to the output image
 *
 * - \c imageIdPrevOutPort  \c /imageIdPrev:o  \n
 *   specifies the port for output of the image identification number corresponding to the previous output image
 *
 * - \c database            \c episodicDatabase  \n
 *   specifies the directory name in which the database of images will be stored
 * 
 * - \c path                \c ~/iCub/app/episodicMemory  \n
 *   specifies the path the to directory in which the database of images will be stored
 *
 * - \c threshold           \c 0.75  \n
 *   specifies the value defining whether or not an input image matches adequately matches a stored image   
 *
 * - \c frequency           \c 2     \n         
 *   specifies the number of images to be streamed per second. 
 * 
 * The database parameter specifies the name of the directory in which the database of images is stored. 
 * This directory must be created before running the module. 
 * 
 * The path parameter specifies the full path to the database directory. 
 * This is where the where the database of image files is stored. 
 *
 * The threshold parameter determines which, if any, of the stored images are recalled:  images which match the input image
 * with a value that equals or exceeds the threshold are recalled and if more than one image exceeds the threshold, 
 * the image with the highest match is recalled.
 * 
 * For example, if the configuration file \c episodicMemory.ini is located in \c C:/iCub/app/episodicMemory/conf 
 * and the database is \c C:/iCub/app/episodicMemory/episodicDatabase then 
 *
 * - \c episodicMemory module must be invoked with \c --context \c episodicMemory/conf 
 * - \c episodicMemory.ini must contain \c "path C:/iCub/app/episodicMemory"
 * - the directory \c C:/iCub/app/episodicMemory/episodicDatabase must exist. 
 *
 *
 * \section portsa_sec Ports Accessed
 * 
 * None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b> 
 *
 *  - \c /episodicMemory
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *    \c set \c thr    \c <n>   ... set the threshold for image recall (where \c <n> is a real number in the range 0-1)
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /episodicMemory
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *
 * - \c /episodicMemory/image:i
 * - \c /episodicMemory/imageId:i
 * - \c /episodicMemory/head:i \n
 *   This port needs to be connected to \c /icub/head/state:o to get the gaze values.
 *
 * <b>Output ports</b> 
 *
 *  - \c /episodicMemory
 *  - \c /episodicMemory/image:o
 *  - \c /episodicMemory/imageId:o
 *  - \c /episodicMemory/imageIdPrev:o
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 *
 * <b>I/O Port Types & Naming</b> 
 *
 * - \c BufferedPort<ImageOf<PixelRgb> > \c imageInPort;
 * - \c BufferedPort<Bottle>             \c imageIdInPort;          \c //int \c image_id 
 * - \c BufferedPort<ImageOf<PixelRgb> > \c imageOutPort;
 * - \c BufferedPort<Bottle>             \c imageIdOutPort;         \c //int image_id, \c double match_value 
 * - \c BufferedPort<Bottle>             \c imageIdPrevOutPort;     \c //int image_id, \c double match_value 
 * - \c BufferedPort<Vector>             \c headPort;            \n
 *
 * \section in_files_sec Input Data Files
 *
 * \c episodicDatabase  (see above)
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c episodicMemory.ini (see above)
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * \c episodicMemory \c --context \c episodicMemory/conf  \c --from episodicMemory.ini
 *
 * \author David Vernon 
 * (based on code written by Alberto Bietti, Logan Niehaus, and Gionvanni Saponaro for the autoAssociativeMemory module)
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/episodicMemory/include/iCub/episodicMemoryModule.h
**/
  
/*
 * Audit Trail
 * -----------
 *
 * 02/11/09  Started adapting the original autoAssociativeMemory to create the current episodicMemory module
 * 04/11/09  Completed the episodicMemory module
 */ 

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Alberto Bietti, Logan Niehaus, Giovanni Saponaro, David Vernon
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


#ifndef __ICUB_episodicMemory_MODULE_H__
#define __ICUB_episodicMemory_MODULE_H__

//yarp
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

/*
 * Histogram Matching data holder class. Stores information that will be used by the HistMatch implementation of the episodicMemory. 
 */

class HistMatchData
{
private:
    std::vector<ImageOf<PixelRgb> > imgs; //image database (see episodicMemory specification)
    double threshold;                     //threshold (see episodicMemory specification)
    double matchValue;                    //returned match value (see episodicMemory specification)
    string databaseName;                  //name of the database folder in which memory images will be stored
    string databaseContext;               //database context

public:
    Semaphore thrMutex;                   //threshold semaphore
    Semaphore imgMutex;                   //image semaphore

    /** HistMatchData constructor */  
    HistMatchData();
    /** HistMatchData destructor */
    ~HistMatchData();

    
    vector<ImageOf<PixelRgb> >& images();
    void setThreshold(double);
    void getThreshold(double&);

    void setDatabaseContext(string);
    string getDatabaseContext();
    void setDatabaseName(string);
    string getDatabaseName();

    void loadDatabase();

};


class EpisodicMemoryThread : public RateThread
{
private:

   /* class variables */

   bool debug;

   float             roll, pitch, yaw;
   float             rollNew, pitchNew, yawNew;
   Vector            *encoderPositions;
   ImageOf<PixelRgb> *imgIn;
   Bottle            *imgInId;
   int               imageId;
   int               imageIdPrevious;
   double            imageMatch;
   double            imageMatchPrevious;
   int               matchId;
   std::vector<ImageOf<PixelRgb> >::iterator it;
   ImageOf<PixelRgb> matchImage;
   bool              found;
   double            matchValue;
   HistMatchData     data;


   /* thread parameters: they are pointers so that they refer to the original variables in imageSource */

   BufferedPort<ImageOf<PixelRgb> > *imageInPort;
   BufferedPort<Bottle>             *imageIdInPort;
   BufferedPort<ImageOf<PixelRgb> > *imageOutPort;
   BufferedPort<Bottle>             *imageIdOutPort;
   BufferedPort<Bottle>             *imageIdPrevOutPort;
   BufferedPort<Vector>             *robotPort;
   string                           *databaseNameValue;
   string                           *pathValue;
   double                           *thresholdValue;

public:

   /* class methods */

   EpisodicMemoryThread (BufferedPort<ImageOf<PixelRgb> > *imageIn,
                         BufferedPort<Bottle>             *imageIdIn,
                         BufferedPort<ImageOf<PixelRgb> > *imageOut,
                         BufferedPort<Bottle>             *imageIdOut,
                         BufferedPort<Bottle>             *imageIdPrevOut,
                         BufferedPort<Vector>             *robotPort,
                         string                           *databaseName,
                         string                           *path,
                         double                           *threshold,
                         int                              frequency);
   bool threadInit();     
   void threadRelease();
   void run(); 

};


class EpisodicMemory : public RFModule
{
private:
   /* class variables */

   bool debug; 

   /* port names */

   string imageInPortName;
   string imageIdInPortName;
   string imageOutPortName;
   string imageIdOutPortName;
   string imageIdPrevOutPortName;
   string handlerPortName;
   string robotPortName;  
   string moduleName;

   /* parameters */

   string databaseName;
   string path;
   double threshold;
   int    frequency;

   // ports

   BufferedPort<ImageOf<PixelRgb> > imageIn;
   BufferedPort<Bottle>             imageIdIn;
   BufferedPort<ImageOf<PixelRgb> > imageOut;
   BufferedPort<Bottle>             imageIdOut;
   BufferedPort<Bottle>             imageIdPrevOut;
   BufferedPort<Vector>             robotPort;
   Port                             handlerPort; 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   EpisodicMemoryThread *episodicMemoryThread;

public:
   virtual bool configure(yarp::os::ResourceFinder &rf);
   virtual bool updateModule();
   virtual bool interruptModule();
   virtual bool close();
   virtual double getPeriod();
   virtual bool respond(const Bottle& command, Bottle& reply);
};

#endif // __ICUB_episodicMemory_MODULE_H__
//empty line to make gcc happy

