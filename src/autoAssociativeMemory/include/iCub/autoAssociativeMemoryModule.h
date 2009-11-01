// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup icub_module
 * \defgroup icub_autoAssociativeMemory autoAssociativeMemory
 *
 * The auto-associative memory (AAM) module - autoAssociativeMemory - effects the following functionality: 
 *
 * - when an image is presented to the memory, it attempts to recall that image; 
 * - if a previously-stored image matches the presented image sufficiently well, the stored image is recalled; 
 * - if no previously-stored image matches sufficiently well, the presented image is stored; 
 * 
 * Since images are streamed continuously, processing an image (i.e. reading it from the input port and attempting to recall it from the memory) 
 * is triggered by the presence of the threshold value on the input port. The next image to be read and stored/recalled is only processed 
 * when a new threshold is written to that port. 
 * 
 * 
 * The AAM has the following inputs: 
 * 
 * - an input image 
 * - a match threshold t, 0 ? t ? 1; stored images which equal or exceed the threshold are recalled (if more than one image exceeds the threshold, the image with the highest match is recalled) 
 * 
 *
 * The AAM has the following outputs: 
 * 
 * - an output image 
 * - a tuple containing an image id. number and a match value r, 0 ? r ? 1. These are type integer and double respectively. * * 
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * Command Line Parameters 
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - from autoAssociativeMemory.ini       
 *   specifies the configuration file
 * - context autoAssociativeMemory/conf   
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 * - name aam                             
 *   specifies the name of the module (used to form the stem of module port names)
 * - robot icub                           
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * Configuration File Parameters 
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - portImageIn         /image:i
 * - portThresholdIn     /threshold:i
 * - portImageOut        /image:o
 * - portValueOut        /value:o
 * - database            defaultDatabase
 * - path                ~/iCub/app/autoAssociativeMemory
 * - threshold           0.6
 * 
 * The database parameter specifies the name of the directory in which the database of images is stored. 
 * This directory must be created before running the module. 
 * 
 * The path parameter specifies the full path to the database directory. 
 * This is where the where the database of image files is stored. 
 * 
 * For example, if the configuration file autoAssociativeMemory.ini is located in C:/iCub/app/demoAAM/conf 
 * and the database is C:/iCub/app/demoAAM/defaultDatabase then 
 *
 * - autoAssociativeMemory module must be invoked with --context demoAAM/conf 
 * - autoAssociativeMemory.ini must contain "path C:/iCub/app/demoAAM"
 * - the directory C:/iCub/app/demoAAM/defaultDatabase must exist. 
 *
 *
 * \section portsa_sec Ports Accessed
 * 
 * None
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - /aam
 *    This port is used to change the parameters of the module at run time or stop the module
 *    This functionality is not yet used but it may come in useful later on
 *    if/when we wish to change the parameters of the module at run time
 *    For now, just echo all received messages.
 *
 *  - /aam/image:i
 *  - /aam/threshold:i
 *
 * Output ports
 *
 *  - /aam
 *    see above
 *  - /aam/image:o
 *  - /aam/value:o
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 *
 * I/O Port Types & Naming   
 *
 * - BufferedPort<ImageOf<PixelRgb> > portImageIn;
 * - BufferedPort<Bottle>             portThresholdIn;        //Double Threshold 
 * - BufferedPort<ImageOf<PixelRgb> > portImageOut;
 * - BufferedPort<Bottle>             portIdMatchValuesOut;   //Int image_id, Double match_value
 *
 *
 * \section in_files_sec Input Data Files
 *
 * defaultDatabase  (see above)
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * autoAssociativeMemory.ini (see above)
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * autoAssociativeMemory --context autoAssociativeMemory/conf  --from autoAssociativeMemory.ini
 *
 * \author Alberto Bietti, Logan Niehaus, Gionvanni Saponaro, David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/autoAssociativeMemory/include/iCub/autoAssociativeMemoryModule.h
**/
  
/*
 * Audit Trail
 * -----------
 *
 * 22/07/09  Started work on development 
 * 25/07/09  Finished work on algorithm and modularization of the code
 * 27/07/09  Began documentation and evaluation of standards compliance
 * 16/08/09  Migrated to RFModule class and implemented necessary changes to ensure compliance with iCub standards 
 * 26/08/09  Completed compliance with standards and changed to new convention for handling port names
 *           (module name has no / prefix and port names _always_ have / prefix)
 * 14/10/09  Normalized histograms before computing the histogram intersection. 
 *
 */ 

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Alberto Bietti, Logan Niehaus, Giovanni Saponaro
 * Maintained by David Vernon
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


#ifndef __ICUB_AAM_MODULE_H__
#define __ICUB_AAM_MODULE_H__

#include <ace/OS.h>

//yarp
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/all.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//opencv
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

/*
 * Histogram Matching data holder class. Stores information that will be used by the HistMatch implementation of the AAM. 
 */

class HistMatchData
{
private:
    std::vector<ImageOf<PixelRgb> > imgs; //image database (see AAM specification)
    double threshold;                     //threshold (see AAM specification)
    double matchValue;                    //returned match value (see AAM specification)
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


/** 
    Threshold Receiver class. Important for non-blocking callback functionality of the module
 */
class ThresholdReceiver : public BufferedPort<Bottle>
{
private:
    HistMatchData* data;

    BufferedPort<ImageOf<PixelRgb> >* imgPortIn;
    BufferedPort<ImageOf<PixelRgb> >* imgPort;
    BufferedPort<Bottle>* matchPort;

    /** Callback for threshold receiver.
        Function is called when a new threshold value becomes available. */
    virtual void onRead(Bottle&);

public:
    ThresholdReceiver(HistMatchData* d, BufferedPort<ImageOf<PixelRgb> >* iPortIn, BufferedPort<ImageOf<PixelRgb> >* iPort, BufferedPort<Bottle>* mPort);

};

/**
    AssociativeMemoryModule Class. Brief AAM module, as outlined in the iCub cognitive architecture spec.
        This particular implementation uses color histogram intersection methods
        to recall images from stored memories. For more information on this spec,
        see the Wiki.
 */
class AutoAssociativeMemoryModule : public RFModule
{
private:
    // port names ... changed from ConstString to string DV 26/8/2009
    string _namePortImageIn;
    string _namePortThresholdIn;
    string _namePortImageOut;
    string _namePortValueOut;
    string _nameHandlerPort;

    string moduleName;

    HistMatchData data;

    // ports
    BufferedPort<ImageOf<PixelRgb> > _portImageIn;
    ThresholdReceiver* _portThresholdIn;
    BufferedPort<ImageOf<PixelRgb> > _portImageOut;
    BufferedPort<Bottle> _portValueOut;
    // inputs
    ImageOf<PixelRgb>* _inputImg;
    double _threshold;
    // outputs
    ImageOf<PixelRgb>* _outputImg;
    double _matchResult;
  
	//a port to handle messages
    Port handlerPort; 

public:
	virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual bool close();
	virtual double getPeriod();
	virtual bool respond(const Bottle& command, Bottle& reply);
};

#endif // __ICUB_AAM_MODULE_H__
//empty line to make gcc happy

