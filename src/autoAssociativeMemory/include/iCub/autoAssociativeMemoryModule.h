// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Alberto Bietti, Logan Niehaus, Giovanni Saponaro 
 * email:   <firstname.secondname>@robotcub.org
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

/**
*
* @ingroup icub_module
* \defgroup icub_autoAssociativeMemory autoAssociativeMemory
*
* description of the module goes here ....
*
*
* \section lib_sec Libraries
* YARP.
*
* \section parameters_sec Parameters
* 
* --file autoAssociativeMemory.ini: specifies parameter file
*
* \section portsa_sec Ports Accessed
* 
* - /icub/cam/left
*                      
* \section portsc_sec Ports Created
*
*  Input ports
*
*  - /aam/image:i
*  - /aam/threshold:i
*
* Output ports
*
*  - /aam/image:o
*  - /aam/value:o
*
* \section in_files_sec Input Data Files
* None
*
* \section out_data_sec Output Data Files
* None
*
* \section conf_file_sec Configuration Files
* None
* 
* \section tested_os_sec Tested OS
* Linux
*
* \section example_sec Example Instantiation of the Module
* autoAssociativeMemory --file autoAssociativeMemory.ini
*
* \author Alberto Bietti, Logan Niehaus, Gionvanni Saponaro
* 
* Copyright (C) 2009 RobotCub Consortium
* 
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
* 
*This file can be edited at src/autoAssociativeMemory/src/autoAssociativeMemoryModule.cpp.
**/
 
/*
Audit Trail
-----------

22/07/09  Started work on development 

25/07/09  Finished work on algorithm and modularization of the code

27/07/09  Began documentation and evaluation of standards compliance

*/ 


#ifndef __ICUB_AAM_MODULE_H__
#define __ICUB_AAM_MODULE_H__

#include <ace/OS.h>

//yarp
#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>
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
    Histogram Matching data holder class. Stores information that will be used by the HistMatch implementation of the AAM. 

*/

class HistMatchData
{
private:
    std::vector<ImageOf<PixelRgb> > imgs; //image database (see AAM specification)
    double threshold; //threshold (see AAM specification)
    double matchValue;    //returned match value (see AAM specification)
    string databaseName;  //name of the database folder in which memory images will be stored
    string databaseContext;   //database context

public:
    Semaphore thrMutex;   //threshold semaphore
    Semaphore imgMutex;   //image semaphore

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
class AutoAssociativeMemoryModule : public Module
{
private:
    // port names
    ConstString _namePortImageIn;
    ConstString _namePortThresholdIn;
    ConstString _namePortImageOut;
    ConstString _namePortValueOut;

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
  
public:
    virtual bool open(Searchable &config);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual bool close();
};

#endif // __ICUB_AAM_MODULE_H__
