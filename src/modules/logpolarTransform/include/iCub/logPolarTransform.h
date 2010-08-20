// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/** 
 * @ingroup icub_module
 *
 * \defgroup icub_logpolarTransform logpolarTransform
 *
 * Perform a log-polar transform on an input image and generate a transformed output image.
 * The direction of the transform, from Cartesian to log-polar or vice versa, is specified by a flag in the configuration file.
 * The default direction is Cartesian to log-polar.
 * The parameters of the transform, i.e. the number of angles, the number of rings, and the overlap of the receptive fields are specified in the configuration file.
 * The default number of angles and rings is 252 and 152, respectively.  The default overlap is 1.0.
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
 * (e.g. \c --from file.ini). The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c logpolarTransform.ini   \n    
 *   specifies the configuration file
 *
 * - \c context \c LogPolarTransform/conf  \n
 *   specifies the sub-path from \c $ICUB_ROOT/app to the configuration file
 *
 * - \c name \c LogPolarTransform  \n        
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c imageInputPort \c /image:i   \n
 *   specifies the input port name (this string will be prefixed by \c /LogPolarTransform
 *   or whatever else is specifed by the name parameter
 *
 * - \c imageOutputPort \c /image:o \n    
 *   specifies the input port name (this string will be prefixed by \c /LogPolarTransform
 *   or whatever else is specifed by the name parameter
 *
 * - \c direction \c CARTESIAN2LOGPOLAR \n
 *   specifies the direction of the tranform; the alternative direction is LOGPOLAR2CARTESIAN
 *   
 * - \c angles \c 252  \n           
 *   specifies the number of receptive fields per ring (i.e. the number of samples in the theta/angular dimension); 
 *   required for CARTESIAN2LOGPOLAR transform direction
 *
 * - \c rings \c 152 \n            
 *   specifies the number of rings (i.e. the number of samples in the r dimension);
 *   required for CARTESIAN2LOGPOLAR transform direction
 *
 * - \c xsize \c 320 \n            
 *   specifies the number of samples in the X dimension; 
 *   required for LOGPOLAR2CARTESIAN transform direction
 *
 * - \c ysize \c 240  \n          
 *   specifies the number samples in the Y dimension; 
 *   required for LOGPOLAR2CARTESIAN transform direction
 *
 * - \c overlap \c 1.0     \n        
 *   specifies the relative overlap of each receptive field
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /logpolarTransform \n
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available
 * 
 *  -  help \n
 *  -  quit \n
 *  
 *    Note that the name of this port mirrors whatever is provided by the \c  --name \c parameter \c value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: yarp rpc /LogPolarTransform
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - /logpolarTransform/image:i
 *
 * <b>Output ports</b>
 *
 *  - \c /logpolarTransform
 *    see above
 *
 *  - \c /logpolarTransform/image:o
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - \c BufferedPort<ImageOf<PixelRgb> >   \c imageInputPort;     
 * - \c BufferedPort<ImageOf<PixelRgb> >   \c imageOutputPort;       
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
 * \c logpolarTransform.ini  in \c $ICUB_ROOT/app/logpolarTransform/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>logpolarTransform --name logPolarTransform --context logpolarTransform/conf --from lp2cart.ini  </tt>
 * <tt>logpolarTransform --name logpolarTransform --context logpolarTransform/conf --from cart2lp.ini  </tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at $ICUB_ROOT/src/logpolarTransform/include/iCub/logPolarTransform.h
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
 * 20/09/09  Began development  DV
 * 18/08/10  Removed dependency on fourierVision, simpler.  GM
 * 18/08/10  Made flexbible input. GM
 */ 

/**
 * @file logPolarTransform.h
 * @brief definition of the logpolar transform module classes (generic logpolar module).
 */

#ifndef __ICUB_LOGPOLARTRANSFORM_MODULE_H__
#define __ICUB_LOGPOLARTRANSFORM_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 

/* Log-Polar includes */
#include <iCub/RC_DIST_FB_logpolar_mapper.h>


class LogPolarTransformThread : public yarp::os::Thread
{
private:
    /* thread parameters: they are pointers so that they refer to the original variables in LogPolarTransform */
    yarp::os::BufferedPort<yarp::sig::FlexImage> *imagePortIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *imagePortOut;   
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;

    int *directionValue;     
    int *anglesValue;     
    int *ringsValue;   
    int *xSizeValue;
    int *ySizeValue;
    double *overlapValue;     

    iCub::logpolar::logpolarTransform trsf;

public:
    LogPolarTransformThread(yarp::os::BufferedPort<yarp::sig::FlexImage > *imageIn,  yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *imageOut, 
                            int *direction, int *x, int *y, int *angles, int  *rings, double *overlap);
    bool threadInit();     
    void threadRelease();
    void run(); 

    bool allocLookupTables(int which, int necc, int nang, int w, int h, double overlap);
    bool freeLookupTables();

    virtual void onStop() {
        imagePortIn->interrupt();
        imagePortOut->interrupt();
        imagePortIn->close();
        imagePortOut->close();
    }
};

enum {
    CARTESIAN2LOGPOLAR = 0,
    LOGPOLAR2CARTESIAN = 1
};

class LogPolarTransform : public yarp::os::RFModule
{
    //int debug;

    std::string moduleName;
    std::string robotPortName;  
    std::string inputPortName;
    std::string outputPortName;  
    std::string handlerPortName;
    std::string transformDirection;
    int    direction;                // direction of transform
    int    numberOfAngles;           // theta samples
    int    numberOfRings;            // r samples
    int    xSize;                    // x samples
    int    ySize;                    // y samples
    double  overlap;                 // overlap of receptive fields

    /* class variables */

    yarp::os::BufferedPort<yarp::sig::FlexImage> imageIn;      // input port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageOut;     // output port
    yarp::os::Port handlerPort;                              //a port to handle messages 

    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    LogPolarTransformThread *logPolarTransformThread;

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // __ICUB_LOGPOLARTRANSFORM_MODULE_H__
//empty line to make gcc happy

