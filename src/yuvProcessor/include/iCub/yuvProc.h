/** 
 * @ingroup icub_module
 *
 * \defgroup icub_yuvProcessor yuvProcessor
 *
 * Functionally, receives the rectified Y (intensity) channel from Francesco's Rea Coulour Processor Module and performs centre-surround processing and construction of an intensity uniqueness map (intensity saliency) via a difference-of-Gaussian pyramid filter bank. 
 * It also receives rectified U and V chrominance channels from Francesco's Rea Coulour Processor Module and performs centre-surround processing and construction of a colour chrominance uniqueness map (colour saliency) via a difference-of-Gaussian pyramid filter bank.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP
 * IPP
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters <\b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c yuvProc.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c yuvProc/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c yuvProc \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * <b>Configuration File Parameters </b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c YUVPort \c /Y/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c YUVPort \c /U/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c YUVPort \c /V/image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c YPort \c /Y/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c UVPort \c /UV/image:o \n  
 *   specifies the output port name (this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /yuvProc \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /yuvProc
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /yuvProc/Y/image:i \n
 *  - \c /yuvProc/U/image:i \n
 *  - \c /yuvProc/V/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /yuvProc \n
 *    see above
 *
 *  - \c /yuvProc/Y/image:o \n
 *  - \c /yuvProc/UV/image:o \n
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelMono> >   \c myInputPortY; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c myInputPortU; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c myInputPortV; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c YPort;   
 * \c BufferedPort<ImageOf<PixelMono> >   \c UVPort;       
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
 * \c yuvProc.ini  in \c $ICUB_ROOT/app/yuvProc/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10 and Debian Stable 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>YUVProcessor --name yuvProc --context yuvProc/conf --from yuvProc.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/yuvProc/include/iCub/yuvProc.h
 * 
 */


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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

#ifndef __ICUB_YUV_PROC_H__
#define __ICUB_YUV_PROC_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
//ipp includes
#include <ipp.h>
//local includes
#include "iCub/centsur.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  
class YUVThread : public Thread
{
private:

    /* class variables */
    ImageOf<PixelMono> *imgY;
  	ImageOf<PixelMono> *imgU;    
    ImageOf<PixelMono> *imgV;

    /* thread parameters: they are pointers so that they refer to the original variables */

    BufferedPort<ImageOf<PixelMono> >  *imageInputPortY;
    BufferedPort<ImageOf<PixelMono> >  *imageInputPortU;
    BufferedPort<ImageOf<PixelMono> >  *imageInputPortV;

    BufferedPort<ImageOf<PixelMono> > *imageOutPortY;
    BufferedPort<ImageOf<PixelMono> > *imageOutPortUV;  

    IppiSize srcsize;
    int width, height;
    int ncsscale;
    bool init;
    //int psb, psb4, psb3, ycs_psb, col_psb, psb_32f;
    int y_psb, u_psb, v_psb;
    int ycs_psb, col_psb, psb_32f;
    //Ipp8u *colour_in, *colour, *ycs_out, *yuva_orig, *y_orig, *u_orig, *v_orig, *tmp; 
    Ipp8u *ycs_out, *y_orig, *u_orig, *v_orig, *colcs_out;  
    //Ipp8u** pyuva;
    CentSur * centerSurr;
    Ipp32f *cs_tot_32f;
    Ipp32f min,max;

public:

    /* class methods */
    YUVThread(BufferedPort<ImageOf<PixelMono> > *inputPortY,BufferedPort<ImageOf<PixelMono> > *inputPortU, BufferedPort<ImageOf<PixelMono> > *inputPortV, BufferedPort<ImageOf<PixelMono> > *outPortY, BufferedPort<ImageOf<PixelMono> > *outPortUV);
    
    void initAll();
    bool threadInit();     
    void threadRelease();
    void run(); 
};

class yuvProc:public RFModule
{
    /* module parameters */
    string moduleName; 
    //string inputPortName;
    string inputPortNameY;
    string inputPortNameU;
    string inputPortNameV;

    string outputPortNameY;  
    string outputPortNameUV;    
    string handlerPortName;

    /* class variables */
    BufferedPort<ImageOf<PixelMono> > inputPortY;  // input port
    BufferedPort<ImageOf<PixelMono> > inputPortU;
    BufferedPort<ImageOf<PixelMono> > inputPortV;
    BufferedPort<ImageOf<PixelMono> > outPortY;  // intensity output port
    BufferedPort<ImageOf<PixelMono> > outPortUV; // chrominance output port
    Port handlerPort;      //a port to handle messages 
    
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    YUVThread *yuvThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif
//empty line to make gcc happy
