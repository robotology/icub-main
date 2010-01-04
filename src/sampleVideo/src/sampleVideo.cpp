// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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

/* Audit Trail 
 * -----------
 *
 * First version of sampleVideo implemented
 * David Vernon 02/01/10
 */

 

// iCub
#include <iCub/sampleVideo.h>

 
 

bool SampleVideo::configure(yarp::os::ResourceFinder &rf)
{

    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                            Value("sampleVideo"), 
                            "module name (string)").asString(); 

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());

  
    // parse parameters or assign default values (append to getName=="/sampleVideo")

    imageInPortName      = "/";
    imageInPortName     += getName(
                           rf.check("imageInPort",
				           Value("/image:i"),
				           "Input image port (string)").asString()
                           );


    imageOutPortName     = "/";
    imageOutPortName    += getName(
                           rf.check("imageOutPort",
				           Value("/image:o"),
				           "Output image port (string)").asString()
                           );

    frequency            = rf.check("frequency",
                           Value(2),
                           "initial period for image acquisition (int)").asInt();


    printf("sampleVideo: parameters are \n%s\n%s\n%d\n\n",
           imageInPortName.c_str(),
           imageOutPortName.c_str(), 
           frequency);

    // create sampleVideo ports

    imageIn.open(imageInPortName.c_str());
    imageOut.open(imageOutPortName.c_str());
    
   /* create the thread and pass pointers to the module parameters */

   sampleVideoThread = new SampleVideoThread(&imageIn,
                                             &imageOut,
                                             (int)(1000 / frequency));

   /* now start the thread to do the work */

   sampleVideoThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true;
}

bool SampleVideo::updateModule()
{		
    return true;
}

bool SampleVideo::interruptModule()
{
    // interrupt ports gracefully

    imageIn.interrupt();
    imageOut.interrupt();
 
    return true;	
}

bool SampleVideo::close()
{
    cout << "Closing SampleVideo...\n\n";

    // close sampleVideo ports
    
    //_portThresholdIn->close();   
    
    imageIn.close();
    imageOut.close();

    /* stop the thread */

    sampleVideoThread->stop();
   
    return true;
}


//module periodicity (seconds), called implicitly by module

double SampleVideo::getPeriod()

{
    return 0.1; //module periodicity (seconds)
}

 


SampleVideoThread::SampleVideoThread(BufferedPort<ImageOf<PixelRgb> > *imageIn,
                                     BufferedPort<ImageOf<PixelRgb> > *imageOut,
                                     int                              period) : RateThread(period)
{
   debug = false;

   imageInPort          = imageIn;
   imageOutPort         = imageOut;

}

bool SampleVideoThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;

    return true;
}

void SampleVideoThread::run(){

   /* the head has stabilized ... proceed to read an image and recall/store it */
 
   do {
      imgIn = imageInPort->read(true);
   } while (imgIn == NULL);
   	
   ImageOf<PixelRgb>& img = *imgIn;

   imageOutPort->prepare() = img;
   imageOutPort->write();
 
}

void SampleVideoThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */


}


