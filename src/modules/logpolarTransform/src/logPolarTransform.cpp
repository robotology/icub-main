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


/*
 * Audit Trail
 * -----------
 * 20/09/09  Began development   DV
 * 18/08/10  Rewrite by GM, removed dependencies from unnecessary libraries.
 */ 

/**
 * @file logPolarTransform.cpp
 * @brief implementation of the logpolar transform module classes (generic logpolar module).
 */
#include <memory.h>

#include "iCub/logPolarTransform.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

using namespace iCub::logpolar;

bool LogPolarTransform::configure(yarp::os::ResourceFinder &rf)
{    
   /* Process all parameters from both command-line and .ini file */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("logpolarTransform"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   inputPortName         = "/";
   inputPortName        += getName(
                           rf.check("imageInputPort", 
                           Value("/image:i"),
                           "Input image port (string)").asString()
                           );
   
   outputPortName        = "/";
   outputPortName       += getName(
                           rf.check("imageOutputPort", 
                           Value("/image:o"),
                           "Output image port (string)").asString()
                           );

   /* get the direction of the transform */

   transformDirection    = rf.check("direction",
                           Value("CARTESIAN2LOGPOLAR"),
                           "Key value (int)").asString();

   cout << "Configuration of logpolar " << transformDirection << endl;

   if (transformDirection == "CARTESIAN2LOGPOLAR") {
      direction = CARTESIAN2LOGPOLAR;
   }
   else {
      direction = LOGPOLAR2CARTESIAN;
   }

   /* get the number of angles */

   numberOfAngles        = rf.check("angles",
                           Value(252),
                           "Key value (int)").asInt();

   /* get the number of rings */

   numberOfRings         = rf.check("rings",
                           Value(152),
                           "Key value (int)").asInt();
 
   /* get the size of the X dimension */

   xSize                 = rf.check("xsize",
                           Value(320),
                           "Key value (int)").asInt();

   /* get the size of the Y dimension */

   ySize                 = rf.check("ysize",
                           Value(240),
                           "Key value (int)").asInt();


   /* get the overlap ratio */

   overlap               = rf.check("overlap",
                           Value(1.0),
                           "Key value (int)").asDouble();


   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!imageIn.open(inputPortName.c_str())) {
      cout << getName() << ": unable to open port " << inputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!imageOut.open(outputPortName.c_str())) {
      cout << getName() << ": unable to open port " << outputPortName << endl;
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
 
   //attachTerminal();                     // attach to terminal


   /* create the thread and pass pointers to the module parameters */

   logPolarTransformThread = new LogPolarTransformThread(&imageIn, &imageOut, 
                                                         &direction, 
                                                         &xSize, &ySize,
                                                         &numberOfAngles, &numberOfRings, 
                                                         &overlap);

   /* now start the thread to do the work */

   logPolarTransformThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool LogPolarTransform::interruptModule()
{
//    imageIn.interrupt();
//    imageOut.interrupt();
    handlerPort.interrupt();

    return true;
}


bool LogPolarTransform::close()
{
//    imageIn.close();
//    imageOut.close();
    handlerPort.close();

    /* stop the thread */

    logPolarTransformThread->stop();

    return true;
}


bool LogPolarTransform::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";
    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    return true;
}


/* Called periodically every getPeriod() seconds */

bool LogPolarTransform::updateModule()
{
   return true;
}



double LogPolarTransform::getPeriod()
{
   /* module periodicity (seconds), called implicitly by logPolarTransform */
    
   return 0.1;
}

LogPolarTransformThread::LogPolarTransformThread(BufferedPort<FlexImage> *imageIn, BufferedPort<ImageOf<PixelRgb> > *imageOut, 
                                                 int *direction, int *x, int *y, int *angles, int *rings, double *overlap)
{
    imagePortIn        = imageIn;
    imagePortOut       = imageOut;
    directionValue     = direction;
    xSizeValue         = x;
    ySizeValue         = y;
    anglesValue        = angles;
    ringsValue         = rings;
    overlapValue       = overlap;
    inputImage = 0;
}

bool LogPolarTransformThread::threadInit() 
{
    /* grab an image to set the image size */
    FlexImage *image;
    do {
        image = imagePortIn->read(true);
    } while (image == NULL && !isStopping());

    if (isStopping())
        return false;

    const int width  = image->width();
    const int height = image->height();
    // the logpolar mapping has always depth 3 (RGB) but we need to copy the input image in case it's monochrome.
    const int depth = 3; 
 
    cout << "||| logPolarTransformThread: width = " << *xSizeValue << " height = " << *ySizeValue << endl;
    cout << "||| logPolarTransformThread: angles = " << *anglesValue << " rings = " << *ringsValue << endl;

    /* create the input image of the correct resolution  */
    if (*directionValue == CARTESIAN2LOGPOLAR) {
        *xSizeValue = width;
        *ySizeValue = height;
    }

    cout << "||| initializing the logpolar mapping" << endl;
    if (!allocLookupTables(*directionValue, *ringsValue, *anglesValue, *xSizeValue, *ySizeValue, *overlapValue)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table allocation done" << endl;

    inputImage = new ImageOf<PixelRgb>;
    inputImage->resize(width, height);

    return true;
}

void LogPolarTransformThread::run() {
    //
    while (isStopping() != true) {
        FlexImage *image = imagePortIn->read(true);
        if (image != 0) 
        {
            // copies the input image (generic) into a PixelRgb image.
            inputImage->copy(*image);

            if (*directionValue == CARTESIAN2LOGPOLAR) {
                //
                ImageOf<PixelRgb> &outputImage = imagePortOut->prepare();
                outputImage.resize(*anglesValue,*ringsValue);

                trsf.cartToLogpolar(outputImage, *inputImage);

                imagePortOut->write();
            }
            else {
                ImageOf<PixelRgb> &outputImage = imagePortOut->prepare();
                outputImage.resize(*xSizeValue,*ySizeValue);

                trsf.logpolarToCart(outputImage, *inputImage);

                imagePortOut->write();
            }
        }
    }
}

void LogPolarTransformThread::threadRelease() {
    if (inputImage) delete inputImage;
    inputImage = 0;
    freeLookupTables();
}

bool LogPolarTransformThread::allocLookupTables(int which, int necc, int nang, int w, int h, double overlap) {
    //
    if (which == CARTESIAN2LOGPOLAR)
        trsf.allocLookupTables(C2L, necc, nang, w, h, overlap);
    else {
        trsf.allocLookupTables(L2C, necc, nang, w, h, overlap);
    }
    return true;
}

bool LogPolarTransformThread::freeLookupTables() {
    trsf.freeLookupTables();
    return true;
}


// LATER: add OnStop for proper module/thread termination.

