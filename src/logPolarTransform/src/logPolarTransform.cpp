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
 */ 

#include "iCub/logPolarTransform.h"

bool LogPolarTransform::configure(yarp::os::ResourceFinder &rf)
{    
   /* Process all parameters from both command-line and .ini file */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("logPolarTransform"), 
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
                           Value(360),
                           "Key value (int)").asInt();

   /* get the size of the Y dimension */

   ySize                 = rf.check("ysize",
                           Value(240),
                           "Key value (int)").asInt();


   /* get the overlap ratio */

   overlap               = rf.check("overlap",
                           Value(0.5),
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
 
   attachTerminal();                     // attach to terminal


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
   imageIn.interrupt();
   imageOut.interrupt();
   handlerPort.interrupt();

   return true;
}


bool LogPolarTransform::close()
{
   imageIn.close();
   imageOut.close();
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
                        "quit \n" + 
                        "set overlap <n> ... set the overlap of the receptive fields \n" + 
                        "(where <n> is an real number) \n";

  reply.clear(); 

  if (command.get(0).asString()=="quit") {
       reply.addString("quitting");
       return false;     
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
   else if (command.get(0).asString()=="set") {
      if (command.get(1).asString()=="overlap") {
         overlap = command.get(2).asDouble(); // set parameter value
         reply.addString("ok");
      }
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

LogPolarTransformThread::LogPolarTransformThread(BufferedPort<ImageOf<PixelRgb> > *imageIn, BufferedPort<ImageOf<PixelRgb> > *imageOut, 
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
}

bool LogPolarTransformThread::threadInit() 
{
   /* initialize variables and create data-structures */

    debug = false;

    /* grab an image to set the image size */

    do {
       image = imagePortIn->read(true);
    } while (image == NULL);

    width  = image->width();
    height = image->height();
    depth = 3;
 
    if (debug) {
       printf("logPolarTransformThread: width = %d, height = %d, depth = %d\n",width, height, depth);
       printf("logPolarTransformThread: angles = %d, rings = %d, depth = %d\n",*anglesValue, *ringsValue, depth);
    }

    /* create the input image of the correct resolution  */

    if (*directionValue == CARTESIAN2LOGPOLAR) {
       cartesian   = new DVimage(width, height, depth);
       logpolar    = new DVimage(*anglesValue, *ringsValue, depth);
       *xSizeValue = width;
       *ySizeValue = height;
    }
    else {
       logpolar    = new DVimage(width, height, depth);
       cartesian   = new DVimage(*xSizeValue, *ySizeValue, depth);
    }
    return true;
}

void LogPolarTransformThread::run(){

   /* 
    * do the transform
    */ 
   
   if (debug) {
      cout << "LogPolarTransformThread: direction value is        " << *directionValue << endl;
      cout << "LogPolarTransformThread: angles & rings values are " << *anglesValue    << " " << *ringsValue << endl;
      cout << "LogPolarTransformThread: xsize & ysize values are  " << *xSizeValue     << " " << *ySizeValue << endl;
      cout << "LogPolarTransformThread: overlap value is          " << *overlapValue   << endl;
   }

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true

      do {
         image = imagePortIn->read(true);
      } while (image == NULL);

      if (*directionValue == CARTESIAN2LOGPOLAR) {

         width = *xSizeValue;
         height = *ySizeValue;

         for (x=0; x<width; x++) {
            for (y=0; y<height; y++) {
               rgbPixel = image->safePixel(x,y);
               cartesian->put_pixel(x, y, rgbPixel.r, 0);
               cartesian->put_pixel(x, y, rgbPixel.g, 1);
               cartesian->put_pixel(x, y, rgbPixel.b, 2);
           }
         } 

         log_polar_transform (cartesian, logpolar, CARTESIAN2LOGPOLAR, *overlapValue);

         ImageOf<PixelRgb> &outputImage = imagePortOut->prepare();
         outputImage.resize(*anglesValue,*ringsValue);

         width = *anglesValue;
         height = *ringsValue;

         for (x=0; x<width; x++) {
            for (y=0; y<height; y++) {
                logpolar->get_pixel(x, y, &pixel_value, 0); rgbPixel.r=pixel_value; 
                logpolar->get_pixel(x, y, &pixel_value, 1); rgbPixel.g=pixel_value;
                logpolar->get_pixel(x, y, &pixel_value, 2); rgbPixel.b=pixel_value;
                outputImage(x,y) = rgbPixel;
           }
         } 
      }
      else {
         
         width = *anglesValue;
         height = *ringsValue;

         for (x=0; x<width; x++) {
            for (y=0; y<height; y++) {
               rgbPixel = image->safePixel(x,y);
               logpolar->put_pixel(x, y, rgbPixel.r, 0);
               logpolar->put_pixel(x, y, rgbPixel.g, 1);
               logpolar->put_pixel(x, y, rgbPixel.b, 2);
           }
         } 

         log_polar_transform (logpolar, cartesian, LOGPOLAR2CARTESIAN, *overlapValue);

         ImageOf<PixelRgb> &outputImage = imagePortOut->prepare();
         outputImage.resize(*xSizeValue,*ySizeValue);

         width = *xSizeValue;
         height = *ySizeValue;

         for (x=0; x<width; x++) {
            for (y=0; y<height; y++) {
                cartesian->get_pixel(x, y, &pixel_value, 0); rgbPixel.r=pixel_value; 
                cartesian->get_pixel(x, y, &pixel_value, 1); rgbPixel.g=pixel_value;
                cartesian->get_pixel(x, y, &pixel_value, 2); rgbPixel.b=pixel_value;
                outputImage(x,y) = rgbPixel;
           }
         } 
      }
            
      imagePortOut->write();
   }
}

void LogPolarTransformThread::threadRelease() 
{
   /* delete dynamically created data-structures */
      
   if (cartesian != NULL)  delete cartesian;
   if (logpolar != NULL)   delete logpolar;

}
