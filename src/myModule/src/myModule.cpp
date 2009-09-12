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
 * 26/08/09  First version validated   DV
 * 12/09/09  Added functionality to read additional configuration file DV
 */ 

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

#include "iCub/myModule.h"

bool replaceDoubleSlash(string &str) {
   string::size_type loc;
   if ((loc=str.find("//",0))!=string::npos) {
      str.erase(loc+1,1);
      return true;
   }
   return false;
}


bool MyModule::configure(yarp::os::ResourceFinder &rf)
{    
   /* Process all parameters from both command-line and .ini file */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("myModule"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* now, get the rest of the parameters */

   /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */

   robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();

   robotPortName         = "/" + robotName + "/head";

   /* 
    * get the cameraConfig file and read the required parameter values cx, cy 
    * in both the groups [CAMERA_CALIBRATION_LEFT] and [CAMERA_CALIBRATION_RIGHT]
    */

   cameraConfigFilename  = rf.check("cameraConfig", 
                           Value("icubEyes.ini"), 
                           "camera configuration filename (string)").asString();

   cameraConfigFilename = (rf.findFile(cameraConfigFilename.c_str())).c_str();

   Property cameraProperties;

   if (cameraProperties.fromConfigFile(cameraConfigFilename.c_str()) == false) {
      cout << "myModule: unable to read camera configuration file" << cameraConfigFilename;
      return 0;
   }
   else {
      cxLeft  = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_LEFT").check("cx", Value(160.0), "cx left").asDouble();
      cyLeft  = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_LEFT").check("cy", Value(120.0), "cy left").asDouble();
      cxRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("cx", Value(160.0), "cx right").asDouble();
      cyRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("cy", Value(120.0), "cy right").asDouble();
   }


   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   inputPortName         = "/";
   inputPortName        += getName(
                           rf.check("myInputPort", 
                           Value("/image:i"),
                           "Input image port (string)").asString()
                           );
   
   outputPortName        = "/";
   outputPortName       += getName(
                           rf.check("myOutputPort", 
                           Value("/image:o"),
                           "Output image port (string)").asString()
                           );

   replaceDoubleSlash(inputPortName);  // temporary fix until behaviour of getName() is changed
   replaceDoubleSlash(outputPortName); // ibid.

   /* get the threshold value */

   thresholdValue        = rf.check("threshold",
                           Value(8),
                           "Key value (int)").asInt();


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

   myThread = new MyThread(&imageIn, &imageOut, &thresholdValue);

   /* now start the thread to do the work */

   myThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool MyModule::interruptModule()
{
   imageIn.interrupt();
   imageOut.interrupt();
   handlerPort.interrupt();

   return true;
}


bool MyModule::close()
{
   imageIn.close();
   imageOut.close();
   handlerPort.close();

   /* stop the thread */

   myThread->stop();

   return true;
}


bool MyModule::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  getName() + " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set thr <n> ... set the threshold \n" + 
                        "(where <n> is an integer number) \n";

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
      if (command.get(1).asString()=="thr") {
         thresholdValue = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool MyModule::updateModule()
{
   return true;
}



double MyModule::getPeriod()
{
   /* module periodicity (seconds), called implicitly by myModule */
    
   return 0.1;
}

MyThread::MyThread(BufferedPort<ImageOf<PixelRgb>> *imageIn, BufferedPort<ImageOf<PixelRgb>> *imageOut, int *threshold)
{
   imagePortIn    = imageIn;
   imagePortOut   = imageOut;
   thresholdValue = threshold;
}

bool MyThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

   return true;
}

void MyThread::run(){

   /* 
    * do some work ....
    * for example, convert the input image to a binary image using the threshold provided 
    */ 
   
   unsigned char value;

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      cout << "myThread: threshold value is " << *thresholdValue << endl;
      
      do {
         image = imagePortIn->read(true);
      } while (image == NULL);
      
      ImageOf<PixelRgb> &binary_image = imagePortOut->prepare();
      binary_image.resize(image->width(),image->height());

      for (x=0; x<image->width(); x++) {
         for (y=0; y<image->height(); y++) {

             rgbPixel = image->safePixel(x,y);

             if (((rgbPixel.r + rgbPixel.g + rgbPixel.b)/3) > *thresholdValue) {
                value = (unsigned char) 255;
             }
             else {
                value = (unsigned char) 0;
             }

             rgbPixel.r = value;
             rgbPixel.g = value;
             rgbPixel.b = value;

             binary_image(x,y) = rgbPixel;
          }
       }

       imagePortOut->write();
   }
}

void MyThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */
}
