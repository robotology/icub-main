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
 * 01/09/09  Started development.  DV
 * 12/10/09  Added code to write the vergence angles to a file rectification.log
 *           This is conditional upon the value of the debug flag in RectificationThread::threadInit(), line 311
 *           The values are also written to the console.  DV
 */ 

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

#include "iCub/rectification.h"


Rectification::Rectification() {
   debug = false;
}

bool Rectification::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - rectification.ini file (or whatever file is specified by the --from argument)
    *  - icubEyes.ini file (or whatever file is specified by the --cameraConfig argument
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("rectification"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */

   robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();


   /* 
    * get the cameraConfig file and read the required parameter values, fx, fy, cx, cy 
    * in both the groups [CAMERA_CALIBRATION_LEFT] and [CAMERA_CALIBRATION_RIGHT]
    */

   cameraConfigFilename  = rf.check("cameraConfig", 
                           Value("icubEyes.ini"), 
                           "camera configuration filename (string)").asString();

   cameraConfigFilename = (rf.findFile(cameraConfigFilename.c_str())).c_str();

   Property cameraProperties;

   if (cameraProperties.fromConfigFile(cameraConfigFilename.c_str()) == false) {
      cout << "rectification: unable to read camera configuration file" << cameraConfigFilename;
      return 0;
   }
   else {
      fxLeft  = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_LEFT").check("fx", Value(225.0), "fx left").asDouble();
      fyLeft  = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_LEFT").check("fy", Value(225.0), "fy left").asDouble();
      cxLeft  = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_LEFT").check("cx", Value(160.0), "cx left").asDouble();
      cyLeft  = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_LEFT").check("cy", Value(120.0), "cy left").asDouble();
      fxRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx", Value(225.0), "fx right").asDouble();
      fyRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fy", Value(225.0), "fy right").asDouble();
      cxRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("cx", Value(160.0), "cx right").asDouble();
      cyRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("cy", Value(120.0), "cy right").asDouble();
   }

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   leftInputPortName     = "/";
   leftInputPortName    += getName(
                           rf.check("leftImageInPort", 
                           Value("/leftImage:i"),
                           "Left input image port (string)").asString()
                           );
   
   rightInputPortName    = "/";
   rightInputPortName   += getName(
                           rf.check("rightImageInPort", 
                           Value("/rightImage:i"),
                           "Right input image port (string)").asString()
                           );

   leftOutputPortName    = "/";
   leftOutputPortName   += getName(
                           rf.check("leftImageOutPort", 
                           Value("/leftImage:o"),
                           "Left output image port (string)").asString()
                           );
   
   rightOutputPortName   = "/";
   rightOutputPortName  += getName(
                           rf.check("rightImageOutPort", 
                           Value("/rightImage:o"),
                           "Right output image port (string)").asString()
                           );

   robotPortName         = "/";
   robotPortName        += getName(
                           rf.check("headPort", 
                           Value("/head:i"),
                           "Robot head encoder state port (string)").asString()
                           );

   if (debug) {
      printf("rectification: module name is %s\n",moduleName.c_str());
      printf("rectification: robot name is %s\n",robotName.c_str());
      printf("rectification: camera configuration filename is %s\n",cameraConfigFilename.c_str());
      printf("rectification: camera properties are\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",fxLeft,fyLeft,cxLeft,cyLeft,fxRight,fyRight,cxRight,cyRight);
      printf("rectification: port names are\n%s\n%s\n%s\n%s\n%s\n\n",leftInputPortName.c_str(),
                                                                   rightInputPortName.c_str(),
                                                                   leftOutputPortName.c_str(),
                                                                   rightOutputPortName.c_str(),
                                                                   robotPortName.c_str(),
                                                                   cameraConfigFilename.c_str()
                                                                   );
   }
    


   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!leftImageIn.open(leftInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << leftInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!rightImageIn.open(rightInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << rightInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!leftImageOut.open(leftOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << leftOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
      
   if (!rightImageOut.open(rightOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << rightOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!robotPort.open(robotPortName.c_str())) {           
      cout << ": Unable to open port " << robotPortName << endl;  
      return false;
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

   rectificationThread = new RectificationThread(&leftImageIn, &rightImageIn, &leftImageOut, &rightImageOut, &robotPort,
                                                 &fxLeft, &fyLeft, &cxLeft, &cyLeft, &fxRight, &fyRight, &cxRight, &cyRight);

   /* now start the thread to do the work */

   rectificationThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool Rectification::interruptModule()
{
   leftImageIn.interrupt();
   rightImageIn.interrupt();
   leftImageOut.interrupt();
   rightImageOut.interrupt();
   robotPort.interrupt();
   handlerPort.interrupt();

   return true;
}


bool Rectification::close()
{
   leftImageIn.close();
   rightImageIn.close();
   leftImageOut.close();
   rightImageOut.close();
   robotPort.close();
   handlerPort.close();

   /* stop the thread */

   rectificationThread->stop();

   return true;
}


bool Rectification::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
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
   return true;
}


/* Called periodically every getPeriod() seconds */

bool Rectification::updateModule()
{
   return true;
}



double Rectification::getPeriod()
{
   /* module periodicity (seconds), called implicitly by rectification */
    
   return 0.1;
}

RectificationThread::RectificationThread(BufferedPort<ImageOf<PixelRgb> > *leftImageIn, 
                                         BufferedPort<ImageOf<PixelRgb> > *rightImageIn,
                                         BufferedPort<ImageOf<PixelRgb> > *leftImageOut, 
                                         BufferedPort<ImageOf<PixelRgb> > *rightImageOut,
                                         BufferedPort<Vector>            *robotPortInOut,
                                         float *fxLeftValue,  float *fyLeftValue, 
                                         float *cxLeftValue,  float *cyLeftValue, 
                                         float *fxRightValue, float *fyRightValue, 
                                         float *cxRightValue, float *cyRightValue)
{
   leftImagePortIn    = leftImageIn;
   rightImagePortIn   = rightImageIn;
   leftImagePortOut   = leftImageOut;
   rightImagePortOut  = rightImageOut;
   robotPort          = robotPortInOut;
   fxLeft             = fxLeftValue;
   fyLeft             = fyLeftValue; 
   cxLeft             = cxLeftValue;
   cyLeft             = cyLeftValue; 
   fxRight            = fxRightValue;
   fyRight            = fyRightValue; 
   cxRight            = cxRightValue;
   cyRight            = cyRightValue; 
}

bool RectificationThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;
    encoderPositions = NULL;

    width  = 0;
    height = 0;
    depth  = 0;

    /* grab an image to set the image size */

    do {
       leftImage = leftImagePortIn->read(true);
    } while (leftImage == NULL);

    width  = leftImage->width();
    height = leftImage->height();
    depth = 3;
 
    if (debug) printf("rectificationThread: width = %d, height = %d, depth = %d\n",width, height, depth);

    /* create the input images of the correct resolution  */

    leftInput      = new DVimage(width, height, depth);
    rightInput     = new DVimage(width, height, depth);
    leftRectified  = new DVimage(width, height, depth);
    rightRectified = new DVimage(width, height, depth);

    return true;
}

void RectificationThread::run(){

   /* 
    * rectify the two images 
    */ 
      
   unsigned char pixel_value;

   FILE *fp_out;  // encoder values log file for testing ... this code is not executed during normal operation

   if (debug) {
      printf("rectificationThread: parameters are\n%4.1f\n%4.1f\n%4.1f\n%4.1f\n%4.1f\n%4.1f\n%4.1f\n%4.1f\n\n",
         *fxLeft,*fyLeft,*cxLeft,*cyLeft,*fxRight,*fyRight,*cxRight,*cyRight);
   }

   /* log encoder values during tests ... this code is not executed during normal operation */

   if (debug) {
      if ((fp_out = fopen("rectification.log","w")) == 0) {
	     printf("rectificationThread: can't open output rectification.log\n");
     }
   }


   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      /* 
       * Step 1: determine the the camera angles which cause the epipolar distortion to be rectified
       * ===========================================================================================
       *
       * version is the average camera azimuth angle:  vs ~ (L+R)/2
       * vergence is the relative camera azimuth angle: vg = L-R
       *
       * hence:
       *
       * L = vs + vg/2
       * R = vs - vg/2
       *
       * where L and R are the angles specifying the rotation of the camera about the camera Y axis
       * i.e. the absolute camera azimuth angle.
       *
       * See http://eris.liralab.it/wiki/Vergence%2C_Version_and_Disparity
       *
       * However, we wish to rectify relative to, not the absolute camera azimuth angle, 
       * but relative to the gaze angle given by the version angle.
       * Thus, the angles we use are L'and R', viz
       *
       * L' = L - vs = +vg/2
       * R' = R - vs = -vg/2
       *
       */
      do {
         encoderPositions = robotPort->read(true);
      } while (encoderPositions == NULL);

      vergence = (float) encoderPositions->data()[5]; // get the vergence angle

      if (debug) {
         cout << "rectificationThread: vergence angle is " << vergence << endl;
         if (fp_out != NULL) fprintf(fp_out,"Vergence angle is %f\n",vergence);
      }

      leftCameraAngle = vergence / 2;
      rightCameraAngle = -vergence / 2;

      /* 
       * Step 2: grab left and right images and copy images to local format
       * ==================================================================
       */

      if (false && debug) cout << "rectificationThread: grabbing images " << endl;



      do {
         leftImage = leftImagePortIn->read(true);
      } while (leftImage == NULL);
      
      do {
         rightImage = rightImagePortIn->read(true);
      } while (rightImage == NULL);
 

      for (x=0; x<width; x++) {
         for (y=0; y<height; y++) {
            rgbPixel = leftImage->safePixel(x,y);
            leftInput->put_pixel(x, y, rgbPixel.r, 0);
            leftInput->put_pixel(x, y, rgbPixel.g, 1);
            leftInput->put_pixel(x, y, rgbPixel.b, 2);
        }
      } 

      for (x=0; x<width; x++) {
         for (y=0; y<height; y++) {
            rgbPixel = rightImage->safePixel(x,y);
            rightInput->put_pixel(x, y, rgbPixel.r, 0);
            rightInput->put_pixel(x, y, rgbPixel.g, 1);
            rightInput->put_pixel(x, y, rgbPixel.b, 2);
        }
      } 



      /* 
       * Step 3: rectify left and right images 
       * ===================================== 
       */

      if (false && debug) cout << "rectificationThread: performing rectification " << endl;

      rectify(leftInput, rightInput,
              *fxLeft, *fyLeft, *cxLeft, *cyLeft, leftCameraAngle, 
              *fxRight,*fyRight,*cxRight,*cyRight,rightCameraAngle, 
              leftRectified, rightRectified);



      /* 
       * Step 4: copy images back to YARP format and write them out
       * ========================================================== 
       */

      if (false && debug) cout << "rectificationThread: sending images " << endl;

      ImageOf<PixelRgb> &rectifiedLeftImage  = leftImagePortOut->prepare();
      ImageOf<PixelRgb> &rectifiedRightImage = rightImagePortOut->prepare();
      rectifiedLeftImage.resize(width,height);
      rectifiedRightImage.resize(width,height);

      for (x=0; x < width; x++) {
         for (y=0; y < height; y++) {
 
            leftRectified->get_pixel(x, y, &pixel_value, 0); rgbPixel.r=pixel_value; 
            leftRectified->get_pixel(x, y, &pixel_value, 1); rgbPixel.g=pixel_value;
            leftRectified->get_pixel(x, y, &pixel_value, 2); rgbPixel.b=pixel_value;
            rectifiedLeftImage(x,y) = rgbPixel;
             
            rightRectified->get_pixel(x, y, &pixel_value, 0); rgbPixel.r=pixel_value; 
            rightRectified->get_pixel(x, y, &pixel_value, 1); rgbPixel.g=pixel_value;
            rightRectified->get_pixel(x, y, &pixel_value, 2); rgbPixel.b=pixel_value;
            rectifiedRightImage(x,y) = rgbPixel;
         }
      }

      leftImagePortOut->write();
      rightImagePortOut->write();
   }

   if (debug) {
      if (fp_out != NULL) fclose(fp_out);
   }
}

void RectificationThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */

   if (leftInput != NULL)      delete leftInput;
   if (rightInput != NULL)     delete rightInput;
   if (leftRectified != NULL)  delete leftRectified;
   if (rightRectified != NULL) delete rightRectified;
}


