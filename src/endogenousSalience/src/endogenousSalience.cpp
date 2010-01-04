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
 * 03/01/10  Started development.  DV
 *
 */ 


#include "iCub/endogenousSalience.h"


EndogenousSalience::EndogenousSalience() {
   debug = true;
}

bool EndogenousSalience::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - endogenousSalience.ini file (or whatever file is specified by the --from argument)
    *  - icubEyes.ini file (or whatever file is specified by the --cameraConfig argument
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("endogenousSalience"), 
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
    * get the cameraConfig file and read the required parameter values, fx, fy
    * in the [CAMERA_CALIBRATION_RIGHT] group
    */

   cameraConfigFilename  = rf.check("cameraConfig", 
                           Value("icubEyes.ini"), 
                           "camera configuration filename (string)").asString();

   cameraConfigFilename = (rf.findFile(cameraConfigFilename.c_str())).c_str();

   Property cameraProperties;

   if (cameraProperties.fromConfigFile(cameraConfigFilename.c_str()) == false) {
      cout << "endogenousSalience: unable to read camera configuration file" << cameraConfigFilename;
      return 0;
   }
   else {
      fxRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx", Value(225.0), "fx right").asDouble();
      fyRight = (float) cameraProperties.findGroup("CAMERA_CALIBRATION_RIGHT").check("fy", Value(225.0), "fy right").asDouble();
   }

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   cartesianInputPortName     = "/";
   cartesianInputPortName    += getName(
                                rf.check("cartesianImageInPort", 
                                Value("/cartesianImage:i"),
                                "Left input image port (string)").asString()
                                );
   
   logpolarInputPortName      = "/";
   logpolarInputPortName     += getName(
                                rf.check("logpolarImageInPort", 
                                Value("/logpolarImage:i"),
                                "Right input image port (string)").asString()
                                );

   segmentedOutputPortName    = "/";
   segmentedOutputPortName   += getName(
                                rf.check("segmentedImageOutPort", 
                                Value("/segmentedImage:o"),
                                "Left output image port (string)").asString()
                                );
   
   salienceOutputPortName     = "/";
   salienceOutputPortName    += getName(
                                rf.check("salienceOutPort", 
                                Value("/salience:o"),
                                "Right output image port (string)").asString()
                                );

   robotPortName              = "/";
   robotPortName             += getName(
                                rf.check("headPort", 
                                Value("/head:i"),
                                "Robot head encoder state port (string)").asString()
                                );

   hueTolerance               = rf.check("hueTolerance",
                                Value(10),
                                "Tolerance for hue value (int)").asInt();

   saturationTolerance        = rf.check("saturationTolerance",
                                Value(10),
                                "Tolerance for saturation value (int)").asInt();

   hueBins                    = rf.check("hueBins",
                                Value(32),
                                "Number of hue bins in HS histogram (int)").asInt();

   saturationBins             = rf.check("saturationBins",
                                Value(32),
                                "Number of saturation bins in HS histogram (int)").asInt();

   filterRadius               = rf.check("filterRadius",
                                Value(10),
                                "Radius of the morphological opening filter (int)").asInt();

   if (debug) {
      printf("endogenousSalience: module name is                    %s\n",moduleName.c_str());
      printf("endogenousSalience: robot name is                     %s\n",robotName.c_str());
      printf("endogenousSalience: camera configuration filename is  %s\n",cameraConfigFilename.c_str());
      printf("endogenousSalience: camera properties are             %f %f\n",fxRight,fyRight);
      printf("endogenousSalience: hue and saturation tolerances are %d %d\n",hueTolerance,saturationTolerance);
      printf("endogenousSalience: hue and saturation bins are       %d %d\n",hueBins,saturationBins);
      printf("endogenousSalience: port names are\n%s\n%s\n%s\n%s\n%s\n\n",cartesianInputPortName.c_str(),
                                                                          logpolarInputPortName.c_str(),
                                                                          segmentedOutputPortName.c_str(),
                                                                          salienceOutputPortName.c_str(),
                                                                          robotPortName.c_str(),
                                                                          cameraConfigFilename.c_str()
                                                                          );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!cartesianImageIn.open(cartesianInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << cartesianInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!logpolarImageIn.open(logpolarInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << logpolarInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!segmentedImageOut.open(segmentedOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << segmentedOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
      
   if (!salienceOut.open(salienceOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << salienceOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!robotPort.open(robotPortName.c_str())) {           
      cout << getName() << ": Unable to open port " << robotPortName << endl;  
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

   endogenousSalienceThread = new EndogenousSalienceThread(&cartesianImageIn, 
                                                           &logpolarImageIn, 
                                                           &segmentedImageOut, 
                                                           &salienceOut, 
                                                           &robotPort,
                                                           &hueTolerance, 
                                                           &saturationTolerance, 
                                                           &hueBins, 
                                                           &saturationBins, 
                                                           &filterRadius, 
                                                           &fxRight, 
                                                           &fyRight);

   /* now start the thread to do the work */

   endogenousSalienceThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool EndogenousSalience::interruptModule()
{
   cartesianImageIn.interrupt();
   logpolarImageIn.interrupt();
   segmentedImageOut.interrupt();
   salienceOut.interrupt();
   robotPort.interrupt();
   handlerPort.interrupt();

   return true;
}


bool EndogenousSalience::close()
{
   cartesianImageIn.close();
   logpolarImageIn.close();
   segmentedImageOut.close();
   salienceOut.close();
   robotPort.close();
   handlerPort.close();

   /* stop the thread */

   endogenousSalienceThread->stop();

   return true;
}


bool EndogenousSalience::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set hue <n>   ... set the percentage tolerance on the hue value used to segment the image \n" +   
                        "set sat <n>   ... set the percentage tolerance on the saturation value used to segment the image \n" +
                        "set rad <m>   ... set the radius of the filter used to remove small regions from the segmented the image\n" +
                        "(where  <n> is an integer number in the range [0,100] and m is an integer number >= 0)\n";

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
      if (command.get(1).asString()=="hue") {
         hueTolerance = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
      else if (command.get(1).asString()=="sat") {
         saturationTolerance = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
      else if (command.get(1).asString()=="rad") {
         filterRadius = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool EndogenousSalience::updateModule()
{
   return true;
}



double EndogenousSalience::getPeriod()
{
   /* module periodicity (seconds), called implicitly by endogenousSalience */
    
   return 0.1;
}

EndogenousSalienceThread::EndogenousSalienceThread(BufferedPort<ImageOf<PixelRgb> > *cartesianImageIn, 
                                                   BufferedPort<ImageOf<PixelRgb> > *logpolarImageIn,
                                                   BufferedPort<ImageOf<PixelRgb> > *segmentedImageOut, 
                                                   BufferedPort<Bottle >            *salienceOut,
                                                   BufferedPort<Vector>             *robotPortInOut,
                                                   int *hueToleranceValue, 
                                                   int *saturationToleranceValue,
                                                   int *hueBinsValue, 
                                                   int *saturationBinsValue,
                                                   int *filterRadiusValue,
                                                   float *fxRightValue,    
                                                   float *fyRightValue)
{
   cartesianImagePortIn    = cartesianImageIn;
   logpolarImagePortIn     = logpolarImageIn;
   segmentedImagePortOut   = segmentedImageOut;
   saliencePortOut         = salienceOut;
   robotPort               = robotPortInOut;
   hueTolerance            = hueToleranceValue; 
   saturationTolerance     = saturationToleranceValue;
   hueBins                 = hueBinsValue; 
   saturationBins          = saturationBinsValue;
   filterRadius            = filterRadiusValue;
   fxRight                 = fxRightValue;
   fyRight                 = fyRightValue; 
}

bool EndogenousSalienceThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = true;
    encoderPositions = NULL;
    cartesianInput   = NULL;
    logpolarInput    = NULL;
    segmentedOutput  = NULL;
    hsHistogram      = NULL;
    tempImage        = NULL;
    no_of_blobs      = 0;

    for (int i=0; i<=5; i++) {
         encoders[i] = 0;  
    }

    return true;
}

void EndogenousSalienceThread::run(){

   /* 
    * Identify the modal hue and saturation values in the logpolar image, 
    * segment the cartesian image accordingly,
    * identify the largest blob, extract the coordinates of the centroid, 
    * package everything with the encoder values & the focal length values in a bottle,
    * and send it out (typically to the egosphere module)
    * send out the segmented image out for viewing
    */ 
      
   unsigned char pixel_value;

   if (debug) {
      printf("endogenousSalienceThread: parameters are %d %d %d %d %4.1f %4.1f\n\n",
             *hueTolerance, *saturationTolerance, *hueBins, *saturationBins, *fxRight, *fyRight);
   }

   /* create the histogram */

   if (hsHistogram == NULL) {
      hsHistogram = new DVhs_histogram(*hueBins, *saturationBins);
   }
   
   
   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      /* 
       * Step 1: read the encoder values for inclusion in the salience bottle
       * ==========================================================================
       *
       */

      if (debug) cout << "endogenousSalienceThread: reading encoder values" << endl;

      do {
         encoderPositions = robotPort->read(true);
      } while (encoderPositions == NULL);

      for (int i=0; i<=5; i++) {
         encoders[i] = (float) encoderPositions->data()[i];  
         if (debug)   cout << "endogenousSalienceThread: encoder values: " <<  encoders[i] << endl;
      }

           
      /* 
       * Step 2: grab cartesian and logpolar images and copy images to local format
       * ==========================================================================
       */

      if (debug) cout << "endogenousSalienceThread: grabbing images " << endl;

      do {
         cartesianImage = cartesianImagePortIn->read(true);
      } while (cartesianImage == NULL);
      
      do {
         logpolarImage = logpolarImagePortIn->read(true);
      } while (logpolarImage == NULL);



      width  = cartesianImage->width();
      height = cartesianImage->height();
      depth = 3;
      if (debug) printf("endogenousSalienceThread: width = %d, height = %d, depth = %d\n",width, height, depth);
    
      if (cartesianInput == NULL) {
          cartesianInput = new DVimage(width, height, depth);
      }

      if (segmentedOutput == NULL) {
          segmentedOutput = new DVimage(width, height, depth);
      }

      if (tempImage == NULL) {
          tempImage = new DVimage(width, height, depth);
      }



      width  = logpolarImage->width();
      height = logpolarImage->height();
      depth = 3;
      if (debug) printf("endogenousSalienceThread: width = %d, height = %d, depth = %d\n",width, height, depth);

      if (logpolarInput == NULL) {
         logpolarInput = new DVimage(width, height, depth);
      }
 

      width  = cartesianImage->width();
      height = cartesianImage->height();

      for (x=0; x<width; x++) {
         for (y=0; y<height; y++) {
            rgbPixel = cartesianImage->safePixel(x,y);
            cartesianInput->put_pixel(x, y, rgbPixel.r, 0);
            cartesianInput->put_pixel(x, y, rgbPixel.g, 1);
            cartesianInput->put_pixel(x, y, rgbPixel.b, 2);
        }
      } 

      width  = logpolarImage->width();
      height = logpolarImage->height();

      for (x=0; x<width; x++) {
         for (y=0; y<height; y++) {
            rgbPixel = logpolarImage->safePixel(x,y);
            logpolarInput->put_pixel(x, y, rgbPixel.r, 0);
            logpolarInput->put_pixel(x, y, rgbPixel.g, 1);
            logpolarInput->put_pixel(x, y, rgbPixel.b, 2);
        }
      } 


      /* 
       * Step 3: Identify the modal hue and saturation values in the logpolar image
       * ==========================================================================
       *
       */
 
      if (debug) cout << "endogenousSalienceThread: generating histogram " << endl;

      colour_histogram(logpolarInput, hsHistogram);

           
      if (true && debug) cout << "endogenousSalienceThread: identifying histogram mode " << endl;

      hsHistogram->hsMode(&hueMode, &saturationMode);


      /* 
       * Step 4: segment the cartesian image  
       * ===================================== 
       */

      if (debug) cout << "endogenousSalienceThread: performing segmentation " << endl;

      hueRange = (*hueTolerance * (float) 360) / (float) (100);                    // convert percentage to real values
      saturationRange = (*saturationTolerance) / (float) (100);

      colour_segmentation(cartesianInput, hueMode, saturationMode, hueRange, saturationRange, segmentedOutput);

     

      // Step 5: filter the colour_segmentation by performing an morphological opening
      // =============================================================================
		      
      if (debug) cout << "endogenousSalienceThread: post-segmentation filtering " << endl;

      if (*filterRadius > 0) {
         erosion (segmentedOutput, *filterRadius, tempImage);
         dilation(tempImage, *filterRadius, segmentedOutput);
	  }


      // Step 6: perform blob analysis and choose largest blob as the most salient region
      // ================================================================================
 
      if (debug) cout << "endogenousSalienceThread: finding blobs " << endl;

      find_blobs (segmentedOutput->idata,  width, height, tempImage->idata, blob_list, &no_of_blobs);

      if (no_of_blobs > 0) {
         xCentroid = blob_list[0].centroid_x;
	     yCentroid = blob_list[0].centroid_y;
      }
      else {
         xCentroid = cartesianImage->width();
	     yCentroid = cartesianImage->height();
      }
      
      if (debug) cout << "endogenousSalienceThread: blob centroid " << xCentroid << " " << yCentroid << endl;
   
      
      /* 
       * Step 7: build and write out the salience bottle
       * =============================================== 
       */

      if (debug) cout << "endogenousSalienceThread: building and writing salience bottle " << endl;

      size_bottle.clear();
      size_bottle.addString("size");
      size_bottle.addInt(1); 
 
      encod_bottle.clear();
      encod_bottle.addString("encoders");
      encod_bottle.addDouble(encoders[0]);
      encod_bottle.addDouble(encoders[1]);
      encod_bottle.addDouble(encoders[2]);
      encod_bottle.addDouble(encoders[3]);
      encod_bottle.addDouble(encoders[4]);
      encod_bottle.addDouble(encoders[5]);

      salience_bottle.clear();
      salience_bottle.addList() = size_bottle;
      salience_bottle.addList() = encod_bottle;

      width  = cartesianImage->width();
      height = cartesianImage->height();

      // used to input in the objectMap, which asks for azimuth elevation relative to the image, not absolute
 
      azimuth_bottle.clear();
      azimuth_bottle.addString("azimuth");
      azimuth_bottle.addDouble(-atan2(((float)xCentroid - (float)width/2), *fxRight)*180/M_PI);   

      elevation_bottle.clear();
      elevation_bottle.addString("elevation");
      elevation_bottle.addDouble(-atan2(((float)height/2 - (float)yCentroid), *fyRight)*180/M_PI);
       
      // cout << "endogenousSalienceThread: " << xCentroid << " - " << width/2 << " -atan2 " << -atan2(((float)xCentroid - (float)width/2),*fxRight)*180/M_PI << " M_PI " << M_PI << "azimuth " << -atan2(((float)xCentroid - (float)width/2), *fxRight)*180/M_PI << endl;
       
      x_bottle.clear();
      x_bottle.addString("x");
      x_bottle.addInt(xCentroid);

      y_bottle.clear();
      y_bottle.addString("y");
      y_bottle.addInt(yCentroid);

      label_bottle.clear();
      label_bottle.addString("label");
      label_bottle.addString("1");

      match_bottle.clear();
      match_bottle.addInt(0);
      match_bottle.addList() = x_bottle;
      match_bottle.addList() = y_bottle;
      match_bottle.addList() = label_bottle;
      match_bottle.addList() = azimuth_bottle;
      match_bottle.addList() = elevation_bottle;
   
      salience_bottle.addList() = match_bottle;
        
      cout << "salience_bottle " << salience_bottle.toString() << endl;

      saliencePortOut->prepare() = salience_bottle;
      saliencePortOut->write();
    

    
      /* 
       * Step 8: copy segmented image back to YARP format and write it out
       * =================================================================
       */

      if (debug) cout << "endogenousSalienceThread: sending images " << endl;
     
      width  = cartesianImage->width();
      height = cartesianImage->height();

      ImageOf<PixelRgb> &segmentedImage  = segmentedImagePortOut->prepare();
      segmentedImage.resize(width,height);

      for (x=0; x < width; x++) {
         for (y=0; y < height; y++) {
 
            segmentedOutput->get_pixel(x, y, &pixel_value, 0); rgbPixel.r=pixel_value; 
            segmentedOutput->get_pixel(x, y, &pixel_value, 1); rgbPixel.g=pixel_value;
            segmentedOutput->get_pixel(x, y, &pixel_value, 2); rgbPixel.b=pixel_value;
            segmentedImage(x,y) = rgbPixel;

         }
      }

      segmentedImagePortOut->write();
 
   }

}

void EndogenousSalienceThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */

   if (cartesianInput != NULL)      delete cartesianInput;
   if (logpolarInput != NULL)     delete logpolarInput;
   if (segmentedOutput != NULL)  delete segmentedOutput;
   if (hsHistogram != NULL) delete hsHistogram;
}


