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
 * 21/09/09  First version validated   DV
 */ 

 
#include "iCub/imageSource.h"


bool ImageSource::configure(yarp::os::ResourceFinder &rf)
{    

   debug = false;
   
   /* Process all parameters from both command-line and .ini file */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("imageSource"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* now, get the rest of the parameters */

   /* 
    * get the imageFilename
    */

   imageFilename  = rf.check("imageFile", 
                             Value("image.bmp"), 
                             "image filename (string)").asString();

   imageFilename = (rf.findFile(imageFilename.c_str())).c_str();

   /* get the complete name of the image output port */

   outputPortName        = rf.check("outputPort", 
                           Value("/image:o"),
                           "Output image port (string)").asString();

   /* get the complete name of the gaze output port */

   gazePortName          = rf.check("gazePort", 
                           Value("/gaze:o"),
                           "Output gaze port (string)").asString();

   /* get the frequency, width, height, standard deviation, horizontalViewAngle, and verticalViewAngle values */

   frequency             = rf.check("frequency",
                           Value(10),
                           "frequency key value (int)").asInt();

   width                 = rf.check("width",
                           Value(320),
                           "output width key value (int)").asInt();

   height                = rf.check("height",
                           Value(240),
                           "output height key value (int)").asInt();

   noiseLevel            = rf.check("noise",
                           Value(20),
                           "noise level key value (int)").asInt();
 
   window                = rf.check("window",
                           Value(0),
                           "window flag key value (int)").asInt();
 
   random                = rf.check("random",
                           Value(0),
                           "random flag key value (int)").asInt();
 
   horizontalViewAngle   = rf.check("horizontalViewAngle",
                           Value(120.0),
                           "horizontal field of view angle key value (double)").asDouble();

   verticalViewAngle     = rf.check("verticalViewAngle",
                           Value(90.0),
                           "vertical field of view angle key value (double)").asDouble();


   if (debug) {
      cout << "imageSource::configure: image file name  " << imageFilename << endl;
      cout << "imageSource::configure: output port name " << outputPortName << endl;
      cout << "imageSource::configure: gaze port name   " << gazePortName << endl;
      cout << "imageSource::configure: frequency        " << frequency << endl;
      cout << "imageSource::configure: width            " << width << endl;
      cout << "imageSource::configure: height           " << height << endl;
      cout << "imageSource::configure: noise level      " << noiseLevel << endl;
      cout << "imageSource::configure: window flag      " << window << endl;
      cout << "imageSource::configure: random flag      " << random << endl;
      cout << "imageSource::configure: horizontal FoV   " << horizontalViewAngle << endl;
      cout << "imageSource::configure: vertical FoV     " << verticalViewAngle << endl;
   }

   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!imageOut.open(outputPortName.c_str())) {
      cout << "imageSource::configure" << ": unable to open port " << outputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

          
   if (!gazeOut.open(gazePortName.c_str())) {
      cout << "imageSource::configure" << ": unable to open port " << gazePortName << endl;
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

   //cout << "imageSource::configure: calling Thread constructor"   << endl;

   imageSourceThread = new ImageSourceThread(&imageOut, &gazeOut, &imageFilename, 
                                             (int)(1000 / frequency), &width, &height, &noiseLevel, &window, &random,
                                             &horizontalViewAngle, &verticalViewAngle);

   //cout << "imageSource::configure: returning from Thread constructor"   << endl;

   /* now start the thread to do the work */

   imageSourceThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true;       // let the RFModule know everything went well
                      // so that it will then run the module
}


bool ImageSource::interruptModule()
{
   imageOut.interrupt();
   gazeOut.interrupt();
   handlerPort.interrupt();

   return true;
}


bool ImageSource::close()
{
   imageOut.close();
   gazeOut.close();
   handlerPort.close();

   /* stop the thread */

   imageSourceThread->stop();

   return true;
}


bool ImageSource::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set noise <n> ... set the noise level \n" + 
                        "(where <n> is an integer number in the range 0-255) \n";

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
      if (command.get(1).asString()=="noise") {
         noiseLevel = command.get(2).asInt(); // set parameter value
         reply.addString("ok");
      }
   }
   return true;
}


/* Called periodically every getPeriod() seconds */

bool ImageSource::updateModule()
{
   return true;
}



double ImageSource::getPeriod()
{
   /* module periodicity (seconds), called implicitly by imageSource */
    
   return 0.1;
}

 
ImageSourceThread::ImageSourceThread(BufferedPort<ImageOf<PixelRgb> > *imageOut, 
                                     BufferedPort<VectorOf<double> > *gazeOut,
                                     string *imageFilename, int period, int *width, int *height, int *noiseLevel, 
                                     int *window, int *random,
                                     double *horizontalViewAngle, double *verticalViewAngle) : RateThread(period)
{
   debug                    = false;

   imagePortOut             = imageOut;
   gazePortOut              = gazeOut;
   imageFilenameValue       = imageFilename;
   widthValue               = width;
   heightValue              = height;
   noiseValue               = noiseLevel;
   windowValue              = window;
   randomValue              = random;
   horizontalViewAngleValue = horizontalViewAngle;
   verticalViewAngleValue   = verticalViewAngle;
     
   
   if (debug) {
      cout << "ImageSourceThread: image file name  " << *imageFilenameValue << endl;
      cout << "ImageSourceThread: width            " << *widthValue << endl;
      cout << "ImageSourceThread: height           " << *heightValue << endl;
      cout << "ImageSourceThread: noise level      " << *noiseValue << endl;
      cout << "ImageSourceThread: window flag      " << *windowValue << endl;
      cout << "ImageSourceThread: random flag      " << *randomValue << endl;
      cout << "ImageSourceThread: horizontal FoV   " << *horizontalViewAngleValue << endl;
      cout << "ImageSourceThread: vertical FoV     " << *verticalViewAngleValue << endl;
   }
}

bool ImageSourceThread::threadInit() 
{
   /* open the image file and create an image */
   
   if (debug) {
      cout << "ImageSourceThread::threadInit: image file name  " << imageFilenameValue->c_str() << endl;
   }

   if (yarp::sig::file::read(inputImage, imageFilenameValue->c_str())) {
      cout << "ImageSourceThread::threadInit: input image read completed" << endl;
      return true;
   }
   else {
      cout << "ImageSourceThread::threadInit: unable to read image file" << endl;
      return false;
   }

   /* generate a seed for the random variables */

   srand((int)(1000*yarp::os::Time::now()));

   /* set the window offsets to zero */

   xOffset = 0;
   yOffset = 0;
}

void ImageSourceThread::run(){

   if (debug) {
      cout << "ImageSourceThread::run: width          " << *widthValue << endl;
      cout << "ImageSourceThread::run: height         " << *heightValue << endl;
      cout << "ImageSourceThread::run: noise          " << *noiseValue << endl;
      cout << "ImageSourceThread::run: window flag    " << *windowValue << endl;
      cout << "ImageSourceThread::run: random flag    " << *randomValue << endl;
      cout << "ImageSourceThread::run: horizontal FoV " << *horizontalViewAngleValue << endl;
      cout << "ImageSourceThread::run: vertical FoV   " << *verticalViewAngleValue << endl;
   }


   /* 
    * copy the image data from file, either scale or extract sub-image, and add noise
    */ 

   double noise;
   double azimuth;
   double elevation;


 
   /* generate offsets for sub-image extraction */

   if (*windowValue == 1) {

      windowFlag = true;

      if ( (*widthValue < inputImage.width())  && (*heightValue < inputImage.height()) ) {

         if (*randomValue == 1) {

            // random position of window

            xOffset = (int)((float)(inputImage.width() - *widthValue)*((float)rand() / (float)(RAND_MAX)));
            yOffset = (int)((float)(inputImage.height() - *heightValue)*((float)rand() / (float)(RAND_MAX)));

         }
         else {

            // regular scan pattern: row major order, with x and y increment equal to the window dimensions 
            // so that the window scans the complete image (except for borders at the right-hand side and bottom)

            xOffset = xOffset + *widthValue;
            if (xOffset > (inputImage.width() - *widthValue)) {
               xOffset = 0;
               yOffset = yOffset + *heightValue;
               if (yOffset > (inputImage.height() - *heightValue)) {
                  yOffset = 0;
               }
            }
         }
      }
      else {
         windowFlag = false;
         xOffset = 0;
         yOffset = 0;
      }
   }

   if (debug) {
      cout << "ImageSourceThread::run: xOffset     " << xOffset << endl;
      cout << "ImageSourceThread::run: yOffset     " << yOffset << endl;
   }
      
   ImageOf<PixelRgb> &outputImage = imagePortOut->prepare();
   outputImage.resize(*widthValue, *heightValue);
      
   for (x=0; x<outputImage.width(); x++) {
      for (y=0; y<outputImage.height(); y++) {
 
         noise = ((float)rand() / (float)(RAND_MAX));          // 0-1
         noise = 2 * (noise - 0.5);                            // -1 - +1
         noise = noise * (*noiseValue);                        // -noiseValue - +noiseValue

         // decide whether to copy directly or extract a sub-image

         if (windowFlag == false) {

            // scale the image
            
            rgbPixel = inputImage((int)(x * ((float)inputImage.width()/(float)outputImage.width())), 
                                  (int)(y * ((float)inputImage.height()/(float)outputImage.height())));
         }
         else {
              
            // extract a sub-image from the original image
            
            rgbPixel = inputImage((int)(x + xOffset), 
                                  (int)(y + yOffset));
         }

         if (((double) rgbPixel.r + noise) < 0)   
            rgbPixel.r = 0; 
         else if (((double) rgbPixel.r + noise) > 255) 
            rgbPixel.r = 255; 
         else 
            rgbPixel.r = (unsigned char) (rgbPixel.r + noise);

         if (((double) rgbPixel.g + noise) < 0)   
            rgbPixel.g = 0; 
         else if (((double) rgbPixel.g + noise) > 255) 
            rgbPixel.g = 255; 
         else 
            rgbPixel.g = (unsigned char) (rgbPixel.g + noise);

         if (((double) rgbPixel.b + noise) < 0)   
            rgbPixel.b = 0; 
         else if (((double) rgbPixel.b + noise) > 255) 
            rgbPixel.b = 255; 
         else 
            rgbPixel.b = (unsigned char) (rgbPixel.b + noise);
           
         outputImage(x,y) = rgbPixel;
      }
   }
   imagePortOut->write();

   /*
    * Now write out the simulated gaze angles
    */

   if (windowFlag == true) {

      azimuth   = ((xOffset * 2.0) - ((double)inputImage.width() - (double)outputImage.width())) * 
                  (*horizontalViewAngleValue /  (2.0 * (double)inputImage.width()));
   
      elevation = ((yOffset * 2.0) - ((double)inputImage.height() - (double)outputImage.height())) * 
                  (*verticalViewAngleValue /  (2.0 * (double)inputImage.height()));

      
      if (debug) {
         cout << "ImageSourceThread::run: azimuth     " << azimuth << endl;
         cout << "ImageSourceThread::run: elevation   " << elevation << endl;
      }

      VectorOf<double> &vctPos = gazePortOut->prepare();
      vctPos.resize(5);
      vctPos(0) = azimuth;
      vctPos(1) = elevation;
      vctPos(2) = (double)(int)'a'; // absolute (neck reference) coordinates are sent
      vctPos(3) = (double)(int)'s'; // receiver module should do saccades
      vctPos(4) = 0; // saccade index

      // write output vector
       
      gazePortOut->write();
   }
}

void ImageSourceThread::threadRelease() 
{
   /* delete dynamically created data-structures */
}
