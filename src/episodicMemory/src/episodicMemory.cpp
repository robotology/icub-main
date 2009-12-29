// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon 
 * (based on code written by Alberto Bietti, Logan Niehaus, and Gionvanni Saponaro for the autoAssociativeMemory module)
 * email:   david.vernon@robotcub.org
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
 * Migrated to RFModule class and implemented necessary changes to ensure compliance with iCub standards 
 * (see http://eris.liralab.it/wiki/ICub_Documentation_Standards)
 * David Vernon 16/08/09 
 *
 * Changed the way the database context is handled:
 * it used to be the concatenation of the path parameter and the context parameter
 * however, at present, there is no way to find out the context if it is not set explicitly as a parameter, 
 * either in the config file on the command line, such as would be the case when using the default configuration
 * In addition, the current policy is for the context to be $ICUB_ROOT/app/moduleName/conf and this means that
 * the database directory would be under the conf directory and this doesn't make much sense.
 * Consequently, the path parameter is now redefined to be the full path to the database directory
 * and can, therefore, be located anywhere you like (specifically, it doesn't have to be located in the iCub repository)
 * David Vernon 18/08/09
 *
 * Fixed problem with automatic prefixing of module name to port names
 * it is now  getName(rf.check("imageInPort", Value("image:i"), "Input image port (string)").asString());
 * instead of rf.check("imageInPort", Value(getName("image:i")), "Input image port (string)").asString();
 * as the latter only prefixed the default value, not the parameter value
 * David Vernon 19/08/09
 *
 * Completed compliance with standards and changed to new convention for handling port names and module name
 * (module name has no / prefix and port names _always_ have / prefix)
 * David Vernon 26/08/09  
 *
 * Normalized histograms before computing the histogram intersection. 
 * David Vernon 14/10/09
 *
 * Started adapting the original autoAssociativeMemory module to create the current episodicMemory module
 * David Vernon 2/11/09
 *
 * Complete the episodicMemory module
 * David Vernon 4/11/09
 *
 * Added a configuration key-value pair - offline - to allow the memory to be used without checking
 * that the robot gaze is stable
 * David Vernon 21/11/09
 */

 

// iCub
#include <iCub/episodicMemory.h>

//opencv
#include <cv.h>
#include <highgui.h>

//HistMatchData constructor
HistMatchData::HistMatchData()
{
    setThreshold(0.6);  //default threshold value
    databaseName = "episodicDatabase";
    databaseContext = "";
}

//HistMatchData deconstructor
HistMatchData::~HistMatchData()
{
}


//HistMatchData image getter method
vector<ImageOf<PixelRgb> >& HistMatchData::images()
{
    return imgs;
}

//threshold setter
void HistMatchData::setThreshold(double t)
{
    thrMutex.wait();
    threshold = t;
    thrMutex.post();
}

//threshold getter
void HistMatchData::getThreshold(double& t)
{
    thrMutex.wait();
    t = threshold;
    thrMutex.post();
}

//database context setter
void HistMatchData::setDatabaseContext(string s)
{
    databaseContext = s;
}

//database context getter
string HistMatchData::getDatabaseContext()
{
    return databaseContext;
}

//change database name
void HistMatchData::setDatabaseName(string s)
{
    databaseName = s;
}

//get database name
string HistMatchData::getDatabaseName()
{
    return databaseName; 
}


//Loads a vector of JPG images into a bottle based on our 'database' file
void HistMatchData::loadDatabase()
{

    string file;
    string databaseFolder;
    if (databaseContext == "")
        databaseFolder = databaseName;
    else
        databaseFolder = databaseContext + "/" + databaseName;

    cout << "HistMatchData::loadDatabase: trying to read from " << (databaseFolder + "/" + databaseName).c_str() << endl;

    ifstream datafile((databaseFolder + "/" + databaseName).c_str());
    if (datafile.is_open()) {
        while (!datafile.eof()) {  //open the file and read in each line
            getline(datafile,file);
            if (file.size() <= 3) break;
            file = databaseFolder + "/" + file;
            IplImage *thisImg = cvLoadImage(file.c_str());  //load image
            ImageOf <PixelRgb> yarpImg;
            yarpImg.wrapIplImage(thisImg);  
            imgs.push_back(yarpImg);
        }
    }
	else {
		cout << "HistMatchData::loadDatabase: unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}
}
    
 

bool EpisodicMemory::configure(yarp::os::ResourceFinder &rf)
{

    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                            Value("episodicMemory"), 
                            "module name (string)").asString(); 

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());

  
    // parse parameters or assign default values (append to getName=="/episodicMemory")

    imageInPortName      = "/";
    imageInPortName     += getName(
                           rf.check("imageInPort",
				           Value("/image:i"),
				           "Input image port (string)").asString()
                           );

    imageIdInPortName    = "/";
    imageIdInPortName   += getName(
                           rf.check("imageIdInPort",
				           Value("/imageId:i"),
				           "Input image id port (string)").asString()
                           );

   actionInputPortName   = "/";
   actionInputPortName  += getName(
                           rf.check("actionInPort", 
                           Value("/action:i"),
                           "saccade and action tag port (string)").asString()
                           );
  
    imageOutPortName     = "/";
    imageOutPortName    += getName(
                           rf.check("imageOutPort",
				           Value("/image:o"),
				           "Output image port (string)").asString()
                           );

    imageIdOutPortName   = "/";
    imageIdOutPortName  += getName(
                           rf.check("imageIdOutPort",
				           Value("/imageId:o"),
				           "Output image id port (string)").asString()
                           );

    robotPortName         = "/";
    robotPortName        += getName(
                           rf.check("headPort", 
                           Value("/head:i"),
                           "Robot head encoder state port (string)").asString()
                           );

    databaseName         = rf.check("database",
					       Value("episodicDatabase"),
					       "Database name (string)").asString().c_str();

    path                 = rf.check("path",
				           Value("~/iCub/app/episodicMemory"),
    			           "complete path to database directory").asString().c_str(); 

    threshold            = rf.check("threshold",
                           Value(0.75),
                           "initial threshold value (double)").asDouble();

    frequency            = rf.check("frequency",
                           Value(2),
                           "initial period for image acquisition (int)").asInt();

    offline              = rf.check("offline",
                           Value(0),
                           "turn off check on robot gaze if non-zero (int)").asInt();


    printf("episodicMemory: parameters are \n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%f\n%d\n%d\n\n",
           imageInPortName.c_str(),
           imageIdInPortName.c_str(), 
           actionInputPortName.c_str(),
           imageOutPortName.c_str(), 
           imageIdOutPortName.c_str(), 
           robotPortName.c_str(),
           databaseName.c_str(), 
           path.c_str(), 
           threshold, 
           offline,
           frequency);

    // create episodicMemory ports

    imageIn.open(imageInPortName.c_str());
    imageIdIn.open(imageIdInPortName.c_str());
    actionIn.open(actionInputPortName.c_str());
    imageOut.open(imageOutPortName.c_str());
    imageIdOut.open(imageIdOutPortName.c_str());
    
   if (!robotPort.open(robotPortName.c_str())) {           
      cout << ": Unable to open port " << robotPortName << endl;  
      return false;
   }

   // attach a port to the module
   // so that messages received from the port are redirected
   // to the respond method
   
   handlerPortName =  "/";
   handlerPortName += getName();          
 
   handlerPort.open(handlerPortName.c_str());  
 
   attach(handlerPort);   
   attachTerminal();     //attach to terminal
	
   /* create the thread and pass pointers to the module parameters */

   episodicMemoryThread = new EpisodicMemoryThread(&imageIn,
                                                   &imageIdIn,
                                                   &actionIn, 
                                                   &imageOut,
                                                   &imageIdOut,
                                                   &robotPort,
                                                   &databaseName,
                                                   &path,
                                                   &threshold,
                                                   &offline,
                                                   (int)(1000 / frequency));

   /* now start the thread to do the work */

   episodicMemoryThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true;
}

bool EpisodicMemory::updateModule()
{		
    return true;
}

bool EpisodicMemory::interruptModule()
{
    // interrupt ports gracefully

    imageIn.interrupt();
    imageIdIn.interrupt();
    actionIn.interrupt();
    imageOut.interrupt();
    imageIdOut.interrupt();
    robotPort.interrupt();
    handlerPort.interrupt();

    return true;	
}

bool EpisodicMemory::close()
{
    cout << "Closing EpisodicMemory...\n\n";

    // close episodicMemory ports
    
    //_portThresholdIn->close();   
    
    imageIn.close();
    imageIdIn.close();
    actionIn.close();
    imageOut.close();
    imageIdOut.close();
    robotPort.close();
    handlerPort.close();

    /* stop the thread */

    episodicMemoryThread->stop();
   
    return true;
}


//module periodicity (seconds), called implicitly by module

double EpisodicMemory::getPeriod()

{
    return 0.1; //module periodicity (seconds)
}

// Message handler. 

bool EpisodicMemory::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set thr <n>    ... set the threshold for image recall: 0-1\n" + 
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
      if (command.get(1).asString()=="thr") {
         threshold = command.get(2).asDouble(); // set parameter value
         reply.addString("threshold set ok");
      }
   }
   return true;
}


EpisodicMemoryThread::EpisodicMemoryThread(BufferedPort<ImageOf<PixelRgb> > *imageIn,
                                           BufferedPort<VectorOf<double> >  *imageIdIn,
                                           BufferedPort<VectorOf<double> >  *actionIn,
                                           BufferedPort<ImageOf<PixelRgb> > *imageOut,
                                           BufferedPort<VectorOf<double> >  *imageIdOut,
                                           BufferedPort<Vector>             *robotPortInOut,
                                           string                           *databaseName,
                                           string                           *path,
                                           double                           *threshold,
                                           int                              *offline,
                                           int                              period) : RateThread(period)
{
   debug = false;

   imageInPort          = imageIn;
   imageIdInPort        = imageIdIn;
   actionInPort         = actionIn;
   imageOutPort         = imageOut;
   imageIdOutPort       = imageIdOut;
   robotPort            = robotPortInOut;
   databaseNameValue    = databaseName;
   pathValue            = path;
   thresholdValue       = threshold;
   offlineValue         = offline;

   if (debug) {
      cout << "EpisodicMemoryThread: database name  " << *databaseNameValue << endl;
      cout << "EpisodicMemoryThread: path           " << *pathValue << endl;
      cout << "EpisodicMemoryThread: threshold      " << *thresholdValue << endl;
      cout << "EpisodicMemoryThread: offline        " << *offlineValue << endl;
   }
}

bool EpisodicMemoryThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;

    encoderPositions = NULL;
    roll = 0;
    pitch = 0;
    yaw = 0;
    rollNew = 0;
    pitchNew = 0;
    yawNew = 0;

    imageId = -1;           // initialize to -1 so that we know these are not valid values
    imageIdPrevious = -1;   //

    imageMatch = 0;
    imageMatchPrevious = 0;
   
    gazeAzimuth           = 0;
    gazeElevation         = 0;
    gazeAzimuthPrevious   = 0;
    gazeElevationPrevious = 0;

    data.setThreshold(*thresholdValue);
   
    data.setDatabaseContext(*pathValue);

    if (debug)
       std::cout << "EpisodicMemoryThread::threadInit: databaseContext " << (*pathValue).c_str() << endl;
    

	data.setDatabaseName(*databaseNameValue);

    if (debug)
       std::cout << "EpisodicMemoryThread::threadInit: databaseName    " << (*databaseNameValue).c_str() << endl;
    
    data.loadDatabase();

    return true;
}

void EpisodicMemoryThread::run(){


   if (false) {
      cout << "EpisodicMemoryThread::run: database name  " << *databaseNameValue << endl;
      cout << "EpisodicMemoryThread::run: path           " << *pathValue << endl;
      cout << "EpisodicMemoryThread::run: threshold      " << *thresholdValue << endl;
   }

   if (false)
      std::cout << "EpisodicMemoryThread::run: the threshold is now: " << *thresholdValue << std::endl;

   data.setThreshold(*thresholdValue);   // set the threshold ... we do this in case it has been changed at run-time


   /* First check to see if we can read an image identification number
    * If so, simply retrieve the image.
    * Otherwise recall the image that best matches the presented image,
    * or store it if none match it
    */
 
   imgInId = imageIdInPort->read(false);  // try to read a vector containing the image id
 
   if (imgInId != NULL) { 

      /* retrieve the image corresponding to the image id */

      imageId = (int)(*imgInId)(0);

      if (debug) {
         std::cout << "EpisodicMemoryThread::run: retrieving imageId " << imageId << std::endl;
      }
      
      ImageOf<PixelRgb>& img = *imgIn;
      std::vector<ImageOf<PixelRgb> >& images = data.images();
    
      if ( (matchId >= 0) && (matchId <= (images.end() - images.begin())) ) {       // make sure we are accessing an image that exists

	     it = images.begin() + matchId; // set the iterator

	     matchImage = *it;              // retrieve the image
         matchValue = 1.0;              // retrieving an existing image so the match is perfect

         imageIdPrevious = imageId;
         imageId         = matchId;

         imageMatchPrevious = imageMatch;
         imageMatch         = matchValue;

         gazeAzimuthPrevious    = gazeAzimuth;
         gazeElevationPrevious  = gazeElevation;
 
         imageOutPort->prepare() = matchImage;
         imageOutPort->write();
   
         VectorOf<double>& out = imageIdOutPort->prepare();
         out.resize(8,0);
         out[0] = imageId;
         out[1] = imageMatch;
         out[2] = gazeAzimuth;
         out[3] = gazeElevation;
         out[4] = imageIdPrevious;
         out[5] = imageMatchPrevious;
         out[6] = gazeAzimuthPrevious;
         out[7] = gazeElevationPrevious;
         imageIdOutPort->write();

         if (debug) {
            cout << "EpisodicMemoryThread::run: image retrieved " << imageId         << " " << imageMatch         << " " <<  gazeAzimuth         << " " << gazeElevation << endl;
            cout << "                           image previous  " << imageIdPrevious << " " << imageMatchPrevious << " " <<  gazeAzimuthPrevious << " " << gazeElevationPrevious << endl;
         }

      }
   }
   else {
   
      /* no imageId is available so we need to read an image
       * We do so only when an action is input
       * We check also to see if the head has stabilized 
       * but we ignore this step if the offline flag is set
       */

      actionIn  = NULL;
      if (actionIn == NULL) {
         actionIn  = actionInPort->read(true);         // read a vector containing the action (gaze command)
      }
                  
      if (actionIn != NULL) { 
       
         gazeAzimuthPrevious    = gazeAzimuth;
         gazeElevationPrevious  = gazeElevation;

         gazeAzimuth   = (*actionIn)(0);
         gazeElevation = (*actionIn)(1);
      }


      if (*offlineValue == 0) { 
         do {
            encoderPositions = robotPort->read(true);  
         } while (encoderPositions == NULL);

         if (encoderPositions != NULL) {
            roll  = (float) encoderPositions->data()[1];  
            pitch = (float) encoderPositions->data()[0];  
            yaw   = (float) encoderPositions->data()[2];  
         }

 
         do {
            encoderPositions = robotPort->read(true); // change to true
         } while (encoderPositions == NULL);

         if (encoderPositions != NULL) {
            rollNew  = (float) encoderPositions->data()[1];  
            pitchNew = (float) encoderPositions->data()[0];  
            yawNew   = (float) encoderPositions->data()[2];  
         }
      }

      if (false) {
         cout << "EpisodicMemoryThread::run: old roll, pitch, yaw angles are: " << endl << roll << endl << pitch << endl << yaw << endl;
         cout << "EpisodicMemoryThread::run: new roll, pitch, yaw angles are: " << endl << rollNew << endl << pitchNew << endl << yawNew << endl;
      } 

      if ((roll == rollNew) && (pitch == pitchNew) && (yaw == yawNew)) {

         /* the head has stabilized ... proceed to read an image and recall/store it */
    
         do {
            imgIn = imageInPort->read(true);
         } while (imgIn == NULL);
   	
         ImageOf<PixelRgb>& img = *imgIn;
    
         data.imgMutex.wait();

         std::vector<ImageOf<PixelRgb> >& images = data.images();
         IplImage* currImg = cvCreateImage(cvSize(img.width(), img.height()), IPL_DEPTH_8U, 3);

         //get the images from the port
         cvCvtColor((IplImage*)img.getIplImage(), currImg, CV_RGB2HSV);

         int arr[2] = { 16, 16 }; // 16x16 histogram bins are used, as that is what is done in the literature
         CvHistogram* currHist = cvCreateHist(2, arr, CV_HIST_ARRAY);

         //convert from RGB to HSV and split the 3 channels
         IplImage *currImgH = cvCreateImage(cvGetSize(currImg), IPL_DEPTH_8U, 1);  //hue
         IplImage *currImgS = cvCreateImage(cvGetSize(currImg), IPL_DEPTH_8U, 1);  //saturation
         IplImage *currImgV = cvCreateImage(cvGetSize(currImg), IPL_DEPTH_8U, 1);  //value (thrown away)
         cvSplit(currImg, currImgH, currImgS, currImgV, NULL);
         IplImage* imgArr[2] = { currImgH, currImgS };
         cvCalcHist(imgArr, currHist);
 
         matchValue = *thresholdValue;
         found = false;
         matchId = 0;
         // std::cout << "threshold: " << *thresholdValue << " ";  //for debugging purposes only
    
         //for each image present in the database
         for (it = images.begin(); it != images.end(); ++it)
         {
            IplImage* refImg = cvCreateImage(cvSize(it->width(), it->height()), IPL_DEPTH_8U, 3);
    
            cvCvtColor((IplImage*)it->getIplImage(), refImg, CV_RGB2HSV);
    
            CvHistogram* refHist = cvCreateHist(2, arr, CV_HIST_ARRAY);
    
            //convert the image to HSV, then split the 3 channels
            IplImage *refImgH = cvCreateImage(cvGetSize(refImg), IPL_DEPTH_8U, 1);
            IplImage *refImgS = cvCreateImage(cvGetSize(refImg), IPL_DEPTH_8U, 1);
            IplImage *refImgV = cvCreateImage(cvGetSize(refImg), IPL_DEPTH_8U, 1);
            cvSplit(refImg, refImgH, refImgS, refImgV, NULL);
            imgArr[0] = refImgH;
            imgArr[1] = refImgS;
            cvCalcHist(imgArr, refHist);
    
            //do a histogram intersection, and check it against the threshold 

            // The Bhattacharyya distance metric seemed to produce better results at the VVV 09 summer school
            // however, it works better with normalized histograms so I've added normalization. DV 14/10/09
           
            cvNormalizeHist(currHist,1.0);
            cvNormalizeHist(refHist,1.0);
            double comp = 1 - cvCompareHist(currHist, refHist, CV_COMP_BHATTACHARYYA);  


            // Alternative is intersection DV 14/10/09
            // this method of intersection is the one proposed by Swain and Ballard
            // the intersection value should be normalized by the number of pixels
            // i.e. img.width() * img.height() which is the same as the integral of the histogram

            //double comp = cvCompareHist(currHist, refHist, CV_COMP_INTERSECT)/(img.width() * img.height()); 
                                                                                   
            //cout << comp << " ";    //once again, only for debugging purposes
            //printf("%3.2f ",comp);

            if (comp > matchValue) {
	           matchValue = comp;
	           matchImage = *it;
	           matchId = it - images.begin();
	           found = true;
	        }
            cvReleaseImage(&refImg); 
            cvReleaseImage(&refImgH); 
            cvReleaseImage(&refImgS); 
            cvReleaseImage(&refImgV);
            cvReleaseHist(&refHist);
         }
    
         //if the image produces a match
         if (found)
         {
            imageIdPrevious    = imageId;
            imageId            = matchId;

            imageMatchPrevious = imageMatch;
            imageMatch         = matchValue;

            imageOutPort->prepare() = matchImage;
            imageOutPort->write();
        
            VectorOf<double>& out = imageIdOutPort->prepare();
            out.resize(8,0);
            out[0] = imageId;
            out[1] = imageMatch;
            out[2] = gazeAzimuth;
            out[3] = gazeElevation;
            out[4] = imageIdPrevious;
            out[5] = imageMatchPrevious;
            out[6] = gazeAzimuthPrevious;
            out[7] = gazeElevationPrevious;
            imageIdOutPort->write();

            if (debug) {
               cout << "EpisodicMemoryThread::run: image retrieved " << imageId         << " " << imageMatch         << " " <<  gazeAzimuth         << " " << gazeElevation << endl;
               cout << "                           image previous  " << imageIdPrevious << " " << imageMatchPrevious << " " <<  gazeAzimuthPrevious << " " << gazeElevationPrevious << endl;
            }
         }
         else  { //no match found
    
            //add the image to the database in memory, then into the filesystem.
            images.push_back(img);
            imageOutPort->prepare() = img;
            imageOutPort->write();
    
            //create a filename that is imageXX.jpg

            imageIdPrevious    = imageId;
            imageId            = images.size()-1;

            imageMatchPrevious = imageMatch;
            imageMatch         = (double)1.0;

            VectorOf<double>& out = imageIdOutPort->prepare();
            out.resize(8,0);
            out[0] = imageId;
            out[1] = imageMatch;
            out[2] = gazeAzimuth;
            out[3] = gazeElevation;
            out[4] = imageIdPrevious;
            out[5] = imageMatchPrevious;
            out[6] = gazeAzimuthPrevious;
            out[7] = gazeElevationPrevious;
            imageIdOutPort->write();

            
            if (debug) {
               cout << "EpisodicMemoryThread::run: image stored    " << imageId         << " " << imageMatch         << " " <<  gazeAzimuth         << " " << gazeElevation << endl;
               cout << "                           image previous  " << imageIdPrevious << " " << imageMatchPrevious << " " <<  gazeAzimuthPrevious << " " << gazeElevationPrevious << endl;
            }

            string s;
            ostringstream oss(s);
            oss << "image" << images.size()-1 << ".jpg";
            if (debug) {
               cout << "EpisodicMemoryThread::run: image stored    ";
               cout << oss.str() << endl;
            }

            //write it out to the proper database

            string databasefolder = data.getDatabaseContext() + "/" + data.getDatabaseName();
            cvCvtColor(img.getIplImage(), currImg, CV_RGB2BGR);  //opencv stores images as BGR

		    // cout << "episodicMemory: trying to save to " << (databasefolder + "/" + oss.str()).c_str() << endl;

            cvSaveImage((databasefolder + "/" + oss.str()).c_str(), currImg);
            ofstream of;

	        // cout << "episodicMemory: trying to save to " << (databasefolder + "/" + data->getDatabaseName()).c_str() << endl;

            of.open((databasefolder + "/" + data.getDatabaseName()).c_str(),ios::app);
            of << oss.str() << endl;
            of.close();
         }
    
         cvReleaseImage(&currImg); cvReleaseImage(&currImgH); cvReleaseImage(&currImgS); cvReleaseImage(&currImgV);
         cvReleaseHist(&currHist);
    
         data.imgMutex.post();
      }
   }
}

void EpisodicMemoryThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */


}


