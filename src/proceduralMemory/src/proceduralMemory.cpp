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
 * 11/11/09  Started development.  DV
 * 23/12/09  Completed initial development DV
 */ 

#include "iCub/proceduralMemory.h"


ProceduralMemory::ProceduralMemory() {
   debug = true;
}

bool ProceduralMemory::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - proceduralMemory.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("proceduralMemory"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   imageIdInputPortName          = "/";
   imageIdInputPortName         += getName(
                                   rf.check("imageIdInPort", 
                                   Value("/imageId:i"),
                                   "current image id port (string)").asString()
                                   );
   
   modeInputPortName            = "/";
   modeInputPortName           += getName(
                                   rf.check("modePort", 
                                   Value("/mode:i"),
                                   "mode (corresponding to learning, prediction, reconstruction) port (string)").asString()
                                   );
 
   imageIdOutputPortName         = "/";
   imageIdOutputPortName        += getName(
                                   rf.check("imageIdOutPort", 
                                   Value("/imageId:o"),
                                   "current image id port (string)").asString()
                                   );
   
   actionOutputPortName          = "/";
   actionOutputPortName         += getName(
                                   rf.check("actionOutPort", 
                                   Value("/action:o"),
                                   "saccade and action tag port (string)").asString()
                                   );
  
   weightsOutputPortName        = "/";
   weightsOutputPortName        += getName(
                                   rf.check("weightsPort", 
                                   Value("/weights:o"),
                                   "weights port (string)").asString()
                                   );

   databaseName                  = rf.check("database",
					               Value("proceduralDatabase"),
					               "Database name (string)").asString().c_str();

   path                          = rf.check("path",
				                   Value("~/iCub/app/proceduralMemory"),
    			                   "complete path to database file").asString().c_str(); 

   if (debug) {
      printf("proceduralMemory: module name is %s\n",moduleName.c_str());
      printf("proceduralMemory: parameters are\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n\n",   
              imageIdInputPortName.c_str(),
              modeInputPortName.c_str(),
              imageIdOutputPortName.c_str(),
              actionOutputPortName.c_str(),
              weightsOutputPortName.c_str(),
              databaseName.c_str(), 
              path.c_str()
            );
   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!imageIdIn.open(imageIdInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << imageIdInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!modeIn.open(modeInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << modeInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!imageIdOut.open(imageIdOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << imageIdOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
      
   if (!actionOut.open(actionOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << actionOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!weightsOut.open(weightsOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << weightsOutputPortName << endl;
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

   proceduralMemoryThread = new ProceduralMemoryThread(&imageIdIn, 
                                                       &modeIn, 
                                                       &imageIdOut, 
                                                       &actionOut, 
                                                       &weightsOut, 
                                                       &databaseName, 
                                                       &path);

   /* now start the thread to do the work */

   proceduralMemoryThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool ProceduralMemory::interruptModule()
{
   imageIdIn.interrupt();
   modeIn.interrupt();
   imageIdOut.interrupt();
   actionOut.interrupt();
   weightsOut.interrupt();
   handlerPort.interrupt(); 

   return true;
}


bool ProceduralMemory::close()
{
   imageIdIn.close();
   modeIn.close();
   imageIdOut.close();
   actionOut.close();
   weightsOut.close();
   handlerPort.close(); 

   /* stop the thread */

   proceduralMemoryThread->stop();

   return true;
}


bool ProceduralMemory::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
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

   return true;
}


/* Called periodically every getPeriod() seconds */

bool ProceduralMemory::updateModule()
{
   return true;
}



double ProceduralMemory::getPeriod()
{
   /* module periodicity (seconds), called implicitly by proceduralMemory */
    
   return 0.1;
}

 

ProceduralMemoryThread::ProceduralMemoryThread(BufferedPort<VectorOf<double> >  *imageIdIn,
                                               BufferedPort<VectorOf<double> >  *modeIn,
                                               BufferedPort<VectorOf<double> >  *imageIdOut,
                                               BufferedPort<VectorOf<double> >  *actionOut,
                                               BufferedPort<ImageOf<PixelRgb> > *weightsOut,
                                               string                           *databaseName,
                                               string                           *path)
{
   imageIdInPort         = imageIdIn;
   modeInPort            = modeIn;
   imageIdOutPort        = imageIdOut;
   actionOutPort         = actionOut;
   weightsOutPort        = weightsOut;
   databaseNameValue     = databaseName;
   pathValue             = path;

   if (debug) {
      cout << "ProceduralMemoryThread: database name  " << *databaseNameValue << endl;
      cout << "ProceduralMemoryThread: path           " << *pathValue << endl;
   }

}

bool ProceduralMemoryThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;

    width  = 0;
    height = 0;
    depth  = 0;

    imageIdCurrent = 0;
    imageSimilarityCurrent = 0;
    azimuthCurrent = 0;
    elevationCurrent = 0;
    imageIdPrevious = 0;
    imageSimilarityPrevious = 0;
    azimuthPrevious = 0;
    elevationPrevious = 0;

    data.setDatabaseContext(*pathValue);

    if (debug)
       std::cout << "ProceduralMemoryThread::threadInit: databaseContext " << (*pathValue).c_str() << endl;
    
	data.setDatabaseName(*databaseNameValue);

    if (debug)
       std::cout << "ProceduralMemoryThread::threadInit: databaseName    " << (*databaseNameValue).c_str() << endl;
    
    data.loadDatabase();

    return true;
}

void ProceduralMemoryThread::run(){

   /* 
    * start the procedural memory operating 
    */ 
      
   if (debug) {
      cout << "ProceduralMemoryThread::run: database name  " << *databaseNameValue << endl;
      cout << "ProceduralMemoryThread::run: path           " << *pathValue << endl;
   }

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      /* 
       * Step 1: read the mode, the image id, the action, and the previous image id
       * ==========================================================================
       *
       */

      imageIdCurrent = 0;
      imageSimilarityCurrent = 0;
      azimuthCurrent = 0;
      elevationCurrent = 0;
      imageIdPrevious = 0;
      imageSimilarityPrevious = 0;
      azimuthPrevious = 0;
      elevationPrevious = 0;
      coupling = 0;


      /* Determine which mode we are in: learning, exploration, or reconstruction */

      modeIn = modeInPort->read(false);              // read a vector containing the mode; don't block

      if (modeIn != NULL) {
         mode = (int) (*modeIn)[0];
      }
      else {
         mode = LEARNING_MODE;
      }

      imageIdIn  = imageIdInPort->read(true);        // read a vector containing the image data; do block
      
      if (mode == LEARNING_MODE) {
    
         imageIdCurrent          = (int) (*imageIdIn)[0];
         imageSimilarityCurrent  =       (*imageIdIn)[1];
         azimuthCurrent          =       (*imageIdIn)[2];
         elevationCurrent        =       (*imageIdIn)[3];
         imageIdPrevious         = (int) (*imageIdIn)[4];
         imageSimilarityPrevious =       (*imageIdIn)[5];
         azimuthPrevious         =       (*imageIdIn)[6];
         elevationPrevious       =       (*imageIdIn)[7];

         azimuthDelta = azimuthCurrent - azimuthPrevious;
         elevationDelta = elevationCurrent - elevationPrevious;

         data.updateWeights(imageIdPrevious, azimuthDelta, elevationDelta, imageIdCurrent); 

         // for testing only
         //data.predict(imageIdPrevious, temp1, temp2, temp3, coupling); 
         //data.reconstruct(temp3, temp1, temp2, imageIdCurrent, coupling); 
         
         ImageOf<PixelRgb> &weightsImage = weightsOutPort->prepare();
         weightsImage.resize(data.getNumberOfImages(), data.getNumberOfImages());
         data.generateWeightsImage(weightsImage);
         weightsOutPort->write();

      }
      else if (mode == PREDICTION_MODE) {
            
         /* we use the current imageId to retrieve the associated subsequent action and, in turn, the associated image */

         imageIdPrevious          = (int)  (*imageIdIn)[0];
         imageSimilarityPrevious  =        (*imageIdIn)[1];
         azimuthPrevious          =        (*imageIdIn)[2];
         elevationPrevious        =        (*imageIdIn)[3];

         data.predict(imageIdPrevious, azimuthDelta, elevationDelta, imageIdCurrent, coupling); 
         imageSimilarityCurrent = 0;

         azimuthCurrent   = azimuthPrevious + azimuthDelta;
         elevationCurrent = elevationPrevious + elevationDelta;

         VectorOf<double>& out = imageIdOutPort->prepare();
         out.resize(2,0);
         out[0] = imageIdCurrent;
         out[1] = coupling;
         imageIdOutPort->write();

         VectorOf<double> &vctPos = actionOutPort->prepare();
         vctPos.resize(5);
         vctPos(0) = azimuthCurrent;
         vctPos(1) = elevationCurrent;
         vctPos(2) = (double)(int)'a'; // absolute (neck reference) coordinates are sent
         vctPos(3) = (double)(int)'s'; // receiver module should do saccades
         vctPos(4) = 0; // saccade index
         actionOutPort->write();

      }
      else if (mode == RECONSTRUCTION_MODE) {
             
         /* we use the current imageId to retrieve the associated prior action and, in turn, the associated image */

         imageIdCurrent          = (int)  (*imageIdIn)[0];
         imageSimilarityCurrent  =        (*imageIdIn)[1];
         azimuthCurrent          =        (*imageIdIn)[2];
         elevationCurrent        =        (*imageIdIn)[3];

         data.reconstruct(imageIdPrevious, azimuthDelta, elevationDelta, imageIdCurrent, coupling); 
         imageSimilarityPrevious = 0;

         azimuthPrevious   = azimuthCurrent   - azimuthDelta;
         elevationPrevious = elevationCurrent - elevationDelta;

         VectorOf<double>& out = imageIdOutPort->prepare();
         out.resize(2,0);
         out[0] = imageIdPrevious;
         out[1] = coupling;
         imageIdOutPort->write();

         VectorOf<double> &vctPos = actionOutPort->prepare();
         vctPos.resize(5);
         vctPos(0) = azimuthPrevious;
         vctPos(1) = elevationPrevious;
         vctPos(2) = (double)(int)'a'; // absolute (neck reference) coordinates are sent
         vctPos(3) = (double)(int)'s'; // receiver module should do saccades
         vctPos(4) = 0; // saccade index
         actionOutPort->write();
      }

      if (debug) {
         if (mode == LEARNING_MODE)         cout << "ProceduralMemoryThread::run: Learning mode" << endl;
         else if (mode == PREDICTION_MODE)  cout << "ProceduralMemoryThread::run: Prediction mode" << endl;
         else if (mode == RECONSTRUCTION_MODE) cout << "ProceduralMemoryThread::run: Explanation mode" << endl;
         cout << "ProceduralMemoryThread::run: imageIdPrevious         " << imageIdPrevious         << endl;
         cout << "ProceduralMemoryThread::run: imageIdCurrent          " << imageIdCurrent          << endl;
         //cout << "ProceduralMemoryThread::run: imageSimilarityPrevious " << imageSimilarityPrevious << endl;
         //cout << "ProceduralMemoryThread::run: imageSimilarityCurrent  " << imageSimilarityCurrent  << endl;
         //cout << "ProceduralMemoryThread::run: azimuthPrevious         " << azimuthPrevious         << endl;
         //cout << "ProceduralMemoryThread::run: azimuthCurrent          " << azimuthCurrent          << endl;
         cout << "ProceduralMemoryThread::run: azimuthDelta            " << azimuthDelta            << endl;
         //cout << "ProceduralMemoryThread::run: elevationPrevious       " << elevationPrevious       << endl;
         //cout << "ProceduralMemoryThread::run: elevationCurrent        " << elevationCurrent        << endl;
         cout << "ProceduralMemoryThread::run: elevationDelta          " << elevationDelta          << endl;
         cout << "ProceduralMemoryThread::run: coupling                " << coupling                << endl;
         cout << endl;
      }

      data.saveDatabase();
   }
}

void ProceduralMemoryThread::threadRelease() 
{
   cout << "ProceduralMemoryThread::threadRelease: saving database" << endl;

   data.saveDatabase();

}


// ProceduralMemoryData constructor
// --------------------------------

ProceduralMemoryData::ProceduralMemoryData()
{
    debug = true;

    databaseName = "proceduralMemoryDatabase";
    databaseContext = "";

    numberOfImages  = MAX_NUMBER_OF_IMAGES;
    numberOfActions = (NORMALIZED_HORIZONTAL_DELTA_GAZE_RANGE+1)*(NORMALIZED_VERTICAL_DELTA_GAZE_RANGE+1);

    weightTotal = 0;

    for (int i = 0; i< numberOfImages; i++ ) {
       for (int j = 0; j < numberOfActions; j++) {
          for (int k = 0; i< numberOfImages; i++ ) {
             weightMatrix[i][j][k] = 0;
          }
       }
    }
}

// ProceduralMemoryData destructor

ProceduralMemoryData::~ProceduralMemoryData()
{
}



// ProceduralMemoryData database context setter

void ProceduralMemoryData::setDatabaseContext(string s)
{
    databaseContext = s;
}

//ProceduralMemoryData context getter

string ProceduralMemoryData::getDatabaseContext()
{
    return databaseContext;
}

// ProceduralMemoryData change database name
void ProceduralMemoryData::setDatabaseName(string s)
{
    databaseName = s;
}

// ProceduralMemoryData get database name
string ProceduralMemoryData::getDatabaseName()
{
    return databaseName; 
}


//ProceduralMemoryData loads the network of perception, actions, and association weights in the 'database' file

void ProceduralMemoryData::loadDatabase()
{

    string file;
    string databaseFolder;
    ifstream datafile;
    int i, j, k;
    double weight;

    if (databaseContext == "")
        databaseFolder = databaseName;
    else
        databaseFolder = databaseContext + "/" + databaseName;

    //cout << "ProceduralMemoryData::loadDatabase: reading from " << (databaseFolder + "/" + databaseName).c_str() << endl;

    datafile.open((databaseFolder + "/" + databaseName).c_str(),ios::in);

    if (datafile.is_open()) {

       datafile >> numberOfImages;
       datafile >> numberOfActions;
       datafile >> weightTotal;

       while (!datafile.eof()) {
          datafile >> i >> j >> k >> weight;
          weightMatrix[i][j][k] = weight;
          //cout << i << " " << j << " " << k << " " << weightMatrix[i][j][k] << endl;
       }
             
       datafile.close();

    }
	else {
		cout << "ProceduralMemoryData::loadDatabase: unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}   
}



//ProceduralMemoryData saves the network of perception, actions, and association weights in the 'database' file

void ProceduralMemoryData::saveDatabase()
{

    string file;
    string databaseFolder;
    if (databaseContext == "")
        databaseFolder = databaseName;
    else
        databaseFolder = databaseContext + "/" + databaseName;

    // cout << "ProceduralMemoryData::saveDatabase: writing to " << (databaseFolder + "/" + databaseName).c_str() << endl;

    ofstream datafile;
       
    datafile.open((databaseFolder + "/" + databaseName).c_str(), ios::out);

    if (datafile.is_open()) {
       datafile << numberOfImages << endl;
       datafile << numberOfActions << endl;
       datafile << weightTotal << endl;

       for (int i = 0; i < numberOfImages; i++ ) {
          for (int j = 0; j < numberOfActions; j++) {
             for (int k = 0; k < numberOfImages; k++ ) {
                if (weightMatrix[i][j][k] != 0) {
                   datafile << i << " " << j << " " << k << " " << weightMatrix[i][j][k] << endl;
                }
             }
          }
       }
    
       datafile.close();
    }
	else {
		cout << "ProceduralMemoryData::saveDatabase: unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}   

}



//ProceduralMemoryData update the weights for a given (P, A, P) triple.

void ProceduralMemoryData::updateWeights(int previousImageId, double deltaHorizontalGaze, double deltaVerticalGaze, int nextImageId) {
   
   int i; 
   int j;
   int k;

   i = previousImageId;
   j = gaze2action(deltaHorizontalGaze, deltaVerticalGaze); 
       //action2gaze(j, deltaHorizontalGaze, deltaVerticalGaze);  // check forward and backward mappings are consistent
   k = nextImageId;

   /* check bounds */

   if (i<0 || i>=numberOfImages) {
      cout << "ProceduralMemoryData::updateWeights Error: invalid image number " << i << endl; 
   }
   else if (j<0 || j>=numberOfActions) {
      cout << "ProceduralMemoryData::updateWeights Error: invalid action number " << j << endl; 
   }
   else if (k<0 || k>=numberOfImages) {
      cout << "ProceduralMemoryData::updateWeights Error: invalid image number " << k << endl; 
   }
   else {

      /* update matrices */

      weightMatrix[i][j][k]+=1;
      weightTotal++;

      if (false) {
         cout << "ProceduralMemoryData::updateWeights weightMatrix[" << i << "][" << j << "][" << k << "]: " << weightMatrix[i][j][k] << endl;
      }
   }
};

/*
 * ProceduralMemoryData compute coupling between a given (P, A, P) triple.
 *
 * For a given perception-action-perception triple (P_x, A_y, P_z) the couple strength is defined as
 * 
 *      s = W(P_x, A_y, P_z) / Sum_i Sum_j Sum_k W(P_i, A_j, P_k)
 *
 */

double ProceduralMemoryData::coupling(int x, int y, int z) {
   
   double coupling;

   coupling = 0;

   /* check bounds */

   if (x<0 || x>=numberOfImages) {
      cout << "ProceduralMemoryData::coupling Error: invalid image number " << x << endl; 
   }
   else if (y<0 || y>=numberOfActions) {
      cout << "ProceduralMemoryData::coupling Error: invalid action number " << y << endl; 
   }
   else if (z<0 || z>=numberOfImages) {
      cout << "ProceduralMemoryData::coupling Error: invalid image number " << z << endl; 
   }
   else {

      /* computer coupling value */

      coupling = weightMatrix[x][y][z] / weightTotal;
      
      if (false) {
         cout << "ProceduralMemoryData::coupling (" << x << ", " << y << ", " << z << "): " << coupling << endl;
      }
   }

   return coupling;
};


//ProceduralMemoryData get the number of images in the adjacency matrices.

int ProceduralMemoryData::getNumberOfImages() {
  return numberOfImages;
};


//ProceduralMemoryData get the number of actions in the adjacency matrices.

int ProceduralMemoryData::getNumberOfActions() {
  return numberOfActions;
};

// ProceduralMemoryData sum the 3D PAP adjacency matrix weights along the A axis 
// to generate an RGB image and contrast stretch so we can see the result 

void ProceduralMemoryData::generateWeightsImage(ImageOf<PixelRgb> &weightImage) {
   
   PixelRgb rgbPixel;
   double min;
   double max;

   /* integrate along the action axis */

   for (int i = 0; i< numberOfImages; i++ ) {
      for (int j = 0; j < numberOfImages; j++) {
         
         integratedWeightMatrix[i][j] = 0;

         for (int k = 0; k < numberOfActions; k++) {
            integratedWeightMatrix[i][j] += weightMatrix[i][k][j];
         }
      }
   }


   /* find the maximum  and minimum for contrast stretching */

   min =  1E10;
   max = -1E10;

   for (int i = 0; i < numberOfImages; i++ ) {
      for (int j = 0; j < numberOfImages; j++) {
         if (integratedWeightMatrix[i][j] > max) max = (float) integratedWeightMatrix[i][j];
         if (integratedWeightMatrix[i][j] < min) min = (float) integratedWeightMatrix[i][j];
      }
   }

   /* generate the image for display */

   for (int i = 0; i< numberOfImages; i++ ) {
      for (int j = 0; j < numberOfImages; j++) {

          rgbPixel.r = (unsigned char) (255 * (( (float)(integratedWeightMatrix[i][j]) - min)/(max-min)));
          rgbPixel.g = rgbPixel.r;
          rgbPixel.b = rgbPixel.r;
          weightImage(j,i) = rgbPixel; // NB reversal of indices as we want the images indices vertically in the image
          if (false) {
             cout << "ProceduralMemoryData::generateWeightsImage integratedWeightMatrix[" << i << "][" << j << "]: " << integratedWeightMatrix[i][j] << endl;
          }
       }
    }
};


//ProceduralMemoryData given the first P in a (P, A, P) triple, predict the subsequent associated A-P.

void ProceduralMemoryData::predict(int previousImageId, double & deltaHorizontalGaze, double & deltaVerticalGaze, int & nextImageId, double & eventCoupling) {
 
   /* 
    * find the action with the maximum weight associated with previousImageId 
    * then find the image with the maximum weight associated with this action
    */
   
   double maxWeight;
   int maxj;
   int maxk;

   deltaHorizontalGaze = 0;
   deltaVerticalGaze = 0;
   nextImageId = 0;
   eventCoupling = 0;

   /* check for valid parameters */

   if (previousImageId < 0 || previousImageId >= numberOfImages) {
      cout << "ProceduralMemoryData::predict Error: invalid image number " << previousImageId << endl; 
   }
   else {

      /* find action-image pair with greatest weight associated with previousImageId */

      maxWeight = 0;
      maxj = 0;
      maxk = 0;
 
      for (int j = 0; j < numberOfActions; j++) {
         for (int k = 0; k < numberOfImages; k++) {
            if (weightMatrix[previousImageId][j][k] > maxWeight) {
               maxWeight = weightMatrix[previousImageId][j][k];
               maxj = j;
               maxk = k;
           }
        }
      }
     
      action2gaze(maxj, deltaHorizontalGaze, deltaVerticalGaze);

      nextImageId = maxk;

      eventCoupling = coupling(previousImageId, maxj, nextImageId);
   }

   // cout << "ProceduralMemoryData::predict     "  << previousImageId << " (" << deltaHorizontalGaze << ", " << deltaVerticalGaze << ") " << nextImageId  <<  ": " << eventCoupling << endl;

}

//ProceduralMemoryData given the second P in a (P, A, P) triple, reconstruct the prior associated P-A.

void ProceduralMemoryData::reconstruct(int & previousImageId, double & deltaHorizontalGaze, double & deltaVerticalGaze, int nextImageId, double & eventCoupling) {
 
   double maxWeight;
   int maxi;
   int maxj;

   previousImageId = 0;
   deltaHorizontalGaze = 0;
   deltaVerticalGaze = 0;
   eventCoupling = 0;

   /* check for valid parameters */

   if (nextImageId < 0 || nextImageId >= numberOfImages) {
      cout << "ProceduralMemoryData::reconstruct Error: invalid image number " << nextImageId << endl; 
   }
   else {

      /* find action-image pair with greatest weight associated with previousImageId */

      maxWeight = 0;
      maxi = 0;
      maxj = 0;
    
      for (int i = 0; i < numberOfImages; i++) {
         for (int j = 0; j < numberOfActions; j++) {
            if (weightMatrix[i][j][nextImageId] > maxWeight) {
               maxWeight = weightMatrix[i][j][nextImageId];
               maxi = i;
               maxj = j;
            }
         }
      }

      action2gaze(maxj, deltaHorizontalGaze, deltaVerticalGaze);

      previousImageId = maxi;

      eventCoupling = coupling(previousImageId, maxj, nextImageId);
   }

   // cout << "ProceduralMemoryData::reconstruct "  << previousImageId << " (" << deltaHorizontalGaze << ", " << deltaVerticalGaze << ") " << nextImageId << ": " << eventCoupling << endl;
}


/*
 * ProceduralMemoryData convert from (deltaHorizontalGaze, deltaVerticalGaze) to actionOffset 
 * 
 * That is, map from 2D gaze change coordinates to 1D action coordinates 
 * for use with the paMatrix and apMatrix graph adjacency matrices
 */

int ProceduralMemoryData::gaze2action(double deltaHorizontalGaze, double deltaVerticalGaze) {
 
   int    actionOffset;
   double normalizedDHG;
   double normalizedDVG;

   normalizedDHG = (int) (((deltaHorizontalGaze / HORIZONTAL_DELTA_GAZE_RANGE) * NORMALIZED_HORIZONTAL_DELTA_GAZE_RANGE) + NORMALIZED_HORIZONTAL_DELTA_GAZE_RANGE/2); 
   normalizedDVG = (int) (((deltaVerticalGaze   / VERTICAL_DELTA_GAZE_RANGE)   * NORMALIZED_VERTICAL_DELTA_GAZE_RANGE) + NORMALIZED_VERTICAL_DELTA_GAZE_RANGE/2); 

   actionOffset = (int) NORMALIZED_VERTICAL_DELTA_GAZE_RANGE * (int) normalizedDHG + (int) normalizedDVG;

   // cout << "ProceduralMemoryData::gaze2action " << (int) deltaHorizontalGaze << ", " << (int) deltaVerticalGaze << " -> " << actionOffset << endl;
   
   return actionOffset;
}

/*
 * ProceduralMemoryData convert from actionOffset to (deltaHorizontalGaze, deltaVerticalGaze)   
 * 
 * That is, map from 1D action coordinates to 2D gaze change coordinates 
 * for use with the paMatrix and apMatrix graph adjacency matrices
 */

void ProceduralMemoryData::action2gaze(int actionOffset, double & deltaHorizontalGaze, double & deltaVerticalGaze) {
 
   double normalizedDHG;
   double normalizedDVG;

   normalizedDHG = (int) ( actionOffset / NORMALIZED_VERTICAL_DELTA_GAZE_RANGE ) - NORMALIZED_HORIZONTAL_DELTA_GAZE_RANGE/2;
   normalizedDVG = (int) ( actionOffset %  NORMALIZED_VERTICAL_DELTA_GAZE_RANGE ) - NORMALIZED_VERTICAL_DELTA_GAZE_RANGE/2;
  
   deltaHorizontalGaze = (normalizedDHG / NORMALIZED_HORIZONTAL_DELTA_GAZE_RANGE) * HORIZONTAL_DELTA_GAZE_RANGE;
   deltaVerticalGaze   = (normalizedDVG / NORMALIZED_VERTICAL_DELTA_GAZE_RANGE)   * VERTICAL_DELTA_GAZE_RANGE;

   // cout << "ProceduralMemoryData::action2gaze "  << deltaHorizontalGaze << ", " << deltaVerticalGaze << " <- " << actionOffset << endl;
   
}