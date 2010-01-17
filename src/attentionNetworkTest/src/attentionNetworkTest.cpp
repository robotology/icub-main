// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon 
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
 * Initial version
 * David Vernon 15/01/10
 */

 

// iCub
#include <iCub/attentionNetworkTest.h>

//opencv
#include <cv.h>
#include <highgui.h>

//ANTData constructor
ANTData::ANTData()
{
    databaseName = "ANTdatabase";
    databaseContext = "";
}

//ANTData deconstructor
ANTData::~ANTData()
{
}


//ANTData image getter method
vector<ImageOf<PixelRgb> >& ANTData::images()
{
    return imgs;
}

//database context setter
void ANTData::setDatabaseContext(string s)
{
    databaseContext = s;
}

//database context getter
string ANTData::getDatabaseContext()
{
    return databaseContext;
}

//change database name
void ANTData::setDatabaseName(string s)
{
    databaseName = s;
}

//get database name
string ANTData::getDatabaseName()
{
    return databaseName; 
}


//Loads a vector of JPG images into a bottle based on our 'database' file
void ANTData::loadDatabase()
{

    string file;
    string databaseFolder;
    if (databaseContext == "")
        databaseFolder = databaseName;
    else
        databaseFolder = databaseContext + "/" + databaseName;

    cout << "ANTData::loadDatabase: trying to read from " << (databaseFolder + "/" + databaseName).c_str() << endl;

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
		cout << "ANTData::loadDatabase: unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}
}
    
 

bool AttentionNetworkTest::configure(yarp::os::ResourceFinder &rf)
{

    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                            Value("attentionNetworkTest"), 
                            "module name (string)").asString(); 

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());

  
    // parse parameters or assign default values (append to getName=="/attentionNetworkTest")

    ANTimageOutPortName  = "/";
    ANTimageOutPortName += getName(
                           rf.check("ANTimageOutPort",
				           Value("/ANTimage:o"),
				           "Output image port (string)").asString()
                           );

    databaseName         = rf.check("database",
					       Value("ANTdatabase"),
					       "Database name (string)").asString().c_str();

    path                 = rf.check("path",
				           Value("~/iCub/app/attentionNetworkTest"),
    			           "complete path to database directory").asString().c_str(); 

    d1                   = rf.check("d1",
                           Value(1600),
                           "fixation time value (double)").asInt();

    d2                   = rf.check("d2",
                           Value(100),
                           "cue time value (double)").asInt();
  
    d3                   = rf.check("d3",
                           Value(400),
                           "fixation time value (double)").asInt();

    d4                   = rf.check("d4",
                           Value(1700),
                           "target time value (double)").asInt();

    d5                   = rf.check("d5",
                           Value(0),
                           "fixation time value (double)").asInt();

    printf("attentionNetworkTest: parameters are \n%s\n%s\n%d\n%d\n%d\n%d\n%d\n\n",
           databaseName.c_str(), 
           path.c_str(), 
           d1, d2, d3, d4, d5);

    // create attentionNetworkTest ports

    ANTimageOut.open(ANTimageOutPortName.c_str());
    
   /* create the thread and pass pointers to the module parameters */

   attentionNetworkTestThread = new AttentionNetworkTestThread(&ANTimageOut,
                                                               &databaseName,
                                                               &path,
                                                               &d1,
                                                               &d2,
                                                               &d3,
                                                               &d4,
                                                               &d5);

   /* now start the thread to do the work */

   attentionNetworkTestThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true;
}

bool AttentionNetworkTest::updateModule()
{		
    return true;
}

bool AttentionNetworkTest::interruptModule()
{
    // interrupt ports gracefully

    ANTimageOut.interrupt();

    return true;	
}

bool AttentionNetworkTest::close()
{
    cout << "Closing AttentionNetworkTest...\n\n";

    // close attentionNetworkTest ports
    
    ANTimageOut.close();

    /* stop the thread */

    attentionNetworkTestThread->stop();
   
    return true;
}


//module periodicity (seconds), called implicitly by module

double AttentionNetworkTest::getPeriod()

{
    return 0.1; //module periodicity (seconds)
}


AttentionNetworkTestThread::AttentionNetworkTestThread(BufferedPort<ImageOf<PixelRgb> > *ANTimageOut,
                                                                   string               *databaseName,
                                                                   string               *path,
                                                                   int                  *d1,
                                                                   int                  *d2,
                                                                   int                  *d3,
                                                                   int                  *d4,
                                                                   int                  *d5)
{
   debug = false;

   ANTimageOutPort       = ANTimageOut;
   databaseNameValue     = databaseName;
   pathValue             = path;
   d1Value               = d1;
   d2Value               = d2;
   d3Value               = d3;
   d4Value               = d4;
   d5Value               = d5;


   if (debug) {
      cout << "AttentionNetworkTestThread: database name  " << *databaseNameValue << endl;
      cout << "AttentionNetworkTestThread: path           " << *pathValue << endl;
      cout << "AttentionNetworkTestThread: d1             " << *d1Value << endl;
      cout << "AttentionNetworkTestThread: d2             " << *d2Value << endl;
      cout << "AttentionNetworkTestThread: d3             " << *d3Value << endl;
      cout << "AttentionNetworkTestThread: d4             " << *d4Value << endl;
      cout << "AttentionNetworkTestThread: d5             " << *d5Value << endl;
   }
}

bool AttentionNetworkTestThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = true;

    data.setDatabaseContext(*pathValue);

	data.setDatabaseName(*databaseNameValue);

    if (debug)
       std::cout << "AttentionNetworkTestThread::threadInit: databaseName    " << (*databaseNameValue).c_str() << endl;
    
    data.loadDatabase();

    return true;
}

void AttentionNetworkTestThread::run(){

   debug = true;

   if (debug) {
      cout << "AttentionNetworkTestThread::run: database name  " << *databaseNameValue << endl;
      cout << "AttentionNetworkTestThread::run: path           " << *pathValue << endl;
      cout << "AttentionNetworkTestThread::run: d1             " << *d1Value << endl;
      cout << "AttentionNetworkTestThread::run: d2             " << *d2Value << endl;
      cout << "AttentionNetworkTestThread::run: d3             " << *d3Value << endl;
      cout << "AttentionNetworkTestThread::run: d4             " << *d4Value << endl;
      cout << "AttentionNetworkTestThread::run: d5             " << *d5Value << endl;
   }

   /*
    ANT images
    ANT00   fixation
    ANT01   cue centre
    ANT02   cue top and bottom
    ANT03   cue top
    ANT04   cue bottom
    ANT05   target neutral top right
    ANT06   target neutral bottom right
    ANT07   target neutral top left
    ANT08   target neutral bottom left
    ANT09   target congurent top right
    ANT10   target congurent bottom right
    ANT11   target congurent top left
    ANT12   target congurent bottom left
    ANT13   target incongurent top right
    ANT14   target incongurent bottom right
    ANT15   target incongurent top left
    ANT16   target incongurent bottom left
   */

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true

      std::vector<ImageOf<PixelRgb> >& images = data.images();

      for (cueCondition = 3; cueCondition < 4; cueCondition++) {
         for (location = 0; location <2 ; location++) {
            for (direction = 0; direction < 2; direction++) {
               for (flankerCondition = 0; flankerCondition <3; flankerCondition++) {

                  /* fixation */

                  imageId = 0;
            	  it = images.begin() + imageId; // set the iterator
            	  ANTimage = *it;                // retrieve the image
                  ANTimageOutPort->prepare() = ANTimage;
                  ANTimageOutPort->write();
                  pause(*d1Value);

                  /* cue */

                  if (cueCondition < 3) {
                     // not spatial 
                     imageId = cueCondition;
                  }
                  else {
                     // spatial ... depends on location
                     imageId = cueCondition+location;
                  }

	              it = images.begin() + imageId; // set the iterator
	              ANTimage = *it;                // retrieve the image
                  ANTimageOutPort->prepare() = ANTimage;
                  ANTimageOutPort->write();
                  pause(*d2Value);

                  /* fixation */

                  imageId = 0;
	              it = images.begin() + imageId; // set the iterator
	              ANTimage = *it;                // retrieve the image
                  ANTimageOutPort->prepare() = ANTimage;
                  ANTimageOutPort->write();
                  pause(*d3Value);

                  /* target */

                  imageId = 5 + 4*flankerCondition + location + 2*direction;

	              it = images.begin() + imageId; // set the iterator
	              ANTimage = *it;                // retrieve the image
                  ANTimageOutPort->prepare() = ANTimage;
                  ANTimageOutPort->write();
                  pause(*d4Value);

                  /* fixation */

                  imageId = 0;
	              it = images.begin() + imageId; // set the iterator
	              ANTimage = *it;                // retrieve the image
                  ANTimageOutPort->prepare() = ANTimage;
                  ANTimageOutPort->write();
                  pause(*d5Value);
               }
            }
         }
      }
   }

}

void AttentionNetworkTestThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */


}


