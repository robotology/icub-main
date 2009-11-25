// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Alberto Bietti, Logan Niehaus, Giovanni Saponaro 
 * email:   <firstname.secondname>@robotcub.org
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
 * it is now  getName(rf.check("portImageIn", Value("image:i"), "Input image port (string)").asString());
 * instead of rf.check("portImageIn", Value(getName("image:i")), "Input image port (string)").asString();
 * as the latter only prefixed the default value, not the parameter value
 * David Vernon 19/08/09
 *
 * Completed compliance with standards and changed to new convention for handling port names and module name
 * (module name has no / prefix and port names _always_ have / prefix)
 * David Vernon 26/08/09  
 *
 * Normalized histograms before computing the histogram intersection. 
 * David Vernon 14/10/09
 */
 
 

// iCub
#include <iCub/autoAssociativeMemoryModule.h>
//opencv
#include <cv.h>
#include <highgui.h>

//HistMatchData constructor
HistMatchData::HistMatchData()
{
    setThreshold(0.6);  //default threshold value
    databaseName = "defaultDatabase";
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

    // cout << "autoAssociativeMemory: trying to read from " << (databaseFolder + "/" + databaseName).c_str() << endl;

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
		cout << "autoAssociativeMemory: unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}
}
    
ThresholdReceiver::ThresholdReceiver(HistMatchData* d, BufferedPort<ImageOf<PixelRgb> >* iPortIn, BufferedPort<ImageOf<PixelRgb> >* iPort, BufferedPort<Bottle>* mPort) : data(d), imgPortIn(iPortIn), imgPort(iPort), matchPort(mPort) { }


void ThresholdReceiver::onRead(Bottle& t)
{
    double threshold = t.get(0).asDouble();
    std::cout << "the threshold is now: " << threshold << std::endl;
    data->setThreshold(threshold);
    
    ImageOf<PixelRgb>* imgIn = imgPortIn->read();
    
    if (imgIn == NULL) 
   		return;
   	
   	ImageOf<PixelRgb>& img = *imgIn;
    
    data->imgMutex.wait();

    std::vector<ImageOf<PixelRgb> >& images = data->images();
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

    double matchValue;
    matchValue = threshold;
    bool found = false;
    std::vector<ImageOf<PixelRgb> >::iterator it;
    ImageOf<PixelRgb> matchImage;
    int matchId = 0;
    // std::cout << "threshold: " << threshold << " ";  //for debugging purposes only
    
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


        if (comp > matchValue)
	    {
	      matchValue = comp;
	      matchImage = *it;
	      matchId = it - images.begin();
	      found = true;
	    }
        cvReleaseImage(&refImg); cvReleaseImage(&refImgH); cvReleaseImage(&refImgS); cvReleaseImage(&refImgV);
        cvReleaseHist(&refHist);
      }
    //if the image produces a match
    if (found)
      {
        imgPort->prepare() = matchImage;
        imgPort->write();
        
        Bottle& out = matchPort->prepare();
        out.clear();
        out.addInt(matchId);
        out.addDouble(matchValue);
        matchPort->write();
        cout << "found" << endl;
      }
    //no match found
    else
      {
    
        //add the image to the database in memory, then into the filesystem.
        images.push_back(img);
        imgPort->prepare() = img;
        imgPort->write();
    
        //create a filename that is imageXX.jpg
        Bottle& out = matchPort->prepare();
        out.clear();
        out.addInt(images.size()-1);
        out.addDouble(1.0);
        matchPort->write();
        cout << "stored" << endl;
        string s;
        ostringstream oss(s);
        oss << "image" << images.size()-1 << ".jpg";
        cout << oss.str() << endl;
    
    
        //write it out to the proper database

        string databasefolder = data->getDatabaseContext() + "/" + data->getDatabaseName();
        cvCvtColor(img.getIplImage(), currImg, CV_RGB2BGR);  //opencv stores images as BGR

		// cout << "autoAssociativeMemory: trying to save to " << (databasefolder + "/" + oss.str()).c_str() << endl;

        cvSaveImage((databasefolder + "/" + oss.str()).c_str(), currImg);
        
        ofstream of;

	    // cout << "autoAssociativeMemory: trying to save to " << (databasefolder + "/" + data->getDatabaseName()).c_str() << endl;

        of.open((databasefolder + "/" + data->getDatabaseName()).c_str(),ios::app);
        of << oss.str() << endl;
        of.close();
    
      }
    
    cvReleaseImage(&currImg); cvReleaseImage(&currImgH); cvReleaseImage(&currImgS); cvReleaseImage(&currImgV);
    cvReleaseHist(&currHist);
    
    data->imgMutex.post();
}

/** AAM Module open. This is inherited from the module class. 
*/
bool AutoAssociativeMemoryModule::configure(yarp::os::ResourceFinder &rf)
{
    /* get the module name which will form the stem of all module port names */

    moduleName            = rf.check("name", 
                            Value("aam"), 
                            "module name (string)").asString(); 

    /*
     * before continuing, set the module name before getting any other parameters, 
     * specifically the port names which are dependent on the module name
     */
   
    setName(moduleName.c_str());

  
    // parse parameters or assign default values (append to getName=="/aam")

    _namePortImageIn     = "/";
    _namePortImageIn    += getName(
                           rf.check("portImageIn",
				           Value("/image:i"),
				           "Input image port (string)").asString()
                           );

    _namePortThresholdIn = "/";
    _namePortThresholdIn+= getName(
                           rf.check("portThresholdIn",
					       Value("/threshold:i"),
					       "Input threshold port (string)").asString()
                           );

    _namePortImageOut    = "/";
    _namePortImageOut   += getName(
                           rf.check("portImageOut",
				           Value("/image:o"),
				           "Output image port (string)").asString()
                           );

    _namePortValueOut    = "/";
    _namePortValueOut   += getName(
                           rf.check("portValueOut",
				           Value("/value:o"),
				           "Output value port (string)").asString()
                           );

    string databaseName  = rf.check("database",
					       Value("defaultDatabase"),
					       "Database name (string)").asString().c_str();

    string path          = rf.check("path",
				           Value("~/iCub/app/autoAssociativeMemory"),
    			           "complete path to context").asString().c_str(); 

    double thr           = rf.check("threshold",
                           Value(0.6),
                           "initial threshold value (double)").asDouble();

    // printf("autoAssociativeMemory: parameters are \n%s\n%s\n%s\n%s\n%s\n%s\n%f\n\n",
    //       _namePortImageIn.c_str(),_namePortThresholdIn.c_str(), _namePortImageOut.c_str(), _namePortValueOut.c_str(), databaseName.c_str(), path.c_str(), thr);
    
    data.setThreshold(thr);
   

    /* Changed the way the database context is handled:
     * it used to be the concatenation of the path parameter and the context parameter
     * however, at present, there is no way to find out the context if it is not set explicitly as a parameter, 
     * either in the config file on the command line, such as would be the case when using the default configuration
     * In addition, the current policy is for the context to be $ICUB_ROOT/app/moduleName/conf and this means that
     * the database directory would be under the conf directory and this doesn't make much sense.
     * Consequently, the path parameter is now redefined to be the full path to the database directory
     * and can, therefore, be located anywhere you like (specifically, it doesn't have to be located in the iCub repository)
     */

    data.setDatabaseContext(path);
    //std::cout << "autoAssociativeMemory: databaseContext " << path.c_str() << endl << endl;
    
	data.setDatabaseName(databaseName);
    //std::cout << "autoAssociativeMemory: databaseName    " << databaseName.c_str() << endl << endl;
    
    data.loadDatabase();

    // create AAM ports
    _portThresholdIn = new ThresholdReceiver(&data, &_portImageIn, &_portImageOut, &_portValueOut);

    _portThresholdIn->useCallback();

    _portImageIn.open(_namePortImageIn.c_str());
    _portThresholdIn->open(_namePortThresholdIn.c_str());
    _portImageOut.open(_namePortImageOut.c_str());
    _portValueOut.open(_namePortValueOut.c_str());

    // attach a port to the module
    // so that messages received from the port are redirected
    // to the respond method
   
    _nameHandlerPort =  "/";
    _nameHandlerPort += getName();          
 
    handlerPort.open(_nameHandlerPort.c_str());  
 
    attach(handlerPort);   
    attachTerminal();     //attach to terminal
	
    return true;
}

bool AutoAssociativeMemoryModule::updateModule()
{		
    return true;
}

bool AutoAssociativeMemoryModule::interruptModule()
{
    // interrupt ports gracefully
    _portImageIn.interrupt();
    _portThresholdIn->interrupt();
    _portImageOut.interrupt();
    _portValueOut.interrupt();
	
    return true;	
}

bool AutoAssociativeMemoryModule::close()
{
    cout << "Closing Auto-Associative Memory...\n\n";

    // close AAM ports
    _portImageIn.close();
    _portThresholdIn->close();
    _portImageOut.close();
    _portValueOut.close();
  
    // free data structures, delete dynamically-created variables
    
    // is this necessary?
    //Network::fini();
    
    return true;
}


//module periodicity (seconds), called implicitly by module

double AutoAssociativeMemoryModule::getPeriod()

{
    return 0.1; //module periodicity (seconds)
}

// Message handler. 
// This allows other modules or a user to send commands to the module (in bottles)
// This functionality is not yet used but it may come in useful later on
// if/when we wish to change the parameters of the module at run time
// For now, just echo all received messages.

bool AutoAssociativeMemoryModule::respond(const Bottle& command, Bottle& reply) 
{
    // Message handler. Just echo all received messages.
	
    cout<<"Got something, echo is on"<<endl;
    if (command.get(0).asString()=="quit")
        return false;     
    else
        // do something and then reply
        reply=command;
    return true;
}

