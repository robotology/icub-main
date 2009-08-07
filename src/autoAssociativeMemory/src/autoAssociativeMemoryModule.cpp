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

// iCub
#include <iCub/autoAssociativeMemoryModule.h>

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
    
        double comp = 1 - cvCompareHist(currHist, refHist, CV_COMP_BHATTACHARYYA);  //this method of in tersection seems to produce better results
        //cout << comp << " ";    //once again, only for debugging purposes
        //do a histogram intersection, and check it against the threshold 
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
        cvSaveImage((databasefolder + "/" + oss.str()).c_str(), currImg);
        
        ofstream of;
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
bool AutoAssociativeMemoryModule::open(Searchable &config)
{
    // read options from command line
    Bottle initialBottle(config.toString().c_str());
    
    // if autoAssociativeMemory.ini exists, then load it
    ConstString initializationFile = initialBottle.check("from",
						       Value("autoAssociativeMemory.ini"),
						       "Initialization file (string)").asString();
    
    ConstString context = initialBottle.check("context",
					    Value("autoAssociativeMemory"),
					    "Context (string)").asString();

    // create and initialize resource finder
    ResourceFinder rf;
    rf.setDefaultContext(context.c_str());
    rf.setDefaultConfigFile(initializationFile.c_str());
    rf.configure("ICUB_ROOT", 0, NULL);

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());

    // parse parameters or assign default values (append to getName=="/aam")
    _namePortImageIn = botConfig.check("portImageIn",
				     Value(getName("image:i")),
				     "Input image port (string)").asString();
    _namePortThresholdIn = botConfig.check("portThresholdIn",
					 Value(getName("threshold:i")),
					 "Input threshold port (string)").asString();
    _namePortImageOut = botConfig.check("portImageOut",
				      Value(getName("image:o")),
				      "Output image port (string)").asString();
    _namePortValueOut = botConfig.check("portValueOut",
				      Value(getName("value:o")),
				      "Output value port (string)").asString();

    string databaseName = botConfig.check("database",
					Value("defaultDatabase"),
					"Database name (string)").asString().c_str();
    string path = botConfig.check("path",
				Value("~/iCub/app/"),
				"complete path to context").asString().c_str();
    double thr = botConfig.check("threshold",
                               Value(0.6),
                               "initial threshold value (double)").asDouble();
				   
    data.setThreshold(thr);
  
    string ctxt = path;
    ctxt += context.c_str();
    data.setDatabaseContext(ctxt);
    cout << "context: " << ctxt << endl;
    data.setDatabaseName(databaseName);
    cout << "databaseName: " << databaseName << endl;
    data.loadDatabase();

    // create AAM ports
    _portThresholdIn = new ThresholdReceiver(&data, &_portImageIn, &_portImageOut, &_portValueOut);

    _portThresholdIn->useCallback();

    _portImageIn.open(_namePortImageIn);
    _portThresholdIn->open(_namePortThresholdIn);
    _portImageOut.open(_namePortImageOut);
    _portValueOut.open(_namePortValueOut);
	
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
