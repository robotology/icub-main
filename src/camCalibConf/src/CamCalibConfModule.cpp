// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibConfModule.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

CamCalibConfModule::CamCalibConfModule(){

    _ocvImgIn = NULL;
    _ocvImgTmp1 = NULL;
    _ocvImgTmp2 = NULL;
    _ocvImgOut = NULL;
    
    _oldImgSize.width = 160;
    _oldImgSize.height = 120;
    initImages(_oldImgSize.width,_oldImgSize.height); 

    _corners = NULL;
    _imagePoints = NULL;
    _objectPoints = NULL;
    _pointCounts = NULL;
    _intrinsicMatrix = NULL;    
    _distortionCoeffs = NULL;       
    //_rotationVectors = NULL;       
    //_translationVectors = NULL;

    _grabFlag = false;
    _patternImageCounter = 0;
}

CamCalibConfModule::~CamCalibConfModule(){
   // data is released trough close/interrupt methods
}

double CamCalibConfModule::getPeriod(){
	return 0.0;
}

bool CamCalibConfModule::configure(ResourceFinder& rf){
       
	ConstString str = rf.check("name", Value("/camCalibConf"), "module name (string)").asString();
	setName(str.c_str()); // modulePortName  
	attachTerminal();

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());
    botConfig.setMonitor(rf.getMonitor());
    // is group option present?
    Value *valGroup; // check assigns pointer to reference
    if(rf.check("group", valGroup, "Configuration group to load module options from (string).")){
        string strGroup = valGroup->asString().c_str();
        // is group a valid bottle?
        if (!rf.findGroup(strGroup.c_str()).isNull()){
            botConfig.clear();
            botConfig.fromString(rf.findGroup(strGroup.c_str(), string("Loading configuration from group " + strGroup).c_str()).toString().c_str());
        }
        else{
            cout << endl << "Group " << strGroup << " not found." << endl;
            return false;
        }
    }

    _numPatternImagesRequired = botConfig.check("numPatternImagesRequired",
                                            Value(25),
                                            "Number of images of the pattern required to run calibration (int").asInt();
    _numPatternInnerCornersX = botConfig.check("numPatternInnerCornersX",
                                            Value(8),
                                            "Number of inner corners (where two black squares touch) in x-direction (int)").asInt();
    _numPatternInnerCornersY = botConfig.check("numPatternInnerCornersY",
                                            Value(6),
                                            "Number of inner corners (where two black squares touch) in y-direction (int)").asInt();
    _patternSquareSideLength = botConfig.check("patternSquareSideLength",
                                            Value(28.0),
                                            "Length of one side of a square in the pattern in mm (double)").asDouble();
    _outputFilename = botConfig.check("outputFilename",
                                            Value(""),
                                            "File to write/update camera calibration parameters (string)").asString();
    _outputGroupname = botConfig.check("outputGroupname",
                                            Value(""),
                                            "Configuration group to update within existing file (string)").asString();
       

    _corners = new CvPoint2D32f[_numPatternInnerCornersX *
                                _numPatternInnerCornersY];
    _imagePoints = cvCreateMat( _numPatternImagesRequired * 
                                _numPatternInnerCornersX *
                                _numPatternInnerCornersY,
                                2,
                                CV_32F);
    _objectPoints = cvCreateMat(_numPatternImagesRequired * 
                                _numPatternInnerCornersX *
                                _numPatternInnerCornersY,
                                3,
                                CV_32F);
    _pointCounts = cvCreateMat( _numPatternImagesRequired,
                                1,
                                CV_32S);

    // number of points in each presented pattern
    for (int i = 0; i < _numPatternImagesRequired; i++)
        CV_MAT_ELEM( *_pointCounts, int, i, 0) = _numPatternInnerCornersX *
                                                 _numPatternInnerCornersY;

    // size of the pattern (2D) in 3D world measurements (mm)
    int pos = 0;
    for (int n = 0; n < _numPatternImagesRequired; n++){
        for (int y = 0; y < _numPatternInnerCornersY; y++){
            for (int x = 0; x < _numPatternInnerCornersX; x++){
                CV_MAT_ELEM( *_objectPoints, float, pos+x, 0) = (float)(x * _patternSquareSideLength + _patternSquareSideLength);
                CV_MAT_ELEM( *_objectPoints, float, pos+x, 1) = (float)(y * _patternSquareSideLength + _patternSquareSideLength);
                CV_MAT_ELEM( *_objectPoints, float, pos+x, 2) = 0.0f;
            }
            pos += _numPatternInnerCornersX;
        }
    }
    // check it 
   /* for (int i = 0; i < (_numPatternInnerCornersY * _numPatternInnerCornersX * _numPatternImagesRequired); i++){
        cout << CV_MAT_ELEM( *_objectPoints, float, i, 0) << " " << CV_MAT_ELEM( *_objectPoints, float, i, 1) << " " << CV_MAT_ELEM( *_objectPoints, float, i, 2) << endl;
    }*/

    // output data structures
    _intrinsicMatrix = cvCreateMat( 3, 3, CV_32F);   
    _distortionCoeffs = cvCreateMat( 1, 4, CV_32F);
    //_rotationVectors = cvCreateMat( 3, _numPatternImagesRequired, CV_32F);
    //_translationVectors = cvCreateMat( 3, _numPatternImagesRequired, CV_32F);
    
    _grabFlag = false;
    _patternImageCounter = 0;

    _prtImg.open(getName("/image"));
    _configPort.open(getName("/conf"));
    attach(_configPort);

    cout << endl;
    cout << "We need to acquire " << _numPatternImagesRequired << " images of the chessboard pattern." << endl;
    cout << endl;
    cout << "Present the pattern and check on the output image if inner corners are detected." << endl;
    cout << "For each position of the recognized pattern you wish to use, press 'g' (grab) followed by the return key." << endl;
    cout << endl;

    return true;
}

bool CamCalibConfModule::close(){
    
    _prtImg.close();

    if (_ocvImgIn != NULL)
        cvReleaseImage(&_ocvImgIn);
    _ocvImgIn = NULL;
    if (_ocvImgTmp1 != NULL)
        cvReleaseImage(&_ocvImgTmp1);
    _ocvImgTmp1 = NULL;
    if (_ocvImgTmp2 != NULL)
        cvReleaseImage(&_ocvImgTmp2);
    _ocvImgTmp2 = NULL;
    if (_ocvImgOut != NULL)
        cvReleaseImage(&_ocvImgOut);
    _ocvImgOut = NULL;

    _oldImgSize.width = -1;
    _oldImgSize.height = -1;

    if (_corners != NULL)
        delete [] _corners;
    _corners = NULL;
    if (_imagePoints != NULL)
        cvReleaseMat(&_imagePoints);
    _imagePoints = NULL;
    if (_objectPoints != NULL)
        cvReleaseMat(&_objectPoints);
    _objectPoints = NULL;
    if (_pointCounts != NULL)
        cvReleaseMat(&_pointCounts);
    _pointCounts = NULL;
    if (_intrinsicMatrix != NULL)
        cvReleaseMat(&_intrinsicMatrix);
    _intrinsicMatrix = NULL;
    if (_distortionCoeffs != NULL)
        cvReleaseMat(&_distortionCoeffs);
    _distortionCoeffs = NULL;
    /*if (_rotationVectors != NULL)
        cvReleaseMat(&_rotationVectors);
    _rotationVectors = NULL;
    if (_translationVectors != NULL)
        cvReleaseMat(&_intrinsicMatrix);
    _intrinsicMatrix = NULL;*/

    return true;
}

bool CamCalibConfModule::interruptModule(){
    
    _prtImg.interrupt();
    return true;
}

bool CamCalibConfModule::updateModule(){
    
	int i;	// otherwise redefinition error with old ms VC6 compiler
    yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
    yrpImgIn = _prtImg.read();
    if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;
    
    _semaphore.wait();

    // size changed?
    if (_oldImgSize.width  != yrpImgIn->width() || 
        _oldImgSize.height != yrpImgIn->height())
        initImages(yrpImgIn->width(), yrpImgIn->height());

    int width = yrpImgIn->width();
    int height = yrpImgIn->height();
    int widthHalf = width/2;
    int heightHalf = height/2;

    // convert to BGR color order for opencv (in any case)
    cvCvtColor(yrpImgIn->getIplImage(), _ocvImgIn, CV_RGB2BGR);

    int found = 0;
    int numCorners = -1;
    
    // find pattern
    found = cvFindChessboardCorners(_ocvImgIn, 
                                    cvSize(_numPatternInnerCornersX, 
                                           _numPatternInnerCornersY), 
                                    _corners, 
                                    &numCorners,
                                    CV_CALIB_CB_NORMALIZE_IMAGE+
                                    CV_CALIB_CB_ADAPTIVE_THRESH);

    // draw pattern
    if (found)
        cvDrawChessboardCorners( _ocvImgIn, cvSize(_numPatternInnerCornersX, 
                                                   _numPatternInnerCornersY),
                                _corners, 
                                numCorners,
                                found );
    
    // if corners extracted and user wants to grab them
    if (found && _grabFlag){
        
        cout << "Pattern corners successfully extracted for image " << _patternImageCounter+1 << endl;
        cout << endl;

        // check numCorners
        if (numCorners != _numPatternInnerCornersX*_numPatternInnerCornersY){
            cout << "Unexpected number of extracted corners! Exiting..." << endl;
            Time::delay(5.0);
            return false;
        }

        // copy corners to imagePoints matrix
        for (i = 0; i < numCorners; i++){
            CV_MAT_ELEM( *_imagePoints, float, _patternImageCounter * numCorners + i, 0) = _corners[i].x;
            CV_MAT_ELEM( *_imagePoints, float, _patternImageCounter * numCorners + i, 1) = _corners[i].y;
        }

        _patternImageCounter++;
        _grabFlag = false;
    }

    // convert image back to RGB image for yarp
    cvCvtColor(_ocvImgIn, _ocvImgOut, CV_BGR2RGB);

    // buffering old image size
    _oldImgSize.width  = _ocvImgOut->width;
    _oldImgSize.height = _ocvImgOut->height;

    // write image back to yarp network
    yarp::sig::ImageOf<PixelRgb>& yrpImgOut = _prtImg.prepare();
    yrpImgOut.wrapIplImage(_ocvImgOut);
    _prtImg.write();

    _semaphore.post();

    if (_patternImageCounter < _numPatternImagesRequired)
        return true;    // continue acquiring images
    else{
        // we have all the images required for calibration
        cout << "Images acquired, running calibration now..." << endl;

        // check it 
        //cout << endl;
        /*for (i = 0; i < (_numPatternInnerCornersY * _numPatternInnerCornersX * _numPatternImagesRequired); i++){
            cout << CV_MAT_ELEM( *_objectPoints, float, i, 0) << " " << CV_MAT_ELEM( *_objectPoints, float, i, 1) << " " << CV_MAT_ELEM( *_objectPoints, float, i, 2) << endl;
        }*/
        /*cout << "Object Points: " << endl;
        cout << "width: " << _objectPoints->width << " height: " << _objectPoints->height << endl;
        for (int x = 0; x < _objectPoints->width; x++){
            for (int y = 0; y < _objectPoints->height; y++){
                cout << CV_MAT_ELEM( *_objectPoints, float, y, x) << " ";
            }
            cout << endl;
        }*/
        /*cout << endl;
        cout << "Image Points: " << endl;
        cout << "width: " << _imagePoints->width << " height: " << _imagePoints->height << endl;
        for (i = 0; i < (_numPatternInnerCornersY * _numPatternInnerCornersX * _numPatternImagesRequired); i++){
            cout << CV_MAT_ELEM( *_imagePoints, float, i, 0) << " " << CV_MAT_ELEM( *_imagePoints, float, i, 1) << " " <<  endl;
        }*/
        /*for (int x = 0; x < _imagePoints->width; x++){
            for (int y = 0; y < _imagePoints->height; y++){
                cout << CV_MAT_ELEM( *_imagePoints, float, y, x) << " ";
            }
            cout << endl;
        }*/

        cvCalibrateCamera2( _objectPoints, 
                            _imagePoints,
                            _pointCounts, 
                            cvSize(_ocvImgIn->width, _ocvImgIn->height),
                            _intrinsicMatrix, 
                            _distortionCoeffs,
                            NULL,       // not needed 
                            NULL    // not needed
                            );
        cout << "depois da funcao" << endl;

        float fx = CV_MAT_ELEM( *_intrinsicMatrix, float, 0, 0);
        float fy = CV_MAT_ELEM( *_intrinsicMatrix, float, 1, 1);
        float cx = CV_MAT_ELEM( *_intrinsicMatrix, float, 0, 2);
        float cy = CV_MAT_ELEM( *_intrinsicMatrix, float, 1, 2);
        float k1 = CV_MAT_ELEM( *_distortionCoeffs, float, 0, 0);
        float k2 = CV_MAT_ELEM( *_distortionCoeffs, float, 0, 1);
        float p1 = CV_MAT_ELEM( *_distortionCoeffs, float, 0, 2);
        float p2 = CV_MAT_ELEM( *_distortionCoeffs, float, 0, 3);
            
        // write parameters to stdout
        cout << endl;
        cout << "Calibration Parameters:" << endl;
        cout << "fx " << fx << endl;
        cout << "fy " << fy << endl;
        cout << "cx " << cx << endl;
        cout << "cy " << cy << endl;
        cout << "k1 " << k1 << endl;
        cout << "k2 " << k2 << endl;
        cout << "p1 " << p1 << endl;
        cout << "p2 " << p2 << endl;
        cout << endl;

        // write parameters to file
        if (strcmp(_outputFilename.c_str(), "") != 0){
            cout << "Writing/updating calibration parameters to/in file: " << _outputFilename.c_str() << endl;
            if(writeCalibrationToFile(width, height, fx, fy, cx, cy, k1, k2, p1, p2, _outputFilename, _outputGroupname))
                cout << "Configurtaion file written/modified successfully." << endl;
            else
                cout << "Writing file failed!" << endl;
        }
        cout << endl;
        cout << "Calibration finished. Exiting in " << endl;
        for (i = 20; i > -1; i--){
            yarp::os::Time::delay(1.0);
            cout << i << " ";
        } 
        cout << endl;
        return false;   // closing module
    }
}

bool CamCalibConfModule::respond(const Bottle& command, Bottle& reply) {
    
    if(RFModule::respond(command, reply))
        return true;
    reply.clear();  // reply contains a "command not recognized" from Module::respond()

    switch (command.get(0).asVocab()) {
    case VOCAB1('g'):
        _semaphore.wait();
        _grabFlag = true;
        cout << "Trying to extract pattern corners..." << endl;
        _semaphore.post();
        return true;
    default:
        reply.add("command not recognized");
        return false;
    }
    return false;
}

void CamCalibConfModule::initImages(int width, int height){

    if (_ocvImgIn != NULL)
        cvReleaseImage(&_ocvImgIn);
    _ocvImgIn = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3 );

    if (_ocvImgTmp1 != NULL)
        cvReleaseImage(&_ocvImgTmp1);
    _ocvImgTmp1 = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);

    if (_ocvImgTmp2 != NULL)
        cvReleaseImage(&_ocvImgTmp2);
    _ocvImgTmp2 = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);\

    if (_ocvImgOut != NULL)
        cvReleaseImage(&_ocvImgOut);
    _ocvImgOut = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
  
    // trick the yarp image wrapper
    _ocvImgOut->colorModel[0] = 'R';
    _ocvImgOut->colorModel[1] = 'G';
    _ocvImgOut->colorModel[2] = 'B';
    _ocvImgOut->channelSeq[0] = 'R';
    _ocvImgOut->channelSeq[1] = 'G';
    _ocvImgOut->channelSeq[2] = 'B'; 
}

bool CamCalibConfModule::writeCalibrationToFile( int width, int height,
                                                float fx, float fy,
                                                float cx, float cy,
                                                float k1, float k2,
                                                float p1, float p2,
                                                string filename,
                                                string groupname){

    vector<string> lines;

    bool append = false;

    ifstream in;
    in.open(filename.c_str()); //filename.c_str());
    
    if(in.is_open()){
        // file exists
        string line;
        bool sectionFound = false;
        bool sectionClosed = false;

        // process lines
        while(std::getline(in, line)){
            // check if we left calibration section
            if (sectionFound == true && line.find("[", 0) != string::npos)
                sectionClosed = true;   // also valid if no groupname specified
            // check if we enter calibration section
            if (line.find(string("[") + groupname + string("]"), 0) != string::npos)
                sectionFound = true;
            // if no groupname specified
            if (groupname == "")
                sectionFound = true;
            // if we are in calibration section (or no section/group specified)
            if (sectionFound == true && sectionClosed == false){
                // replace w line
                if (line.find("w",0) != string::npos){
                    stringstream ss;
                    ss << width;
                    line = "w " + string(ss.str());
                }
                // replace h line
                if (line.find("h",0) != string::npos){
                    stringstream ss;
                    ss << height;
                    line = "h " + string(ss.str());
                }
                // replace fx line
                if (line.find("fx",0) != string::npos){
                    stringstream ss;
                    ss << fx;
                    line = "fx " + string(ss.str());
                }
                // replace fy line
                if (line.find("fy",0) != string::npos){
                    stringstream ss;
                    ss << fy;
                    line = "fy " + string(ss.str());
                }
                // replace cx line
                if (line.find("cx",0) != string::npos){
                    stringstream ss;
                    ss << cx;
                    line = "cx " + string(ss.str());
                }
                // replace cy line
                if (line.find("cy",0) != string::npos){
                    stringstream ss;
                    ss << cy;
                    line = "cy " + string(ss.str());
                }
                // replace k1 line
                if (line.find("k1",0) != string::npos){
                    stringstream ss;
                    ss << k1;
                    line = "k1 " + string(ss.str());
                }
                // replace k2 line
                if (line.find("k2",0) != string::npos){
                    stringstream ss;
                    ss << k2;
                    line = "k2 " + string(ss.str());
                }
                // replace p1 line
                if (line.find("p1",0) != string::npos){
                    stringstream ss;
                    ss << p1;
                    line = "p1 " + string(ss.str());
                }
                // replace p2 line
                if (line.find("p2",0) != string::npos){
                    stringstream ss;
                    ss << p2;
                    line = "p2 " + string(ss.str());
                }
       
            }
            // buffer line
            lines.push_back(line);
        }
        
        in.close();

        // rewrite file
        if (!sectionFound){
            append = true;
            cout << "Camera calibration parameter section " + string("[") + groupname + string("]") + " not found in file " << filename << ". Adding group..." << endl;
        }
        else{
            // rewrite file
            ofstream out;
            out.open(filename.c_str(), ios::trunc);
            if (out.is_open()){
                for (int i = 0; i < (int)lines.size(); i++)
                    out << lines[i] << endl;
                out.close();
            }
            else
                return false;
        }
        
    }
    else{
        append = true;
    }

    if (append){
        // file doesn't exist or section is appended 
        ofstream out;
        out.open(filename.c_str(), ios::app);
        if (out.is_open()){
            out << string("[") + groupname + string("]") << endl;
            out << endl;
            out << "w  " << width << endl;
            out << "h  " << height << endl;
            out << "fx " << fx << endl;
            out << "fy " << fy << endl;
            out << "cx " << cx << endl;
            out << "cy " << cy << endl;
            out << "k1 " << k1 << endl;
            out << "k2 " << k2 << endl;
            out << "p1 " << p1 << endl;
            out << "p2 " << p2 << endl;
            out << endl;
            out.close();
        }
        else
            return false;
    }

    return true;
}
