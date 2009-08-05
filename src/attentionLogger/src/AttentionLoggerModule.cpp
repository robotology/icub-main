// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/AttentionLoggerModule.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace iCub::contrib;

AttentionLoggerModule::AttentionLoggerModule(){

	
}

AttentionLoggerModule::~AttentionLoggerModule(){ 
}


bool AttentionLoggerModule::open(Searchable& config){
 
	// locate configuration file
    ResourceFinder rf;        
	if (config.check("context")){
        rf.setDefaultContext(config.find("context").asString());
	}
	if (config.check("from")){
        rf.setDefaultConfigFile(config.find("from").asString());
	}
	else if (config.check("file")){
        rf.setDefaultConfigFile(config.find("file").asString());
	}
	else{
        rf.setDefaultConfigFile("icubAttentionSelection.ini");
	}
    rf.configure("ICUB_ROOT",0,NULL);
	Property prop(rf.toString());
	prop.fromString(config.toString(), false);
	prop.setMonitor(config.getMonitor());

    string strNameControlboard = prop.check("controlboard",
                                            Value("/controlboard"),
                                            "The port name of the server controlboard to read encoder values from (string).").asString().c_str();
    _logFileName = "unknown";
    _logDirName = prop.check("logDir",
                                Value("logDir"),
                                "Absolute path of the directory to log to (no trailing '/') (string).").asString().c_str();
    _egoImageWidth = prop.check("egoImageWidth",
                                Value(320),
                                "Width of ego-sphere image (int).").asInt();
    _egoImageHeight = prop.check("egoImageHeight",
                                Value(240),
                                "Height of ego-sphere image (int).").asInt();

    // open remote controlboard
    Bottle botControlboard(string(
                           string("(device remote_controlboard) ") +
                           string("(local ") + string(this->getName("controlboard").c_str()) + string(") ") +
                           string("(remote ") + strNameControlboard + string(") ")
                           ).c_str());
    if(!_dd.open(botControlboard)){
        cout << "Could not open remote controlboard (server controlboard port name: " << strNameControlboard << ")" << endl;
        return false;
    }
    if(!_dd.view(_iEnc)){
        cout << "Motor controlboard does not implement IEncoders!" << endl;
        return false;
    }
    _numAxes = 0;
    _iEnc->getAxes(&_numAxes);
    if (_numAxes < 6){
        cout << "Number of motor axes is < 6. Probably the remote controlboard failed to connect to the server controlboard at given port name: " << strNameControlboard << endl;
        return false;
    }
    _encoders = new double[_numAxes];
    _rotW2C = new double[9];
    _rotC2W = new double[9];
    _gaze = new double[3];
    //open ports
    if(!_prtEgoSalience.open(getName("ego/map/i:img")))
        return false;
    if(!_prtEgoRgb.open(getName("ego/rgb/i:img")))
        return false;
    if(!_prtEgoIOR.open(getName("ego/ior/i:img")))
        return false;
    if(!_prtGazeTarget.open(getName("selection/position/i:vct")))
        return false;
    _configPort.open(getName("conf"));
    attach(_configPort, true);
    // open log file
    //time_t topen = time(NULL);
    time_t timeOpen = time(0);
    tm *tmOpen;
    tmOpen = localtime(&timeOpen);
    stringstream timeStream;
    timeStream.str("");
    timeStream << "y" << (tmOpen->tm_year+1900) << "m" << (tmOpen->tm_mon+1) << "d" << tmOpen->tm_mday << "_" << "h" << tmOpen->tm_hour << "m" << tmOpen->tm_min << "s" << tmOpen->tm_sec;
    string timeString = timeStream.str();
	_logFileName = string(_logDirName + string("/") + timeString + string("_") + _logFileName).c_str();
    _logFile.open(_logFileName.c_str(), ios::app);
    if (!_logFile.is_open()){
        cout << "Error opening log file: " << string(_logDirName + string("/") + _logFileName) << endl;
        return false;
    }
    // write title line
    _logFile << "saccade" << "\t" << 
                "time" << "\t" <<
                "tarPosDegX" << "\t" <<
                "tarPosDegY" << "\t" <<
                "tarPosPixX" << "\t" <<
                "tarPosPixY" << "\t" <<
                "enc0" << "\t" <<
                "enc1" << "\t" <<
                "enc2" << "\t" <<
                "enc3" << "\t" <<
                "enc4" << "\t" <<
                "enc5" << "\t" <<
                "curPosDegX" << "\t" <<
                "curPosDegY" << "\t" <<
                "curPosPixX" << "\t" <<
                "curPosPixY" << "\t" <<
                endl;
    yarp::os::Time::turboBoost();
    _timeStart = -1.0;
    _targetIndex = -1;
    _targetPosDegX = -1.0;
    _targetPosDegY = -1.0;
    _targetPosDegXOld = -1.0;
    _targetPosDegYOld = -1.0;
    _targetPosPixX = -1;
    _targetPosPixY = -1;
    _targetPosPixXOld = -1;
    _targetPosPixYOld = -1;
    _flagNewTarget = false;
    return true;
}

bool AttentionLoggerModule::close(){
    _logFile.close();
    _prtEgoSalience.close();
    _prtEgoRgb.close();
    _prtEgoIOR.close();
    _prtGazeTarget.close();
    _dd.close();
    if (_encoders != NULL)
        delete [] _encoders;
    _encoders = NULL;
    if (_rotW2C != NULL)
        delete [] _rotW2C;
    _rotW2C = NULL;
    if (_rotC2W != NULL)
        delete [] _rotC2W;
    _rotC2W = NULL;
    if (_gaze != NULL)
        delete [] _gaze;
    _gaze = NULL;
    return true;
}

bool AttentionLoggerModule::interruptModule(){
     _prtEgoSalience.interrupt();
    _prtEgoRgb.interrupt();
    _prtEgoIOR.interrupt();
    _prtGazeTarget.interrupt();
    return true;
}

bool AttentionLoggerModule::updateModule(){

    // time things
    if (_timeStart == -1.0)
        _timeStart = Time::now();
    double time = Time::now() - _timeStart;
    _sstream.clear();
    _sstream << time;
    string strTime(_sstream.str());

    // current gaze direction from encoder values (blocking)
    _iEnc->getEncoders(_encoders); // read encoder values
    _eyeMatrix = _headKin.fkine(_encoders, 'r'); // 'r' -> use right eye kinematics
    _headKin.Gaze(&_azimuth, &_elevation, _encoders, 'r');
    /*convertRobMatrix(_eyeMatrix, _rotW2C); 
    transpose(_rotW2C, _rotC2W);
    calcGazeVector(_rotC2W, _gaze);
    calcAngles(_gaze, _azimuth, _elevation);*/
    calcGazePixel(_azimuth, _elevation, _gazeX, _gazeY);
    
    // target location (from attentionSelection) (non blocking)
    VectorOf<double> *vctGazeTarget = _prtGazeTarget.read(false);
    if (vctGazeTarget == NULL){
        // no new target available, keep the old values
        _targetPosDegX = _targetPosDegXOld;
        _targetPosDegY = _targetPosDegYOld;
        _targetPosPixX = _targetPosPixXOld;
        _targetPosPixY = _targetPosPixYOld;
    }
    else{
        _targetPosDegX = (*vctGazeTarget)[0];
        _targetPosDegY = (*vctGazeTarget)[1];
        // target gaze position in pixels
        calcGazePixel(_targetPosDegX, _targetPosDegY, 
                      _targetPosPixX, _targetPosPixY);
		// a new target is available
		if (_targetPosDegX != _targetPosDegXOld || 
			_targetPosDegY != _targetPosDegYOld){
			_targetIndex++;
			_flagNewTarget = true;
		}
    }
    _targetPosDegXOld = _targetPosDegX;
    _targetPosDegYOld = _targetPosDegY;
    _targetPosPixXOld = _targetPosPixX;
    _targetPosPixYOld = _targetPosPixY;
    
    // write values to logFile
    _logFile << _targetIndex << "\t" << 
                time << "\t" <<
                _targetPosDegX << "\t" <<
                _targetPosDegY << "\t" <<
                _targetPosPixX << "\t" <<
                _targetPosPixY << "\t" <<
                _encoders[0] << "\t" <<
                _encoders[1] << "\t" <<
                _encoders[2] << "\t" <<
                _encoders[3] << "\t" <<
                _encoders[4] << "\t" <<
                _encoders[5] << "\t" <<
                _azimuth << "\t" <<
                _elevation << "\t" <<
                _gazeX << "\t" <<
                _gazeY << "\t" <<
                endl;

    // if new target read/save images
    if (_flagNewTarget){
        string filename;
        // read images (blocking)
		std::cout << "Writing images for target index: " << _targetIndex << std::endl;
        ImageOf<PixelFloat> *imgEgoSalienceFloat = _prtEgoSalience.read();
        ImageOf<PixelRgb> *imgEgoRgb = _prtEgoRgb.read();
        ImageOf<PixelRgb> *imgEgoIOR = _prtEgoIOR.read();
        if (imgEgoSalienceFloat == NULL || imgEgoRgb == NULL || imgEgoIOR == NULL){
            cout << "At least one image port returned a NULL value!" << endl;
            return true;
        }
        // convert to rgb needed for saving
        _imgEgoSalienceRgb.resize(_egoImageWidth, _egoImageHeight);
        colorRgbFromFloat((IplImage*)imgEgoSalienceFloat->getIplImage(), (IplImage*)_imgEgoSalienceRgb.getIplImage(), 1.0f);
        // save image egoSalience 
        _sstream.str("");
        _sstream << _logDirName << "/" << _targetIndex << "_" << "_egoSalience.ppm";
        filename = _sstream.str();
        yarp::sig::file::write(_imgEgoSalienceRgb, filename.c_str());
        // save image egoRgb
        _sstream.str("");
        _sstream << _logDirName << "/" << _targetIndex << "_" << "_egoRgb.ppm";
        filename = _sstream.str();
        yarp::sig::file::write(*imgEgoRgb, filename.c_str());
        // save image egoIOR
        _sstream.str("");
        _sstream << _logDirName << "/" << _targetIndex << "_" << "_egoIOR.ppm";
        filename = _sstream.str();
        yarp::sig::file::write(*imgEgoIOR, filename.c_str());
    }
    
    _flagNewTarget = false;
    return true;
}

// helper function to pass correct matrix to projectors
void AttentionLoggerModule::convertRobMatrix(RobMatrix &robMatrix, double *matrix){
    double tmp[9];
    for (int b = 0; b < 3; b++)
        for (int a =0; a < 3; a++)
            tmp[b*3 + a] = robMatrix.M[b][a];
    // convert to 'intuitive' reference frame (axis switching)
    matrix[0] = -tmp[4];
    matrix[1] = -tmp[7];
    matrix[2] = tmp[1];
    matrix[3] = -tmp[5];
    matrix[4] = -tmp[8];
    matrix[5] = tmp[2];
    matrix[6] = -tmp[3];
    matrix[7] = -tmp[6];
    matrix[8] = tmp[0];
}

void AttentionLoggerModule::calcGazeVector(double *rotCamera2World, double *gaze){
    double vct[3]; 
    vct[0] = 0.0; vct[1] = 0.0; vct[2] = 1.0; // 1 0 0
    gaze[0] = rotCamera2World[0] * vct[0] + rotCamera2World[1] * vct[1] + rotCamera2World[2] * vct[2];
    gaze[1] = rotCamera2World[3] * vct[0] + rotCamera2World[4] * vct[1] + rotCamera2World[5] * vct[2];
    gaze[2] = rotCamera2World[6] * vct[0] + rotCamera2World[7] * vct[1] + rotCamera2World[8] * vct[2];  
}

void AttentionLoggerModule::calcAngles(double *gaze, double &azimuth, double &elevation){
    azimuth = -atan(gaze[1]/gaze[0]) * 180 / M_PI;
    elevation = -atan(gaze[2]/sqrt(pow(gaze[0],2) + pow(gaze[1],2))) * 180 / M_PI;
    //azimuth = -atan2(gaze[0],gaze[1]) * 180 / M_PI;
    //elevation = -atan2(sqrt(pow(gaze[0],2) + pow(gaze[1],2)),gaze[2]) * 180 / M_PI;
}

void AttentionLoggerModule::calcGazePixel(double &azimuth, double &elevation, int &gazeX, int &gazeY){
    gazeX = (int)( ((double)(_egoImageWidth))/360.0 * azimuth + ((double)(_egoImageWidth))/2.0);
    gazeY = (int)( ((double)(_egoImageHeight))/180.0 * elevation + ((double)(_egoImageHeight))/2.0);
    if (_gazeX < 0){
        cout << "AttentionLoggerModule::calcGazePixel(): " << _gazeX << "!" << endl;
        _gazeX = 0;
    }
    if (_gazeY < 0){
        cout << "AttentionLoggerModule::calcGazePixel(): " << _gazeY << "!" << endl;
        _gazeY = 0;
    }
    if (_gazeX >= _egoImageWidth){
        cout << "AttentionLoggerModule::calcGazePixel(): " << _gazeX << "!" << endl;
        _gazeX = _egoImageWidth - 1;
    }
    if (_gazeY >= _egoImageHeight){
        cout << "AttentionLoggerModule::calcGazePixel(): " << _gazeY << "!" << endl;
        _gazeY = _egoImageHeight - 1;
    }
}


void AttentionLoggerModule::colorRgbFromFloat(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor){
    
    float flValue = 0.0f;
    int   arraypos = 0;
 
    for (int y = 0; y < imgFloat->height; y++){
    	arraypos = imgRgb->widthStep*y;
        for (int x = 0; x < imgFloat->width; x++){
            //cout << ((float*)(imgFloat->imageData + imgFloat->widthStep*y))[x] << " ";
            flValue = scaleFactor * ((float*)(imgFloat->imageData + imgFloat->widthStep*y))[x];
            if (flValue < 0.0f){
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+0] = (uchar)0;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+1] = (uchar)0;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+2] = (uchar)0;
            }
            else if (flValue >= 0.0f){
                if (flValue > 255.0f)
                    flValue = 255.0f;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+0] = (uchar)flValue;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+1] = (uchar)flValue;
                ((uchar*)(imgRgb->imageData + arraypos))[x*3+2] = (uchar)flValue;
            }
        }
    }                
}
