// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007-2009 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/AttentionSelectionModule.h>
#include <cv.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

template <class T>
T portable_min(T x, T y) {
    return (x<y)?x:y;
}

template <class T>
T portable_max(T x, T y) {
    return (x>y)?x:y;
}

AttentionSelectionModule::AttentionSelectionModule() :
	_dblTPF(0.05),
	_dblTPFAchieved(0.0),
	_intFPS(20),
	_intFPSAchieved(0),
	_intPrintFPSAfterNumFrames(20),
	_intFC(0),
	_dblStartTime(0),
	_inhibitOutput(false){
    MAX_SALIENCE=255.0f;
	TRACKER_PRIORITY=0.1f;
	_trackerX = 0.0;
	_trackerY = 0.0;
	_trackerSalience = -1.0f;
}

AttentionSelectionModule::~AttentionSelectionModule(){ 
}


bool AttentionSelectionModule::configure(yarp::os::ResourceFinder &rf){

    _hViewAngle = rf.check("horizontalViewAngle",
                                Value(360.0),
                                "Width of input saliency map corresponds to how many degrees? (double)").asDouble();
    _vViewAngle = rf.check("verticalViewAngle",
                                Value(180.0),
                                "Height of input saliency map corresponds to how many degrees? (double)").asDouble();
    _resX = rf.check("resx",
                        Value(320),
                        "EgoSphere resolution (pixels int).").asInt();
    _resY = rf.check("resy",
                        Value(240),
                        "EgoSphere resolution (pixels int).").asInt();
    _thresholdDifference = (float)rf.check("thresholdDifference",
                                Value(20),
                                "|oldMaxSalience - newMaxSalience| > thresholdSaccade in order to switch to a new attention location (double)").asDouble();
    _verbose = (bool)rf.check("verbose",
                                Value(0),
                                "Verbose output (int [0|1]).").asInt();
    _thresholdDistanceFraction = (float)rf.check("thresholdDistanceFraction",
                                    Value(0.1666),
                                    "thresholdDistanceFraction * min(imgWidth, imgHeight) = thresholdDifference (distance needed for a new salient location to be selected) (double).").asDouble();
    _timeDelay = rf.check("timeDelay",
                                Value(2.0),
                                "Time delay before selecting a new most salient location (seconds double).").asDouble();

    _limitAzimuthRightPix = (int)(-rf.check("limitAzimuthRight",
                                        Value(-180.0),
                                        "Limit for egocentric locations selected for saccades (degree double).").asDouble()
                                        / _hViewAngle * (double)_resX + ((double)_resX/2.0));
    _limitAzimuthLeftPix = (int)(-rf.check("limitAzimuthLeft",
                                        Value(180.0),
                                        "Limit for egocentric locations selected for saccades 9degree double).").asDouble()
                                        / _hViewAngle * (double)_resX + ((double)_resX/2.0));
    _limitElevationTopPix = (int)(-rf.check("limitElevationTop",
                                        Value(90.0),
                                        "Limit for egocentric locations selected for saccades 9degree double).").asDouble()
                                        / _vViewAngle * (double)_resY + ((double)_resY/2.0));
    _limitElevationBottomPix = (int)(-rf.check("limitElevationBottom",
                                        Value(-90.0),
                                        "Limit for egocentric locations selected for saccades 9degree double).").asDouble()
                                        / _vViewAngle * (double)_resY + ((double)_resY/2.0));
	_intFPS = rf.check("fps", Value(50), "Try to achieve this number of frames per second (int).").asInt();
	_intPrintFPSAfterNumFrames = rf.check("fpsOutputFrequency", Value(20), "Print the achieved framerate after this number of frames (int).").asInt();
	_dblTPF = 1.0f/((float)_intFPS);
    _inhibitOutput = (bool)rf.check("inhibitOutput", Value(false), "Do not send the selected target position [0/1].").asInt();

    _maxSalienceX = -1;
    _maxSalienceY = -1;
    _maxSalience = 0.0f;
    _saccadeIndex = 0;
    _timeStart = yarp::os::Time::now();
    _gazeX = _resX/2;
    _gazeY = _resY/2;
    _gazeXOld = _resX/2;
    _gazeYOld = _resY/2;
	_gazeAz = 0.0;
	_gazeEl = 0.0;
	_gazeAzOld = 0.0;
	_gazeElOld = 0.0;
	_intGazeState = 5; // magic number for gaze-state 'REST' given by controlGaze2
	_dblGazeAz = 0.0;
	_dblGazeEl = 0.0;

	_w = 320.0;
	_w2 = 160.0;
	_h = 240.0;
	_h2 = 120.0;

    bool ok = true;

	ok &= _remoteEgosphere.open(getName("/remoteEgoSphere"));

    ok &= _prtImgFloatSalienceIn.open(getName("/i:map"));
	ok &= _prtImgFloatSelectionOut.open(getName("/o:map"));
    ok &= _prtVctPosOut.open(getName("/o:position"));
    ok &= _trackersignalOutput_port.open(getName("/o:velocity"));
    ok &= _prtVctTrackerIn.open(getName("/i:tracker"));
	ok &= _prtBotGazeStateIn.open(getName("/i:gaze"));
    ok &= _configPort.open(getName("/conf"));
    attach(_configPort);
	_dblStartTime = yarp::os::Time::now();
    return ok;
}

bool AttentionSelectionModule::close(){
    _prtImgFloatSalienceIn.close();
	_prtImgFloatSelectionOut.close();
    _prtVctPosOut.close();
    _configPort.close();
    _trackersignalOutput_port.close();
    _prtVctTrackerIn.close();
	_prtBotGazeStateIn.close();
	_remoteEgosphere.close();
    return true;
}

bool AttentionSelectionModule::interruptModule(){
    _prtImgFloatSalienceIn.interrupt();
	_prtImgFloatSelectionOut.interrupt();
    _prtVctPosOut.interrupt();
    _configPort.interrupt();
    _trackersignalOutput_port.interrupt();
    _prtVctTrackerIn.interrupt();
	_prtBotGazeStateIn.interrupt();
    return true;
}

bool AttentionSelectionModule::updateModule(){

	// framerate stuff
	_intFC++;
	if(_intPrintFPSAfterNumFrames <= _intFC && _intPrintFPSAfterNumFrames > 0){
		std::cout << "FPS: " << _intFPSAchieved << std::endl;
		_intFC = 0;
	}
	_dblTPFAchieved = ((float)(yarp::os::Time::now() - _dblStartTime));
	if(_dblTPFAchieved < _dblTPF){
		yarp::os::Time::delay(_dblTPF-_dblTPFAchieved);
		_intFPSAchieved = _intFPS;
	}
	else{
		_intFPSAchieved = (int)::floor((1.0 / _dblTPFAchieved) + 0.5);
	}
	_dblStartTime = yarp::os::Time::now();

	// Variables
	yarp::sig::ImageOf<PixelFloat> *yrpImgFloatSalienceIn = NULL;
    Vector *vctTrackerSignal = NULL;
	yarp::os::Bottle* botGazeState = NULL;

	// Read input ports:

    // get the images
    yrpImgFloatSalienceIn = _prtImgFloatSalienceIn.read(false); // non-blocking read
	// get the state of the gaze controller
	botGazeState = _prtBotGazeStateIn.read(false); // non-blocking read (caution: botGazeState might be NULL)
    // get the tracker signal
    vctTrackerSignal = _prtVctTrackerIn.read(false); // non-blocking 
   
    _mutex.wait(); // do not move this line before the read calls (quit not possible if waiting for input)

	// Process input:

	// process the gaze-state:
	if(botGazeState != NULL){ // update gaze-state and gaze position if gaze-state bottle available
		if(botGazeState->size() < 3){
			cout << "Error: Received gaze state bottle has size: " << botGazeState->size() << ". Should have size 3 [state(int), azimuth(double), elevation(double)." << endl;
		}
		else{
			_intGazeState = botGazeState->get(0).asInt();
			_dblGazeAz = botGazeState->get(1).asDouble();
			_dblGazeEl = botGazeState->get(2).asDouble();
		}
	}
	if (_intGazeState == 4){ // gaze pos not reachable (in limit) see enum in control_gaze.h
		if(_verbose){
			cout << endl << "Adding IOR azimuth: " << _gazeAzOld << " elevation: " << _gazeElOld << endl;
		}
		_remoteEgosphere.addIORRegion(_gazeAzOld, _gazeElOld); // does not wait for response (it's save to do this if connection is not established)
	}

	// process saliency input
	if(yrpImgFloatSalienceIn != NULL){
		_w = (double)yrpImgFloatSalienceIn->width();
		_w2 = _w/2.0;
		_h = (double)yrpImgFloatSalienceIn->height();
		_h2 = _h/2.0;
		_thresholdDistance = (float)(_thresholdDistanceFraction*(float)(portable_min(_w,_h)));  
		//cout << "thresholddistance: " << _thresholdDistance << endl;
		// get salience maximum 
		int x,y;
		float maxSalience = 0.0f; 
		this->getPeak(*yrpImgFloatSalienceIn, x, y, maxSalience);
		if(_verbose){
			std::cout << "Got image with peak at: " << x << " by " << y << std::endl;
		}
		// adjust max salience position if:
		// difference in saliency value > threshold1 &&
		// distance from last most salient location > threshold2
		if ( ((float)fabs((float)(_maxSalience - maxSalience)) >= _thresholdDifference) &&
			(pow((double)abs(x - _maxSalienceX),2) + 
			 pow((double)abs(y - _maxSalienceY),2) >= 
			 pow((double)_thresholdDistance,2)) ){
				_maxSalience = maxSalience;
				_maxSalienceX = x;
				_maxSalienceY = y;
		}
	}

	// process tracker input
	if(vctTrackerSignal != NULL){
		if(vctTrackerSignal->size() < 5){
			cout << "AttentionSelectionModule::update() Error! Tracker signal vector size too small." << endl;
		}
		else{
			// valid tracker signal received
			_trackerX = (*vctTrackerSignal)[0];
			_trackerY = (*vctTrackerSignal)[1];
			_trackerSalience = (*vctTrackerSignal)[4];
			//cout << "Tracker signal received... position: " << (*vctTrackerSignal)[0] << " " << (*vctTrackerSignal)[1] << " time: " << (*vctTrackerSignal)[4] << " assigned salience: " << _trackerSalience << endl;
		}
	}

	// Output salience positions
	if (_verbose){
		std::cout << "Salience map peak: " << _maxSalience << " at: " << _maxSalienceX << "x" << (int)_maxSalienceY << "; tracker salience: " << (int)_trackerSalience << " at: " << (int)_trackerX << "x" << (int)_trackerY << std::endl;
	}

	// Use peak of the salience map
	if(_trackerSalience < _maxSalience){
		// write target position for saccade to most salient location
		if (_maxSalience > 0.01f){ // && (_gazeXOld != _gazeX || _gazeYOld != _gazeY)){
			// check for saccade delay time and selection limits
			if ((yarp::os::Time::now() - _timeStart > _timeDelay) &&
				(_maxSalienceX < _limitAzimuthRightPix) && 
				(_maxSalienceX > _limitAzimuthLeftPix) &&
				(_maxSalienceY > _limitElevationTopPix) &&
				(_maxSalienceY < _limitElevationBottomPix) ){
					_gazeX = _maxSalienceX;
					_gazeY = _maxSalienceY;
					_timeStart = yarp::os::Time::now(); // reset the start time
					cout << "reset gaze.. at: " << yarp::os::Time::now() << endl;
			}
			else{
				_gazeX = _gazeXOld;
				_gazeY = _gazeYOld;
			}
			if((_gazeXOld != _gazeX || _gazeYOld != _gazeY)){
				_saccadeIndex++;
			}
			// prepare output vector
			_gazeAz = -((double)_gazeX - _w2)/ _w * _hViewAngle;
			_gazeEl = -((double)_gazeY - _h2)/ _h * _vViewAngle;
			if(!_inhibitOutput){
				VectorOf<double> &vctPos = _prtVctPosOut.prepare();
				vctPos.resize(5);
				vctPos(0) = _gazeAz;
				vctPos(1) = _gazeEl;
				vctPos(2) = (double)(int)'a'; // absolute (neck reference) coordinates are sent
				vctPos(3) = (double)(int)'s'; // receiver module should do saccades
				vctPos(4) = _saccadeIndex;
				// write output vector
				_prtVctPosOut.write();
				// debug output
				if (_verbose){
					cout << "selected pos: ";
					for (int i = 0; i < 2; i++){
						cout << vctPos(i) << " ";
					}
					cout << " mode ";
					for (int i = 2; i < 4; i++){
						cout << (char)vctPos(i) << " ";
					}
					cout << " saccade index: " << vctPos(4);
					cout << "; gaze state: " << _intGazeState << " gaze az: " << _dblGazeAz << " gaze el: "  << _dblGazeEl << endl;
					cout << endl;
				}
			}
		}
	}
	// Use the position reported by the tracker
	else{
		// caution, in this case the _gazeX/Y is not updated..
		VectorOf<double> &normTrackOutput = _trackersignalOutput_port.prepare();
		normTrackOutput.resize(5);
		normTrackOutput[0] = _trackerX;
		normTrackOutput[1] = _trackerY;
		normTrackOutput[2] = 'r';
		normTrackOutput[3] = 'p';
		normTrackOutput[4] = _trackerSalience;
		// write out
		_trackersignalOutput_port.write();
		// debug output
		if (_verbose){
			cout << "tracker pos: ";
			for (int i = 0; i < 2; i++){
				cout << normTrackOutput(i) << " ";
			}
			cout << " mode: ";
			for (int i = 2; i < 4; i++){
				cout << (char)normTrackOutput(i) << " ";
			}
			cout << " time: " << normTrackOutput(4) << " ";
			cout << endl;
		}
	}

	// Write a small visualization map output
	if(yrpImgFloatSalienceIn != NULL){
		int intWOutX = 256;
		int intHOutY = 256;
		yarp::sig::ImageOf<yarp::sig::PixelFloat> &yrpImgOut = _prtImgFloatSelectionOut.prepare();
		yrpImgOut.resize(intWOutX, intHOutY); // does nothing in case size matches
		yrpImgOut.zero();
		yarp::sig::draw::addCrossHair(*yrpImgFloatSalienceIn, yarp::sig::PixelFloat(255.0f), _gazeX, _gazeY, 20);
		//yarp::sig::draw::addRectangle(*yrpImgFloatSalienceIn, yarp::sig::PixelFloat(255.0f), (int)_trackerX, (int)_trackerY, 10,10);
		cvResize(yrpImgFloatSalienceIn->getIplImage(), yrpImgOut.getIplImage());
		_prtImgFloatSelectionOut.write();
	}
	
    // buffer old values
    _gazeXOld = _gazeX;
    _gazeYOld = _gazeY;
	_gazeAzOld = _gazeAz;
	_gazeElOld = _gazeEl;

    _mutex.post();

    return true;
}

// TODO inefficient: use cvMinMaxLoc
void AttentionSelectionModule::getPeak(ImageOf<PixelFloat> &img, int &i, int &j, float &v){
    //v = -FLT_MAX;
    v = 0.0f;
    i = img.width()/2;
    j = img.height()/2;
    IMGFOR(img,x,y) {
        if (v < img(x,y)){
            v = img(x,y);
            i = x;
            j = y;
        }
    }
}


bool AttentionSelectionModule::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    _mutex.wait();
    switch (command.get(0).asVocab()) {
    case ATTENTIONSELECTION_VOCAB_SET:
        rec = true;
        switch(command.get(1).asVocab()) {
        case ATTENTIONSELECTION_VOCAB_THRESHOLD_DIFFERENCE:
            _thresholdDifference = (float)command.get(2).asDouble();                 
            ok = true;
            break;            
        case ATTENTIONSELECTION_VOCAB_THRESHOLD_DISTANCE:
            _thresholdDistanceFraction = (float)command.get(2).asDouble();                 
            ok = true;
            break;            
		case ATTENTIONSELECTION_VOCAB_THRESHOLD_OUTPUT:
			_inhibitOutput = command.get(2).asInt();
			ok = true;
			break;
        default:
            cout << "received an unknown request after a VOCAB_SET" << endl;
            break;
        }        
        break;
    case ATTENTIONSELECTION_VOCAB_GET:
        rec = true;        
        reply.addVocab(ATTENTIONSELECTION_VOCAB_IS);
        reply.add(command.get(1));
        switch(command.get(1).asVocab()) {
        case ATTENTIONSELECTION_VOCAB_THRESHOLD_DIFFERENCE:
            reply.addDouble((double)_thresholdDifference);
            ok = true;
            break;            
        case ATTENTIONSELECTION_VOCAB_THRESHOLD_DISTANCE:
            reply.addDouble((double)_thresholdDistanceFraction);                
            ok = true;
            break;            
		case ATTENTIONSELECTION_VOCAB_THRESHOLD_OUTPUT:
			reply.addInt((int)_inhibitOutput);
			ok = true;
			break;
	    default:
            cout << "received an unknown request after a VOCAB_GET" << endl;
            break;
        }        
        break;

    }
    _mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(ATTENTIONSELECTION_VOCAB_FAILED);
    }
    else
        reply.addVocab(ATTENTIONSELECTION_VOCAB_OK);

    return ok;
} 	
