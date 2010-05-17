// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>

CamCalibModule::CamCalibModule(){

    _calibTool = NULL;	
}

CamCalibModule::~CamCalibModule(){

}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf){

	ConstString str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
	setName(str.c_str()); // modulePortName  
	attachTerminal();

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());
    botConfig.setMonitor(rf.getMonitor());		
    // Load from configuration group ([<group_name>]), if group option present
    Value *valGroup; // check assigns pointer to reference
    if(botConfig.check("group", valGroup, "Configuration group to load module options from (string).")){
        string strGroup = valGroup->asString().c_str();        
        // is group a valid bottle?
        if (botConfig.check(strGroup.c_str())){            
            Bottle &group=botConfig.findGroup(strGroup.c_str(),string("Loading configuration from group " + strGroup).c_str());
            botConfig.fromString(group.toString());
        }
        else{
            cout << endl << "Group " << strGroup << " not found." << endl;
            return false;
        }
    }

	// framerate
	_intFPS = botConfig.check("fps", Value(20), "Try to achieve this number of frames per second (int).").asInt();
	_intPrintFPSAfterNumFrames = botConfig.check("fpsOutputFrequency", Value(20), "Print the achieved framerate after this number of frames (int).").asInt();
	_dblTPF = 1.0f/((float)_intFPS);
	_intFPSAchieved = 0;
	_intFC = 0;
	_dblTPFAchieved = 0.0;
	_dblStartTime = 0.0;

    string calibToolName = botConfig.check("projection",
                                         Value("pinhole"),
                                         "Projection/mapping applied to calibrated image [projection|spherical] (string).").asString().c_str();

    _calibTool = CalibToolFactories::getPool().get(calibToolName.c_str());
    if (_calibTool!=NULL) {
        bool ok = _calibTool->open(botConfig);
        if (!ok) {
            delete _calibTool;
            _calibTool = NULL;
            return false;
        }
    }

	_prtImgIn.open(getName("/in"));
	_prtImgOut.open(getName("/out"));
    _configPort.open(getName("/conf"));
    attach(_configPort);
	fflush(stdout);
    return true;
}

bool CamCalibModule::close(){
    _prtImgIn.close();
	_prtImgOut.close();
    _configPort.close();
    if (_calibTool != NULL){
        _calibTool->close();
        delete _calibTool;
        _calibTool = NULL;
    }
    return true;
}

bool CamCalibModule::interruptModule(){
    _prtImgIn.interrupt();
	_prtImgOut.interrupt();
    _configPort.interrupt();
    return true;
}

bool CamCalibModule::updateModule(){	

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

	// calibration
    yarp::sig::ImageOf<PixelRgb> *yrpImgIn = _prtImgIn.read(false);
	if (yrpImgIn != NULL) {        
		_semaphore.wait();
		yarp::sig::ImageOf<PixelRgb>& yrpImgOut = _prtImgOut.prepare();
		_calibTool->apply(*yrpImgIn, yrpImgOut);
		_prtImgOut.write();
		_semaphore.post();  
	}	
    return true;
}

double CamCalibModule::getPeriod() {
  return 0.0;
}

