// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>

CamCalibModule::CamCalibModule(){

    _calibTool = NULL;
	_framerate.init(50);
}

CamCalibModule::~CamCalibModule(){

}


bool CamCalibModule::open(Searchable& config){

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
        rf.setDefaultConfigFile("icubCamConf.ini");
	}
    rf.configure("ICUB_ROOT",0,NULL);
	Property prop(rf.toString());
	prop.fromString(config.toString(), false);
	prop.setMonitor(config.getMonitor());

    // pass configuration over to bottle
    Bottle botConfig(prop.toString().c_str());
    botConfig.setMonitor(prop.getMonitor());
    
	// check --group option
    if (prop.check("group")){    
        Bottle &groupSelector=botConfig.addList();
        groupSelector.addString("group");
        groupSelector.addString(config.find("group").asString());
    }    

    // is group option present?
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

	_prtImgIn.open(getName("in"));
	_prtImgOut.open(getName("out"));
    _configPort.open(getName("conf"));
    attach(_configPort, true);
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

    _framerate.addStartTime(yarp::os::Time::now());

    yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
    yrpImgIn = _prtImgIn.read();
    if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;

    _semaphore.wait();
    yarp::sig::ImageOf<PixelRgb>& yrpImgOut = _prtImgOut.prepare();

    _calibTool->apply(*yrpImgIn, yrpImgOut);

    _prtImgOut.write();
    _semaphore.post();

    _framerate.addEndTime(yarp::os::Time::now());
    if(_framerate.hasNewFramerate()){
        cout << _framerate.getFramerate() << " fps" << endl; 
		
    }

	fflush(stdout);
    return true;
}

