// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <SoundLocalizationDummyModule.h>

SoundLocalizationDummyModule::SoundLocalizationDummyModule(){
	_azimuth = 0;
    _elevation = 0;
    _sigmaAz = 0; 
    _sigmaEl = 0;
    _intensity = 0;
	_isDataSet = false;
}

SoundLocalizationDummyModule::~SoundLocalizationDummyModule(){ 
}


bool SoundLocalizationDummyModule::open(Searchable& config){
   
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini\n");
        return false;
    }

    yarp::os::Random::seed((int)yarp::os::Time::now());

    bool ok = true;
    ok &= _prtVctLoc.open(getName("o:loc"));
    _configPort.open(getName("conf"));
    attach(_configPort, true);
    return ok;
}

bool SoundLocalizationDummyModule::close(){
    _prtVctLoc.close();
    _configPort.close();
    return true;
}

bool SoundLocalizationDummyModule::interruptModule(){
    _prtVctLoc.interrupt();
    _configPort.interrupt();
    return true;
}

bool SoundLocalizationDummyModule::updateModule(){

	// from addSoundSource
	_semaphore.wait();
	if(_isDataSet){
		VectorOf<double> &vctOut = _prtVctLoc.prepare();
            vctOut.resize(5);
            vctOut[0] = _azimuth;
            vctOut[1] = _sigmaAz;
            vctOut[2] = _elevation;
            vctOut[3] = _sigmaEl;
            vctOut[4] = _intensity;
		std::cout << "Adding sound source at: " << _azimuth << " " << _elevation << 
std::endl;
            _prtVctLoc.write();
            cout << "azimuth: " << vctOut[0] << " " 
                << "sigmaAz: " << vctOut[1] << " " 
                << "elevation: " << vctOut[2] << " " 
                << "sigmaEl: " << vctOut[3] << " " 
                << "intensity: " << vctOut[4] << endl;
		_isDataSet = false;
	}
	_semaphore.post();
	Time::delay(0.1);


    // pattern
    /*for (int az = -180; az <= 180; az += 45){
        for(int el = -90; el <= 90; el +=45){
            azimuth = az;
            elevation = el;
            sigmaAz = 25;
            sigmaEl = 25;
            intensity = 255.0;
            VectorOf<double> &vctOut = _prtVctLoc.prepare();
            vctOut.resize(5);
            vctOut[0] = azimuth;
            vctOut[1] = sigmaAz;
            vctOut[2] = elevation;
            vctOut[3] = sigmaEl;
            vctOut[4] = intensity;
            _prtVctLoc.write();
            cout << "azimuth: " << vctOut[0] << " " 
                << "sigmaAz: " << vctOut[1] << " " 
                << "elevation: " << vctOut[2] << " " 
                << "sigmaEl: " << vctOut[3] << " " 
                << "intensity: " << vctOut[4] << endl;
            Time::delay(0.5);
        }
    }*/
    
    // random
    /*
    VectorOf<double> &vctOut = _prtVctLoc.prepare();
    vctOut.resize(5);
    double azimuth = yarp::os::Random::uniform(0.0,360.0);
    double elevation = yarp::os::Random::uniform(0.0, 180.0);
    double sigmaAz = yarp::os::Random::uniform(5.0, 20.0);
    double sigmaEl = yarp::os::Random::uniform(5.0, 20.0);
    double intensity = 255.0;
    vctOut[0] = azimuth;
    vctOut[1] = sigmaAz;
    vctOut[2] = elevation;
    vctOut[3] = sigmaEl;
    vctOut[4] = intensity;
    _prtVctLoc.write();
    cout << "azimuth: " << vctOut[0] << " " 
        << "sigmaAz: " << vctOut[1] << " " 
        << "elevation: " << vctOut[2] << " " 
        << "sigmaEl: " << vctOut[3] << " " 
        << "intensity: " << vctOut[4] << endl;
    Time::delay(2.0);*/
    return true;
}

bool SoundLocalizationDummyModule::addSoundSource(double azimuth, double azimuthSigma, double elevation, double elevationSigma, double intensity){
	_azimuth = azimuth;
    _elevation = elevation;
    _sigmaAz = azimuthSigma; 
    _sigmaEl = elevationSigma;
    _intensity = intensity;
	_isDataSet = true;
	return true;
}

bool SoundLocalizationDummyModule::respond(const Bottle &command,Bottle &reply){

	bool ok = false;
    bool rec = false; // is the command recognized?

    _semaphore.wait();
    switch (command.get(0).asVocab()) {
	case SOUND_VOCAB_HELP:
		rec = true;
		{
			reply.addString("help");
			reply.addString("\n");
			reply.addString("add <d> <d> <d> <d> \t: add an artificial sound source <azimuth> double[-180.0-180.0] <azimuth_sigma> double[>0.0]  <elevation> double[-90.0-90.0] <elevation_sigma> double[>0.0].\n");
			reply.addString("add <d> <d> \t: add an artificial sound source <azimuth> double[-180.0-180.0] <elevation> double[-90.0-90.0].\n");
			ok = true;
		}
		break;
	case SOUND_VOCAB_ADD:
		rec = true;
		{
			if(command.size() >= 5){
				double az = command.get(1).asDouble();
				double azSig = command.get(2).asDouble();
				double el = command.get(3).asDouble();
				double elSig = command.get(4).asDouble();
				this->addSoundSource(az, azSig, el, elSig, 255.0);
				ok = true;
			}
			else if(command.size() == 3){
				double az = command.get(1).asDouble();
				double el = command.get(2).asDouble();
				this->addSoundSource(az, 15.0, el, 15.0, 255.0);
				ok = true;
			}
		}
		break;
	}
    _semaphore.post();
	if (!rec){
        ok = Module::respond(command,reply);
	}

    if (!ok) {
        reply.clear();
        reply.addVocab(VOCAB_FAILED);
    }
	else{
        reply.addVocab(VOCAB_OK);
	}

    return ok;	
}
