// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Hornstein, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SOUNDLOCALIZATIONDUMMYMODULE__
#define __SOUNDLOCALIZATIONDUMMYMODULE__

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class SoundLocalizationDummyModule;
    }
}

using namespace iCub::contrib;


/**
 *
 * Sound Localization Module class
 *
 * \see icub_soundlocalization
 *
 */
class iCub::contrib::SoundLocalizationDummyModule : public Module {

private:

    // ports
    BufferedPort<VectorOf<double> >  _prtVctLoc;
    BufferedPort<Bottle> _configPort;

	virtual bool respond(const Bottle &command,Bottle &reply);

	yarp::os::Semaphore _semaphore;

	bool _isDataSet;
	double _azimuth;
    double _elevation; 
    double _sigmaAz; 
    double _sigmaEl; 
    double _intensity; 

public:

    SoundLocalizationDummyModule();
    virtual ~SoundLocalizationDummyModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

	virtual bool addSoundSource(double azimuth, double azimuthSigma, double elevation, double elevationSigma, double intensity);

};


#define VOCAB_FAILED VOCAB4('f','a','i','l')
#define VOCAB_OK VOCAB2('o','k')

#define SOUND_VOCAB_HELP VOCAB4 ('h','e','l','p')
#define SOUND_VOCAB_ADD VOCAB3 ('a','d','d')

#endif

