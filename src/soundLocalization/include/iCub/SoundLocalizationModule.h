// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Hornstein, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SOUNDLOCALIZATIONMODULE__
#define __SOUNDLOCALIZATIONMODULE__

 // std
#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "YARPFft.h"

namespace iCub {
    namespace contrib {
        class SoundLocalizationModule;
    }
}


/**
 *
 * Sound Localization Module class
 *
 * \see icub_soundlocalization
 *
 */
class iCub::contrib::SoundLocalizationModule : public yarp::os::Module {

private:

    // ports
	yarp::os::BufferedPort<yarp::sig::Sound>              _prtSoundIn;
	yarp::os::BufferedPort<yarp::sig::VectorOf<double> >  _prtVctLoc;
	yarp::os::BufferedPort<yarp::sig::VectorOf<double> >  _prtVctLoc2;
    yarp::os::BufferedPort<yarp::os::Bottle> _configPort;

	// Variables
	double _levelthresh;
	double _tiltvarlim;
	int Fs;
	double map[6];
	int numsamples;
	int count, count2;
	double *normalizedDoubleSamplesChan0;
    double *normalizedDoubleSamplesChan1;

	// Methods
	YARPFft * fft;
	void cross_corr(double *left_ear, double *right_ear, double *CrossCorr);
	int maximum(double *CrossCorr);
	bool ILD(double *left_ear, double *right_ear, double &ild);
	bool ITD(double *left_ear, double *right_ear, double &itd, double &var);
	bool notch(double *left_ear, double *right_ear, double &notch, double &var);
	bool ReadSoundFromFile(double *ear, std::string file_name, int nr_of_samples);
	bool SaveSoundToFile(double *ear, char *file_name, int nr_of_samples);

public:

    SoundLocalizationModule();
    virtual ~SoundLocalizationModule();
    
	virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

};


#endif
