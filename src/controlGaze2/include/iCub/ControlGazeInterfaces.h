// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __CONTROLGAZEINTERFACES__
#define __CONTROLGAZEINTERFACES__

#include <string>
#include <yarp/os/Bottle.h>

namespace iCub {
    namespace contrib {
        class IControlGazeControls;
    }
}

/**
 *
 * Interfaces for the ControlGaze Module
 */
class iCub::contrib::IControlGazeControls
{
public:
    virtual ~IControlGazeControls(){}
    virtual bool reset()=0;
	virtual bool help()=0;
    virtual bool saccadeAbsolute(double azimuth, double elevation)=0;
    virtual bool saccadeRelative(double azimuth, double elevation)=0;
    virtual bool saccadeImageRef(double pnx, double pny)=0;
	virtual bool saccadePixelRef(double px, double py)=0;
	virtual bool pursuitAbsolute(double azimuth, double elevation)=0;
    virtual bool pursuitRelative(double azimuth, double elevation)=0;
    virtual bool pursuitImageRef(double pnx, double pny)=0;
	virtual bool pursuitPixelRef(double px, double py)=0;
	virtual bool pursuitInterrupt()=0;
	virtual bool getControllerStatus(string &status)=0;
    virtual bool getSaccadeTime(double &time)=0;
    virtual bool getReference(double &azimuth, double &elevation)=0;
    virtual bool getDirectionEyeRight(double &azimuth, double &elevation)=0;
    virtual bool getDirectionEyeLeft(double &azimuth, double &elevation)=0;
    virtual bool getDirectionHead(double &azimuth, double &elevation)=0;
};

/**
 * Remote Control Vocabulary\n
 * Examples:
 * sac abs azimuth elevation // absolute neck based angles (degrees) \n
 * sac rel azimuth elevation // relative angles (degrees) \n
 * sac img pixNX pixNY // relative normalized pixel values (see: http://eris.liralab.it/wiki/Image_Coordinate_Standard)\n
 * get st // get saccade time\n
 * get stat // get controller status\n
 * get ref // get current reference position (absolute)\n
 * get der // get direction eye right\n
 * get del // get direction eye left\n
 * get dh // get direction head\n
 */
#define CONTROLGAZE_VOCAB_SET VOCAB3('s','e','t')
#define CONTROLGAZE_VOCAB_GET VOCAB3('g','e','t')
#define CONTROLGAZE_VOCAB_IS VOCAB2('i','s')
#define CONTROLGAZE_VOCAB_FAILED VOCAB4('f','a','i','l')
#define CONTROLGAZE_VOCAB_OK VOCAB2('o','k')

#define CONTROLGAZE_VOCAB_HELP VOCAB4('h','e','l','p')
#define CONTROLGAZE_VOCAB_RESET VOCAB4('r','s','e','t')
#define CONTROLGAZE_VOCAB_POS VOCAB3('p','o','s')
#define CONTROLGAZE_VOCAB_SACCADE VOCAB3('s','a','c')
#define CONTROLGAZE_VOCAB_PURSUIT VOCAB3('p','u','r')
#define CONTROLGAZE_VOCAB_INTERRUPT VOCAB3('i','n','t')
#define CONTROLGAZE_VOCAB_COORD_PIX VOCAB3('p','i','x')
#define CONTROLGAZE_VOCAB_COORD_IMG VOCAB3('i','m','g')
#define CONTROLGAZE_VOCAB_COORD_REL VOCAB3('r','e','l')
#define CONTROLGAZE_VOCAB_COORD_ABS VOCAB3('a','b','s')
#define CONTROLGAZE_VOCAB_SACCADE_TIME VOCAB2('s','t')
#define CONTROLGAZE_VOCAB_STATE VOCAB4('s','t','a','t') // controller status
#define CONTROLGAZE_VOCAB_REF VOCAB3('r','e','f')
#define CONTROLGAZE_VOCAB_DIR_EYE_RIGHT VOCAB3('d','e','r')
#define CONTROLGAZE_VOCAB_DIR_EYE_LEFT VOCAB3('d','e','l')
#define CONTROLGAZE_VOCAB_DIR_HEAD VOCAB2('d','h')
#define CONTROLGAZE_VOCAB_MIN_SACCADE_TIME VOCAB3('m','s','t')
#define CONTROLGAZE_VOCAB_LIMIT_RESET_TIME VOCAB3('l','r','t')

#define CONTROLGAZE_VOCAB_VERGENCE_GAIN VOCAB4('v','e','r','g')

#endif


