// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Modified 2008 Dario Figueira
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#ifndef __EGOSPHEREINTERFACES__
#define __EGOSPHEREINTERFACES__

#include <yarp/os/Bottle.h>

namespace iCub {
    namespace contrib {
        class IEgoSphereControls;
    }
}

/**
 *
 * Interface for the EgoSphere
 */
class iCub::contrib::IEgoSphereControls
{
public:
    virtual ~IEgoSphereControls(){}
    virtual bool setSaccadicSuppression(bool on)=0;
    virtual bool getSaccadicSuppression()=0;
    virtual bool setSalienceDecay(double rate)=0;
    virtual double getSalienceDecay()=0;
	virtual bool addIORRegion(double azimuth, double elevation)=0;
    virtual bool reset()=0;
};

#define EGOSPHERE_VOCAB_HELP VOCAB4 ('h','e','l','p')
#define EGOSPHERE_VOCAB_SET VOCAB3('s','e','t')
#define EGOSPHERE_VOCAB_GET VOCAB3('g','e','t')
#define EGOSPHERE_VOCAB_IS VOCAB2('i','s')
#define EGOSPHERE_VOCAB_FAILED VOCAB4('f','a','i','l')
#define EGOSPHERE_VOCAB_OK VOCAB2('o','k')

#define EGOSPHERE_VOCAB_RESET VOCAB4('r','s','e','t')
#define EGOSPHERE_VOCAB_SACCADICSUPPRESSION VOCAB2('o','p') 
#define EGOSPHERE_VOCAB_SALIENCE_DECAY VOCAB2('s','d')
#define EGOSPHERE_VOCAB_THRESHOLD_SALIENCE VOCAB2('t','s')
#define EGOSPHERE_VOCAB_MODALITY_AUDITORY VOCAB2('m','a')
#define EGOSPHERE_VOCAB_MODALITY_VISION VOCAB2('m','v')
#define EGOSPHERE_VOCAB_MODALITY_OBJECTS VOCAB2('m','o')
#define EGOSPHERE_VOCAB_IOR VOCAB3('i','o','r')
#define EGOSPHERE_VOCAB_ADDIORREGION VOCAB3('a','i','r')

inline bool EGOSPHERE_CHECK_FAIL(bool ok, yarp::os::Bottle& response) {
    if (ok) {
        if (response.get(0).isVocab() && response.get(0).asVocab() == EGOSPHERE_VOCAB_FAILED) {
            return false;
        }
    }
    else
        return false;

    return true;
}

#endif


