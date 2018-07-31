// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __EMOTIONINTERFACE__
#define __EMOTIONINTERFACE__

#include <string>
#include <yarp/os/Bottle.h>

namespace iCub {
    namespace contrib {
        class IEmotionInterface;
    }
}

/**
 *
 * Interface for the Facial Expressions
 */
class iCub::contrib::IEmotionInterface
{
public:
    
    virtual ~IEmotionInterface(){}
   
    virtual bool setEyelids(const std::string s)=0;    
    virtual bool setMouth(const std::string s)=0;
    virtual bool setLeftEyebrow(const std::string s)=0;
    virtual bool setRightEyebrow(const std::string s)=0;
    virtual bool setAll(const std::string s)=0;
    virtual bool setRaw(const std::string s)=0;

};

#define EMOTION_VOCAB_SET           yarp::os::createVocab('s','e','t')
#define EMOTION_VOCAB_GET           yarp::os::createVocab('g','e','t')
#define EMOTION_VOCAB_IS            yarp::os::createVocab('i','s')
#define EMOTION_VOCAB_FAILED        yarp::os::createVocab('f','a','i','l')
#define EMOTION_VOCAB_OK            yarp::os::createVocab('o','k')

#define EMOTION_VOCAB_MOUTH         yarp::os::createVocab('m','o','u')
#define EMOTION_VOCAB_EYELIDS       yarp::os::createVocab('e','l','i') 
#define EMOTION_VOCAB_LEFTEYEBROW   yarp::os::createVocab('l','e','b')
#define EMOTION_VOCAB_RIGHTEYEBROW  yarp::os::createVocab('r','e','b')
#define EMOTION_VOCAB_ALL           yarp::os::createVocab('a','l','l')
#define EMOTION_VOCAB_RAW           yarp::os::createVocab('r','a','w')


inline bool EMOTION_CHECK_FAIL(bool ok, yarp::os::Bottle& response) {
    if (ok) {
        if (response.get(0).isVocab() && response.get(0).asVocab() == EMOTION_VOCAB_FAILED) {
            return false;
        }
    }
    else
        return false;

    return true;
}

#endif


