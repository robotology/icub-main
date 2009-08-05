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
   
    virtual bool setEyelids(const ConstString s)=0;    
    virtual bool setMouth(const ConstString s)=0;
    virtual bool setLeftEyebrow(const ConstString s)=0;
    virtual bool setRightEyebrow(const ConstString s)=0;
    virtual bool setAll(const ConstString s)=0;
    virtual bool setRaw(const ConstString s)=0;

};

#define EMOTION_VOCAB_SET VOCAB3('s','e','t')
#define EMOTION_VOCAB_GET VOCAB3('g','e','t')
#define EMOTION_VOCAB_IS VOCAB2('i','s')
#define EMOTION_VOCAB_FAILED VOCAB4('f','a','i','l')
#define EMOTION_VOCAB_OK VOCAB2('o','k')

#define EMOTION_VOCAB_MOUTH VOCAB3('m','o','u')
#define EMOTION_VOCAB_EYELIDS VOCAB3('e','l','i') 
#define EMOTION_VOCAB_LEFTEYEBROW VOCAB3('l','e','b')
#define EMOTION_VOCAB_RIGHTEYEBROW VOCAB3('r','e','b')
#define EMOTION_VOCAB_ALL VOCAB3('a','l','l')
#define EMOTION_VOCAB_RAW VOCAB3('r','a','w')


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


