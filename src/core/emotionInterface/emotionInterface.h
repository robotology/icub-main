// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __EMOTIONINTERFACE__
#define __EMOTIONINTERFACE__

#include <string>
#include <yarp/os/Vocab.h>

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

constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_SET = yarp::os::createVocab('s','e','t');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_GET = yarp::os::createVocab('g','e','t');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_IS  = yarp::os::createVocab('i','s');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_FAILED        = yarp::os::createVocab('f','a','i','l');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_OK            = yarp::os::createVocab('o','k');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_HELP          = yarp::os::createVocab('h','e','l','p');

constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_MOUTH         = yarp::os::createVocab('m','o','u');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_EYELIDS       = yarp::os::createVocab('e','l','i');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_LEFTEYEBROW   = yarp::os::createVocab('l','e','b');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_RIGHTEYEBROW  = yarp::os::createVocab('r','e','b');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_ALL           = yarp::os::createVocab('a','l','l');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_RAW           = yarp::os::createVocab('r','a','w');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_COLOR         = yarp::os::createVocab('c','o','l');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_BRIG        = yarp::os::createVocab('b','r','i','g');
constexpr  yarp::conf::vocab32_t EMOTION_VOCAB_MASK          = yarp::os::createVocab('m','a','s','k');



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


