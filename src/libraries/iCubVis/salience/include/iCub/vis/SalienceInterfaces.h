// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SALIENCEINTERFACES__
#define __SALIENCEINTERFACES__

#include <string>
#include <yarp/os/Bottle.h>

namespace iCub {
    namespace vis {
        class ISalienceControls;
        class IDirectionalSalienceControls;
        class ISalienceModuleControls;
    }
}

/**
 *
 * Interface for a saliency filter.
 */
class iCub::vis::ISalienceControls
{
public:
    virtual ~ISalienceControls(){}
    //virtual yarp::os::Bottle getConfiguration()=0;
    //virtual void setConfiguration(yarp::os::Bottle &config)=0;
    virtual bool setFilterName(std::string n)=0;
    virtual bool setChildFilterName(int j, std::string n)=0;
    virtual std::string getFilterName()=0;
    virtual std::string getChildFilterName(int j)=0;
    virtual bool setWeight(double w)=0;
    virtual double getWeight()=0;
    virtual int getChildCount()=0;
    // methods below should be removed at some point.. 
    //(in favour of access in a more hierachical way)
    virtual bool setChildWeights(yarp::os::Bottle& subWeights)=0;
    virtual bool getChildWeights(yarp::os::Bottle *subWeights)=0;
    virtual bool setChildWeight(int j, double w)=0;
    virtual double getChildWeight(int j)=0;
   
};

/**
 *
 * Interface for the directional salience filter.
 */
class iCub::vis::IDirectionalSalienceControls
{
public:
    virtual ~IDirectionalSalienceControls(){}
    virtual int getNumberScales()=0;
    virtual int getNumberDirections()=0;
    //virtual int getDebugFilterScaleIndex()=0;
    virtual bool setDebugFilterScaleIndex(int size)=0;
    //virtual int getDebugFilterDirectionIndex()=0;
    virtual bool setDebugFilterDirectionIndex(int direction)=0;
    virtual yarp::os::Bottle getDebugImageArrayNames()=0;
    //virtual string getDebugImageArrayName()=0;
    virtual bool setDebugImageArrayName(std::string imgArray)=0;
};

/**
 *
 * Interface for the saliency module.
 */
class iCub::vis::ISalienceModuleControls
{
public:
    virtual ~ISalienceModuleControls(){}
    virtual double getSalienceThreshold()=0;
    virtual bool setSalienceThreshold(double thr)=0;
    virtual int getNumBlurPasses()=0;
    virtual bool setNumBlurPasses(int num)=0;
};

// calls are fn <filtername> set/get property <value>
// e.g.: "fn topFilter.childFilterXY set w 0.5"

// general saliency filter vocab's
#define SALIENCE_VOCAB_HELP VOCAB4('h','e','l','p')
#define SALIENCE_VOCAB_SET VOCAB3('s','e','t')
#define SALIENCE_VOCAB_GET VOCAB3('g','e','t')
#define SALIENCE_VOCAB_IS VOCAB2('i','s')
#define SALIENCE_VOCAB_FAILED VOCAB4('f','a','i','l')
#define SALIENCE_VOCAB_OK VOCAB2('o','k')
#define SALIENCE_VOCAB_CHILD_COUNT VOCAB2('c','c')
#define SALIENCE_VOCAB_WEIGHT VOCAB1('w')
#define SALIENCE_VOCAB_CHILD_WEIGHT VOCAB2('c','w')
#define SALIENCE_VOCAB_CHILD_WEIGHTS VOCAB3('c','w','s')
#define SALIENCE_VOCAB_NAME VOCAB2('f','n')
#define SALIENCE_VOCAB_CHILD_NAME VOCAB2('c','n')
#define SALIENCE_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define SALIENCE_VOCAB_NUM_BLUR_PASSES VOCAB2('n','b')
// directional saliency filter vocab's
#define SALIENCE_VOCAB_DIRECTIONAL_NUM_DIRECTIONS VOCAB3('d','n','d')
#define SALIENCE_VOCAB_DIRECTIONAL_NUM_SCALES VOCAB3('d','n','s')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX VOCAB3('d','s','i')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX VOCAB3('d','d','i')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES VOCAB4('d','a','n','s')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME VOCAB3('d','a','n')

inline bool SALIENCE_CHECK_FAILED(bool ok, yarp::os::Bottle& response) {
    if (ok) {
        if (response.get(0).isVocab() && response.get(0).asVocab() == SALIENCE_VOCAB_FAILED) {
            return false;
        }
    }
    else
        return false;

    return true;
}

#endif


