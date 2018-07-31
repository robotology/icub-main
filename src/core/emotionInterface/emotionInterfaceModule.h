// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#ifndef __EMOTIONINTERFACEMODULE__
#define __EMOTIONINTERFACEMODULE__


 // std
#include <cstdio>
#include <string>
#include <iostream>


// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#include "emotionInterface.h"

namespace iCub {
    namespace contrib {
        class EmotionInitReport;
        class EmotionInterfaceModule;
    }
}

using namespace iCub::contrib;

typedef struct
{  
    char name[3];
    char leb[2];
    char reb[2];
    char mou[2];
    char eli[2];

} EM_CODE;


class iCub::contrib::EmotionInitReport : public PortReport
{
private:
    EmotionInterfaceModule *emo;

public:
    EmotionInitReport(EmotionInterfaceModule *_emo) : emo(_emo) { }
    virtual void report(const PortInfo &info);
};

/**
 *
 * EmotionInterface Module class
 *
 * 
 *
 */
class iCub::contrib::EmotionInterfaceModule : public RFModule,
                                       public IEmotionInterface {

private:

    // input command port
    Port _inputPort;

    // reporter put on the init emotion
    EmotionInitReport emotionInitReport;

    // output command port         
    BufferedPort<Bottle> _outputPort;

    // number of high level emotions
    int _highlevelemotions;

    // table with the setting for each emotion - from config file
    EM_CODE* _emotion_table;

    //Mutual exclusion for the respond method
    Semaphore _semaphore;

    //Time variables for automatic expression switching
    bool   _auto;
    double _lasttime;
    double _period;
    int    _initEmotionTrigger;

    int getIndex(const std::string cmd);
    bool writePort(const char* cmd);

public:

    EmotionInterfaceModule();
    virtual ~EmotionInterfaceModule();
    
    virtual bool configure(ResourceFinder& config);//virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual double getPeriod();
    virtual bool respond(const Bottle &command,Bottle &reply);
    void initEmotion();

    // interface functions
    virtual bool setLeftEyebrow(const std::string cmd);
    virtual bool setRightEyebrow(const std::string cmd);
    virtual bool setEyelids(const std::string cmd);
    virtual bool setMouth(const std::string cmd);
    virtual bool setAll(const std::string cmd);
    virtual bool setRaw(const std::string cmd);
};


#endif
