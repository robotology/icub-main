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
    size_t _highlevelemotions{};
    size_t _numberOfColors{};
    size_t _eyebrowmaskemotions{};
    size_t _mouthmaskemotions{};

    // table with the setting for each emotion - from config file
    EM_CODE* _emotion_table{};
    std::map<std::string, std::string > _bitmask_emotion_table;
    std::map<std::string, std::string > _color_table;

    //Time variables for automatic expression switching
    bool   _auto{};
    double _lasttime{};
    double _period{};
    int    _initEmotionTrigger{};

    int getIndex(std::string cmd);
    bool writePort(const char* cmd);

public:

    EmotionInterfaceModule();
    ~EmotionInterfaceModule() override = default;

    bool configure(ResourceFinder& config) override;//virtual bool open(Searchable& config);
    bool close() override;
    bool interruptModule() override;
    bool updateModule() override;
    double getPeriod() override;
    bool respond(const Bottle &command,Bottle &reply)override;
    void initEmotion();

    // interface functions
    bool setLeftEyebrow(const std::string cmd) override;
    bool setRightEyebrow(const std::string cmd) override;
    bool setEyelids(const std::string cmd) override;
    bool setMouth(const std::string cmd) override;
    bool setAll(const std::string cmd) override;
    bool setRaw(const std::string cmd) override;

    bool setColor(const string& cmd);
    bool setBrightness(const string& cmd);
    bool setMask(const Bottle& cmd);
};


#endif
