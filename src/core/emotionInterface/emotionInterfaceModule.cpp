// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <string>
#include <cstdlib>
#include <yarp/os/ResourceFinder.h>
#include "emotionInterfaceModule.h"


void EmotionInitReport::report(const PortInfo &info) {
    if ((emo!= nullptr) && info.created && !info.incoming)
        emo->initEmotion();
}


EmotionInterfaceModule::EmotionInterfaceModule() : emotionInitReport(this) {
}

bool EmotionInterfaceModule::configure(ResourceFinder& config){
  
    char name[10];
    int i;

    std::string modName = config.find("name").asString();
    setName(modName.c_str());

    _lasttime = Time::now();
    if (config.check("help","if present, display usage message")) {
        yError("Call with --name /module_prefix --file configFile.ini \n");
        return false;
    }

    _highlevelemotions = config.check("emotions", Value(0), "Number of predefined facial expressions").asInt();
    _auto = config.check("auto");
    _period = config.check("period", Value(10.0), "Period for expression switching in auto mode").asDouble();
    if(_highlevelemotions == 0) 
    {
        _emotion_table = nullptr;
    }
    else //allocate space for facial expression codes
    {
        _emotion_table = new EM_CODE[_highlevelemotions];
        if(!_emotion_table)
        {
            yError("Memory allocation problem\n");
            return false;
        }
        for(i = 0; i < _highlevelemotions; i++)
        {
            sprintf(name,"E%d",i+1);
            if(!config.check(name))
            {
                yError("Missing identifier %s.", name);
                return false;
            }
            else
            {
                Bottle& bot = config.findGroup(name);
                if( bot.size() < 6 )
                {
                    printf("Invalid parameter list for identifier %s.", name);
                    return false;
                }
                //first field - name of the expression
                std::string n1 = bot.get(1).toString();
                
                if(n1.length()!=3) //must have length 3
                {
                    yError("First field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n1.c_str();
                    strncpy(_emotion_table[i].name, buffer, 3);
                }

                //second field - command to left eyebrow
                std::string n2 = bot.get(2).toString();
                const char * sfd = n2.c_str();
                if(n2.length()!=3) //must have length 3
                {
                    yError("Second field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n2.c_str();
                    strncpy(_emotion_table[i].leb, buffer, 2);
                }
                //third field - command to right eyebrow
                std::string n3 = bot.get(3).toString();
                if(n3.length()!=3) //must have length 3
                {
                    yError("Third field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n3.c_str();
                    strncpy(_emotion_table[i].reb, buffer, 2);
                }

                //fourth field - command to mouth
                std::string n4 = bot.get(4).toString();
                if(n4.length()!=3) //must have length 3
                {
                    yError("Fourth field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n4.c_str();
                    strncpy(_emotion_table[i].mou, buffer, 2);
                }

                //fifth field - command to eyelids
                std::string n5 = bot.get(5).toString();
                if(n5.length()!=3) //must have length 3
                {
                    yError("Fifth field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n5.c_str();
                    strncpy(_emotion_table[i].eli, buffer, 2);
                }
            }
        }
    }
    
      // open  ports
      
    _inputPort.open(getName("/in")); 
    _outputPort.open(getName("/out"));
    _outputPort.setReporter(emotionInitReport);
    _initEmotionTrigger=0;
    
    attach(_inputPort);

    return true;
}

bool EmotionInterfaceModule::close(){
    
    _inputPort.close();
    _outputPort.close();
    
    if (_emotion_table != nullptr)
    {
        delete [] _emotion_table;
        _emotion_table = nullptr;
    }

    return true;
}

bool EmotionInterfaceModule::interruptModule(){
    
    _inputPort.interrupt();
    _outputPort.interrupt();
    return true;
}

void EmotionInterfaceModule::initEmotion(){
    _initEmotionTrigger++;
}

bool EmotionInterfaceModule::updateModule(){
    double curtime;
    int expr;
    if( _auto )
    {
        curtime = Time::now();
        if(curtime - _lasttime > _period) //change expression
        {
           _lasttime = curtime;
           expr = rand() % _highlevelemotions;
           std::string cmd(_emotion_table[expr].name);
           setAll(cmd);
        }
    }
    else if (_initEmotionTrigger>0)
    {
        setAll("hap");
        _initEmotionTrigger=0;
    }
    return true;
}

double EmotionInterfaceModule::getPeriod(){
    return 1.0;
}

bool EmotionInterfaceModule::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    switch (command.get(0).asVocab()) {
   
    case EMOTION_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case EMOTION_VOCAB_MOUTH:{
                ok = setMouth(command.get(2).toString());
                break;
            }
            case EMOTION_VOCAB_EYELIDS:{
                ok = setEyelids(command.get(2).toString());
                break;
            }
            case EMOTION_VOCAB_LEFTEYEBROW:{
                ok = setLeftEyebrow(command.get(2).toString());
                break;
            }
            case EMOTION_VOCAB_RIGHTEYEBROW:{
                ok = setRightEyebrow(command.get(2).toString());
                break;
            }
            case EMOTION_VOCAB_ALL:{
                ok = setAll(command.get(2).toString());
                break;
            }
            case EMOTION_VOCAB_RAW:{
                ok = setRaw(command.get(2).toString());
                break;
            }
            default:
                cout << "received an unknown request after a VOCAB_SET" << endl;
                break;
            }
        }
        break;
    case EMOTION_VOCAB_GET:
        rec = true;
        /*{
            reply.addVocab(VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case EGOSPHERE_VOCAB_THRESHOLD_SALIENCE:{
                float thr = getThresholdSalience();
                reply.addDouble((double)thr);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_OUTPUT:{
                int v = (int)getOutput();
                reply.addInt(v);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_SALIENCE_DECAY:{
                double rate = getSalienceDecay();
                reply.addDouble(rate);
                ok = true;
                break;
            }
            break;
            default:
                cout << "received an unknown request after a VOCAB_GET" << endl;
                break;
            }
        }
        */
        break;

    }

    if (!rec)
        ok = false;
    
    if (!ok) {
        reply.clear();
        reply.addVocab(EMOTION_VOCAB_FAILED);
    }
    else
        reply.addVocab(EMOTION_VOCAB_OK);

    return ok;
}   

//get the index in _emotions_table of a emotion name
int EmotionInterfaceModule::getIndex(const std::string cmd)
{
    if(_highlevelemotions == 0)
        return -1;
 
    int i;
    for(i = 0; i < _highlevelemotions; i++)
    {
        if(strncmp(_emotion_table[i].name, cmd.c_str(), 3) == 0) //strings identical
            break;
    }

    if( i == _highlevelemotions ) // no match
       return -1;
    
    return i;
}

//send the actual string to the port
bool EmotionInterfaceModule::writePort(const char* cmd)
{
    Bottle &btmp = _outputPort.prepare();
    btmp.clear();
    btmp.addString(cmd);
    _outputPort.write(true);
    Time::delay(0.001);
    return true;
}


// interface functions
bool EmotionInterfaceModule::setLeftEyebrow(const std::string cmd)
{
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
        return false;

    if( _emotion_table[i].leb[0] == '*' || _emotion_table[i].leb[1] == '*') 
        return true;  //leave it in the same state

    cmdbuffer[0]= 'L';
    cmdbuffer[1]=_emotion_table[i].leb[0];
    cmdbuffer[2]=_emotion_table[i].leb[1];
    writePort(cmdbuffer);
    return true;
}

bool EmotionInterfaceModule::setRightEyebrow(const std::string cmd)
{
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
        return false;

    if( _emotion_table[i].reb[0] == '*' || _emotion_table[i].reb[1] == '*') 
        return true;  //leave it in the same state

    cmdbuffer[0]= 'R';
    cmdbuffer[1]=_emotion_table[i].reb[0];
    cmdbuffer[2]=_emotion_table[i].reb[1];
    writePort(cmdbuffer);
    return true;
}

bool EmotionInterfaceModule::setMouth(const std::string cmd)
{
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
        return false;

    if( _emotion_table[i].mou[0] == '*' || _emotion_table[i].mou[1] == '*') 
        return true;  //leave it in the same state

    cmdbuffer[0]= 'M';
    cmdbuffer[1]=_emotion_table[i].mou[0];
    cmdbuffer[2]=_emotion_table[i].mou[1];
    writePort(cmdbuffer);
    return true;
}

bool EmotionInterfaceModule::setEyelids(const std::string cmd)
{
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
        return false;

    if( _emotion_table[i].eli[0] == '*' || _emotion_table[i].eli[1] == '*') 
        return true;  //leave it in the same state

    cmdbuffer[0]= 'S';
    cmdbuffer[1]=_emotion_table[i].eli[0];
    cmdbuffer[2]=_emotion_table[i].eli[1];
    writePort(cmdbuffer);
    return true;
}

bool EmotionInterfaceModule::setAll(const std::string cmd)
{
    setLeftEyebrow(cmd);
    setRightEyebrow(cmd);
    setMouth(cmd);
    setEyelids(cmd);
    return true;
}

bool EmotionInterfaceModule::setRaw(const std::string cmd)
{
    writePort(cmd.c_str());
    return true;
}


