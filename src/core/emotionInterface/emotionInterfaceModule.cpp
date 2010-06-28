// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "emotionInterfaceModule.h"
#include <string.h>
#include <stdlib.h>


EmotionInterfaceModule::EmotionInterfaceModule(){

}

EmotionInterfaceModule::~EmotionInterfaceModule(){
   
}

bool EmotionInterfaceModule::open(Searchable& config){
  
    char name[10];
    int i;

    _lasttime = Time::now();
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini \n");
        return false;
    }

    _highlevelemotions = config.check("emotions", 0, "Number of predefined facial expressions").asInt();
    _auto = config.check("auto");
    _period = config.check("period", 10, "Period for expression switching in auto mode").asDouble();
    if(_highlevelemotions == 0) 
    {
        _emotion_table = NULL;
    }
    else //allocate space for facial expression codes
    {
        _emotion_table = new EM_CODE[_highlevelemotions];
        if(!_emotion_table)
        {
            printf("Memory allocation problem\n");
            return false;
        }
        for(i = 0; i < _highlevelemotions; i++)
        {
            sprintf(name,"E%d",i+1);
            if(!config.check(name))
            {
                printf("Missing identifier %s.", name);
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
                ConstString n1 = bot.get(1).toString();
                
                if(n1.length()!=3) //must have length 3
                {
                    printf("First field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n1.c_str();
                    strncpy(_emotion_table[i].name, buffer, 3);
                }

                //second field - command to left eyebrow
                ConstString n2 = bot.get(2).toString();
                const char * sfd = n2.c_str();
                if(n2.length()!=3) //must have length 3
                {
                    printf("Second field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n2.c_str();
                    strncpy(_emotion_table[i].leb, buffer, 2);
                }
                //third field - command to right eyebrow
                ConstString n3 = bot.get(3).toString();
                if(n3.length()!=3) //must have length 3
                {
                    printf("Third field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n3.c_str();
                    strncpy(_emotion_table[i].reb, buffer, 2);
                }

                //fourth field - command to mouth
                ConstString n4 = bot.get(4).toString();
                if(n4.length()!=3) //must have length 3
                {
                    printf("Fourth field of identifier %s has invalid size (must be 3).", name);
                    return false;
                }
                else
                {
                    const char *buffer = n4.c_str();
                    strncpy(_emotion_table[i].mou, buffer, 2);
                }

                //fifth field - command to eyelids
                ConstString n5 = bot.get(5).toString();
                if(n5.length()!=3) //must have length 3
                {
                    printf("Fifth field of identifier %s has invalid size (must be 3).", name);
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
    _inputPort.open(getName("in"));
    _outputPort.open(getName("out"));
    
    attach(_inputPort, true);
    attachTerminal();

    Time::turboBoost();
    return true;
}

bool EmotionInterfaceModule::close(){
    
    _inputPort.close();
    _outputPort.close();
    
    if (_emotion_table != NULL)
    {
        delete [] _emotion_table;
        _emotion_table = NULL;
    }
    
    return true;
}

bool EmotionInterfaceModule::interruptModule(){
    
    _inputPort.interrupt();
    _outputPort.interrupt();
    return true;
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
           ConstString cmd(_emotion_table[expr].name);
           setAll(cmd);
        }
    }
    Time::delay(0.1);
    return true;
}


bool EmotionInterfaceModule::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    _semaphore.wait();
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
    _semaphore.post();

    if (!rec)
        ok = Module::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(EMOTION_VOCAB_FAILED);
    }
    else
        reply.addVocab(EMOTION_VOCAB_OK);

    return ok;
} 	

//get the index in _emotions_table of a emotion name
int EmotionInterfaceModule::getIndex(const ConstString cmd)
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
bool EmotionInterfaceModule::setLeftEyebrow(const ConstString cmd)
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

bool EmotionInterfaceModule::setRightEyebrow(const ConstString cmd)
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

bool EmotionInterfaceModule::setMouth(const ConstString cmd)
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

bool EmotionInterfaceModule::setEyelids(const ConstString cmd)
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

bool EmotionInterfaceModule::setAll(const ConstString cmd)
{
    setLeftEyebrow(cmd);
    setRightEyebrow(cmd);
    setMouth(cmd);
    setEyelids(cmd);
    return true;
}

bool EmotionInterfaceModule::setRaw(const ConstString cmd)
{
    writePort(cmd.c_str());
    return true;
}



