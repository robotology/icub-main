// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <string>
#include <cstdlib>
#include <bitset>
#include <iomanip>
#include <yarp/os/ResourceFinder.h>
#include "emotionInterfaceModule.h"


void EmotionInitReport::report(const PortInfo &info) {
    if ((emo!= nullptr) && info.created && !info.incoming)
        emo->initEmotion();
}

std::bitset<32> populateBitset(const yarp::os::Bottle& bt)
{
    std::bitset<32> bitSet{};
    for(size_t i=0; i<bt.size(); i++){
        bitSet.set(bt.get(i).asInt());
    }
    return bitSet;
}

std::string getHexCode(const Bottle& bot) {

    if( bot.size() < 2 )
    {
        return {};
    }

    auto botBits = bot.get(1).asList();
    if (!botBits || botBits->size() == 0 || botBits->size()>32)
    {
        yError()<<"The bitmask has to be defined as a list of <32 integers";
        return {};
    }
    auto bitSet = populateBitset(*botBits);
    stringstream ss;
    ss << setfill('0') << setw(8) << hex << uppercase << bitSet.to_ulong();
    return ss.str();
}

void printHelp (yarp::os::Bottle& reply) {

    std::string helpMessage{};
    helpMessage = helpMessage +
                      " commands are: \n" +
                      "help         to display this message\n" +
                      "\n"+
                      "set <part> <emotion>  set a specific emotion for a defined part   \n" +
                      "\tthe available part are: mou, eli, leb, reb, all\n"+
                      "\tthe available emotions are: neu, hap, sad, sur, ang, evi, shy, cun\n"+
                      "\n"+
                      "set col <color>     set the color of the led\n" +
                      "\t!! available only for rfe board !!\n"+
                      "\tthe available colors are: black, white, red, lime, blue, yellow,"
                      " cyan, magenta, silver, gray, maroon, olive, green, purple, teal, navy\n"+
                      "\n"+
                      "set brig <brig>       set the brightness of the leds\n"+
                      "\t!! available only for rfe board !!\n"+
                      "\tthe brightness is defined by an iteger from 0 to 5, where 0 means led of\n"+
                      "\n"+
                      "set mask (<col_leb> <m_name_leb> <brig_leb>) (<col_reb> <m_name_reb> <brig_reb>) (<col_mou> <m_name_mou> <brig_mou>) set color, bitmask and brightness for each part(leb, reb, mou)\n"+
                      "\t!! available only for rfe board !!\n"+
                      "\tm_name stands for mask name and the availble bitmasks can be consulted/added in faceExpressions/emotions_rfe.ini\n";

    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString(helpMessage.c_str());
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
    _numberOfColors = config.check("colors", Value(0), "Number of predefined colors").asInt();
    _eyebrowmaskemotions = config.check("bitmask_eyebrow_emotions", Value(0), "Number of predefined bitmask eyebrow expressions").asInt();
    _mouthmaskemotions = config.check("bitmask_mouth_emotions", Value(0), "Number of predefined bitmask eyebrow expressions").asInt();
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
    if(_numberOfColors) {
        std::string color_id;
        for(size_t index=0; index<_numberOfColors; index++){
            color_id = "C"+std::to_string(index);
            Bottle& bot = config.findGroup(color_id);
            if( bot.size() < 3 )
            {
                yError("Invalid parameter list for the identifier %s.", color_id.c_str());
                return false;
            }

            std::string code = bot.get(2).toString();
            if (code.length()!=3)
            {
                yError("The identifier must have size 3");
                return false;
            }
            _color_table[bot.get(1).asString()] = code.substr(0,code.length()-1);
        }
    }
    if(_eyebrowmaskemotions) {
        std::string id;
        for(size_t index=0; index<_eyebrowmaskemotions; index++){
            id = "BM_EB"+std::to_string(index);
            Bottle& bot = config.findGroup(id);
            auto hexCode = getHexCode(bot);
            if(hexCode.empty()){
                yError()<<"Parsing of the bitmap failed";
                return false;
            }
            _bitmask_emotion_table[id] = hexCode;
        }

    }
    if(_mouthmaskemotions) {

        std::string id;
        for(size_t index=0; index<_eyebrowmaskemotions; index++){
            id = "BM_M"+std::to_string(index);
            Bottle& bot = config.findGroup(id);
            auto hexCode = getHexCode(bot);
            if(hexCode.empty()){
                yError()<<"Parsing of the bitmap failed";
                return false;
            }
            _bitmask_emotion_table[id] = hexCode;
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

    if(_inputPort.isOpen())
        _inputPort.close();
    if(!_outputPort.isClosed())
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
        case EMOTION_VOCAB_HELP:
            printHelp(reply);
            return true;
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
                case EMOTION_VOCAB_COLOR:{
                    ok = setColor(command.get(2).toString());
                    break;
                }
                case EMOTION_VOCAB_BRIG: {
                    ok = setBrightness(command.get(2).toString());
                    break;
                }
                case EMOTION_VOCAB_MASK:{
                    ok = setMask(command);
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

bool EmotionInterfaceModule::setColor(const std::string& cmd)
{
    char cmdbuffer[] = {0,0,0,0};

    if(_color_table.find(cmd) == _color_table.end()){
        yError()<<"Color"<<cmd<<"not available";
        return false;
    }

    cmdbuffer[0]= 'C';
    cmdbuffer[1]=_color_table[cmd][0];
    cmdbuffer[2]=_color_table[cmd][1];
    return writePort(cmdbuffer);
}

bool EmotionInterfaceModule::setBrightness(const std::string& cmd)
{
    char cmdbuffer[] = {0,0,0,0};

    cmdbuffer[0]= 'B';
    cmdbuffer[1]= '0';
    cmdbuffer[2]= cmd[0];
    return writePort(cmdbuffer);
}


bool EmotionInterfaceModule::setMask(const Bottle& cmd)
{
    string cmdbuffer{};
    if(cmd.size()!=5) {
        yError()<<"Bad request, it should be set mask (color mask) (color mask) (color mask)";
        return false;
    }
    auto botLeb = cmd.get(2).asList(); // bottle of the left eyebrow
    auto botReb = cmd.get(3).asList(); // bottle of the right eyebrow
    auto botMou = cmd.get(4).asList(); // bottle of the mouth eyebrow

    if(!botLeb || !botReb || !botMou) {
        yError()<<"Bad request, missing one of the three list";
        return false;
    }

    if(botLeb->size()!=3 || botReb->size()!=3 || botMou->size()!=3) {
        yError()<<"Bad request, each list has to be (color mask brightness)";
        return false;
    }
    auto colLeb = botLeb->get(0).asString();
    auto colReb = botReb->get(0).asString();
    auto colMou = botMou->get(0).asString();

    auto maskLeb = botLeb->get(1).asString();
    auto maskReb = botReb->get(1).asString();
    auto maskMou = botMou->get(1).asString();

    auto brightLeb = '0'+ botLeb->get(2).toString();
    auto brightReb = '0'+ botReb->get(2).toString();
    auto brightMou = '0'+ botMou->get(2).toString();

    if(_bitmask_emotion_table.find(maskLeb) == _bitmask_emotion_table.end() ||
       _bitmask_emotion_table.find(maskReb) == _bitmask_emotion_table.end() ||
       _bitmask_emotion_table.find(maskMou) == _bitmask_emotion_table.end()) {
        yError()<<"One or more bitmask has not been defined in the ini file";
        return false;
    }

    cmdbuffer += 'Z';
    // LEB
    cmdbuffer += _color_table[colLeb];
    cmdbuffer += brightLeb;
    cmdbuffer += _bitmask_emotion_table[maskLeb];
    // REB
    cmdbuffer += _color_table[colReb];
    cmdbuffer += brightReb;
    cmdbuffer += _bitmask_emotion_table[maskReb];
    // Mouth
    cmdbuffer += _color_table[colMou];
    cmdbuffer += brightMou;
    cmdbuffer += _bitmask_emotion_table[maskMou];

    return writePort(cmdbuffer.c_str());
}
