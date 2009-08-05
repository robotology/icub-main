// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/RemoteAttentionSelection.h>

using namespace std;
using namespace iCub::contrib;
using namespace yarp::os;
using namespace yarp::dev;

RemoteAttentionSelection::RemoteAttentionSelection(){

}
   
RemoteAttentionSelection::~RemoteAttentionSelection(){

}

bool RemoteAttentionSelection::open(const char *name){
	if(name != NULL){
		return configPort.open(name);
	}
	else{
		return false;
	}
}

bool RemoteAttentionSelection::close(){
    configPort.close();
    return true;
}

void RemoteAttentionSelection::setInhibitOutput(bool on){
    setCommand(ATTENTIONSELECTION_VOCAB_THRESHOLD_OUTPUT, (int)on);
}

bool RemoteAttentionSelection::getInhibitOutput(){
    int v = 0;
    getCommand(ATTENTIONSELECTION_VOCAB_THRESHOLD_OUTPUT, v);
	if( v == 0){
		return false;
	}
	else{
		return true;
	}
}

// ***********************

void RemoteAttentionSelection::setCommand(int code) {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(code);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);    
}

void RemoteAttentionSelection::setCommand(int code, double v) {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addDouble(v);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
}

void RemoteAttentionSelection::setCommand(int code, int v) {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(v);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);    
}

void RemoteAttentionSelection::setCommand(int code, string s){
    Bottle cmd;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addString(s.c_str());
    configPort.write(cmd);
}

bool RemoteAttentionSelection::getCommand(int code, double& v) const {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (ATTENTIONSELECTION_CHECK_FAIL(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asDouble();
        return true;
    }
    return false;
}

bool RemoteAttentionSelection::getCommand(int code, int& v) const {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (ATTENTIONSELECTION_CHECK_FAIL(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asInt();
        return true;
    }
    return false;
}

bool RemoteAttentionSelection::getCommand(int code, string& s) const {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (ATTENTIONSELECTION_CHECK_FAIL(ok, response)) {
        s = string(response.get(2).asString().c_str());
        return true;
    }
    return false;
}

void RemoteAttentionSelection::setDouble (int code, int j, double val) {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(j);
    cmd.addDouble(val);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
}


void RemoteAttentionSelection::setDoubleBottle(int v, Bottle &bot){
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(v);
    for (int i = 0; i < bot.size(); i++)
        if (bot.get(i).isDouble())
            cmd.addDouble(bot.get(i).asDouble());
   //bool ok = configPort.write(cmd, response);
   //return CHECK_FAIL(ok, response);
   configPort.write(cmd);
}

bool RemoteAttentionSelection::getDouble(int v, int j, double *val) {
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_GET);
    cmd.addVocab(v);
    cmd.addInt(j);
    bool ok = configPort.write(cmd, response);
    if (ATTENTIONSELECTION_CHECK_FAIL(ok, response)) {
        // ok
        *val = response.get(2).asDouble();
        return true;
    }
    return false;
}

bool RemoteAttentionSelection::getDoubleBottle(int v, Bottle &bot){
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_GET);
    cmd.addVocab(v);
    bool ok = configPort.write(cmd, response);
    if (ATTENTIONSELECTION_CHECK_FAIL(ok, response)){
        for (int i = 2; i < response.size(); i++)
            bot.addDouble(response.get(i).asDouble());
        return true;
    }
    return false;
}

bool RemoteAttentionSelection::getString(int code, int j, string& s){
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_GET);
    cmd.addVocab(code);
    cmd.addInt(j);
    bool ok = configPort.write(cmd, response);
    if (ATTENTIONSELECTION_CHECK_FAIL(ok, response)) {
        // ok
        s = string(response.get(2).asString().c_str());
        return true;
    }
    return false;
}

void RemoteAttentionSelection::setString(int code, int j, string s){
    Bottle cmd, response;
    cmd.addVocab(ATTENTIONSELECTION_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(j);
    cmd.addString(s.c_str());
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
}

