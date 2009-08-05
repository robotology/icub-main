// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/RemoteSalience.h>

RemoteSalience::RemoteSalience(){
}
   
RemoteSalience::~RemoteSalience(){

}

bool RemoteSalience::open(const char * name){
	if(name != NULL){
		return configPort.open(name);
	}
	else{
		return false;
	}
}

bool RemoteSalience::close(){
    configPort.close();
    return true;
}

bool RemoteSalience::setFilterName(std::string n){
    return setCommand(SALIENCE_VOCAB_NAME, n);
}

std::string RemoteSalience::getFilterName(){
    string name;
    getCommand(SALIENCE_VOCAB_NAME, name);
    return name;
}

bool RemoteSalience::setChildFilterName(int j, std::string n){
    return setString(SALIENCE_VOCAB_CHILD_NAME, j, n);
}

std::string RemoteSalience::getChildFilterName(int j){
    string s("");
    getString(SALIENCE_VOCAB_CHILD_NAME, j, s);
    return s;
}

bool RemoteSalience::setWeight(double w){
    return setCommand(SALIENCE_VOCAB_WEIGHT, w);
}

double RemoteSalience::getWeight(){
    double w = 0.0;
    getCommand(SALIENCE_VOCAB_WEIGHT, w);
    return w;
}

int RemoteSalience::getChildCount(){
    int cc = -1;
    getCommand(SALIENCE_VOCAB_CHILD_COUNT, cc);
    return cc;
}

bool RemoteSalience::setChildWeights(yarp::os::Bottle& subWeights){
    return setDoubleBottle(SALIENCE_VOCAB_CHILD_WEIGHTS, subWeights);
}

bool RemoteSalience::getChildWeights(yarp::os::Bottle *subWeights){
    return getDoubleBottle(SALIENCE_VOCAB_CHILD_WEIGHTS, *subWeights);
}

bool RemoteSalience::setChildWeight(int j, double w){
    return setDouble(SALIENCE_VOCAB_CHILD_WEIGHT, j, w);
}

double RemoteSalience::getChildWeight(int j){
    double w = 0.0;
    getDouble(SALIENCE_VOCAB_CHILD_WEIGHT, j, &w);
    return w;
}

double RemoteSalience::getSalienceThreshold(){
    double t = 0.0;
    getCommand(SALIENCE_VOCAB_SALIENCE_THRESHOLD, t);
    return t;
}

bool RemoteSalience::setSalienceThreshold(double thr){
    return setCommand(SALIENCE_VOCAB_SALIENCE_THRESHOLD, thr);
}

int RemoteSalience::getNumBlurPasses(){
    int nb = -1;
    getCommand(SALIENCE_VOCAB_NUM_BLUR_PASSES, nb);
    return nb;
}

bool RemoteSalience::setNumBlurPasses(int num){
    return setCommand(SALIENCE_VOCAB_NUM_BLUR_PASSES, num);
}   

//int RemoteSalience::getTemporalBlur(){
//    int size;
//    getCommand(SALIENCE_VOCAB_TEMPORAL_BLUR, size);
//    return size;
//}

//bool RemoteSalience::setTemporalBlur(int size){
//    return setCommand(SALIENCE_VOCAB_TEMPORAL_BLUR, size);
//}

bool RemoteSalience::setCommand(int code) {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(code);
    //bool ok = configPort.write(cmd, response);
    //return SALIENCE_CHECK_FAILED(ok, response);
    configPort.write(cmd);
    return true;
}

bool RemoteSalience::setCommand(int code, double v) {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addDouble(v);
    //bool ok = configPort.write(cmd, response);
    //return SALIENCE_CHECK_FAILED(ok, response);
    configPort.write(cmd);
    return true;
}

bool RemoteSalience::setCommand(int code, int v) {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(v);
    //bool ok = configPort.write(cmd, response);
    //return SALIENCE_CHECK_FAILED(ok, response);
    configPort.write(cmd);
    return true;
}

bool RemoteSalience::setCommand(int code, string s){
    Bottle cmd;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addString(s.c_str());
    configPort.write(cmd);
    return true;
}

bool RemoteSalience::getCommand(int code, double& v) const {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (SALIENCE_CHECK_FAILED(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asDouble();
        return true;
    }
    return false;
}

bool RemoteSalience::getCommand(int code, int& v) const {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (SALIENCE_CHECK_FAILED(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asInt();
        return true;
    }
    return false;
}

bool RemoteSalience::getCommand(int code, string& s) const {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (SALIENCE_CHECK_FAILED(ok, response)) {
        s = string(response.get(2).asString().c_str());
        return true;
    }
    return false;
}

bool RemoteSalience::setDouble (int code, int j, double val) {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(j);
    cmd.addDouble(val);
    //bool ok = configPort.write(cmd, response);
    //return SALIENCE_CHECK_FAILED(ok, response);
    configPort.write(cmd);
    return true;
}


bool RemoteSalience::setDoubleBottle(int v, Bottle &bot){
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(v);
    for (int i = 0; i < bot.size(); i++)
        if (bot.get(i).isDouble())
            cmd.addDouble(bot.get(i).asDouble());
   //bool ok = configPort.write(cmd, response);
   //return SALIENCE_CHECK_FAILED(ok, response);
   configPort.write(cmd);
   return true;
}

bool RemoteSalience::getDouble(int v, int j, double *val) {
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_GET);
    cmd.addVocab(v);
    cmd.addInt(j);
    bool ok = configPort.write(cmd, response);
    if (SALIENCE_CHECK_FAILED(ok, response)) {
        // ok
        *val = response.get(2).asDouble();
        return true;
    }
    return false;
}

bool RemoteSalience::getDoubleBottle(int v, Bottle &bot){
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_GET);
    cmd.addVocab(v);
    bool ok = configPort.write(cmd, response);
    if (SALIENCE_CHECK_FAILED(ok, response)){
        for (int i = 2; i < response.size(); i++)
            bot.addDouble(response.get(i).asDouble());
        return true;
    }
    return false;
}

bool RemoteSalience::getString(int code, int j, string& s){
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_GET);
    cmd.addVocab(code);
    cmd.addInt(j);
    bool ok = configPort.write(cmd, response);
    if (SALIENCE_CHECK_FAILED(ok, response)) {
        // ok
        s = string(response.get(2).asString().c_str());
        return true;
    }
    return false;
}

bool RemoteSalience::setString(int code, int j, string s){
    Bottle cmd, response;
    cmd.addVocab(SALIENCE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(j);
    cmd.addString(s.c_str());
    //bool ok = configPort.write(cmd, response);
    //return SALIENCE_CHECK_FAILED(ok, response);
    configPort.write(cmd);
    return true;
}

