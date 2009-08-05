// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/RemoteEgoSphere.h>

RemoteEgoSphere::RemoteEgoSphere(){

}
   
RemoteEgoSphere::~RemoteEgoSphere(){

}

bool RemoteEgoSphere::open(const char *name){
	if(name != NULL){
		return configPort.open(name);
	}
	else{
		return false;
	}
}

bool RemoteEgoSphere::close(){
    configPort.close();
    return true;
}

bool RemoteEgoSphere::reset(){
    Bottle cmd;
    cmd.addVocab(EGOSPHERE_VOCAB_RESET);
    configPort.write(cmd);
    return true;
}

bool RemoteEgoSphere::setSaccadicSuppression(bool on){
    return setCommand(EGOSPHERE_VOCAB_SACCADICSUPPRESSION, (int)on);
}

bool RemoteEgoSphere::getSaccadicSuppression(){
    int v = 0;
    getCommand(EGOSPHERE_VOCAB_SACCADICSUPPRESSION, v);
	if( v == 0){
		return false;
	}
	else{
		return true;
	}
}

bool RemoteEgoSphere::setSalienceDecay(double rate){
    return setCommand(EGOSPHERE_VOCAB_SALIENCE_DECAY, rate);
}

double RemoteEgoSphere::getSalienceDecay(){
    double rate = -1.0;
    getCommand(EGOSPHERE_VOCAB_SALIENCE_DECAY, rate);
    return rate;
}

bool RemoteEgoSphere::addIORRegion(double azimuth, double elevation){
	Bottle cmd;
	cmd.addVocab(EGOSPHERE_VOCAB_ADDIORREGION);
	cmd.addDouble(azimuth);
	cmd.addDouble(elevation);
	return configPort.write(cmd);
}

// ***********************

bool RemoteEgoSphere::setCommand(int code) {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(code);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
    return true;
}

bool RemoteEgoSphere::setCommand(int code, double v) {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addDouble(v);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
    return true;
}

bool RemoteEgoSphere::setCommand(int code, int v) {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(v);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
    return true;
}

bool RemoteEgoSphere::setCommand(int code, string s){
    Bottle cmd;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addString(s.c_str());
    configPort.write(cmd);
    return true;
}

bool RemoteEgoSphere::getCommand(int code, double& v) const {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (EGOSPHERE_CHECK_FAIL(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asDouble();
        return true;
    }
    return false;
}

bool RemoteEgoSphere::getCommand(int code, int& v) const {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (EGOSPHERE_CHECK_FAIL(ok, response)) {
        // response should be [cmd] [name] value
        v = response.get(2).asInt();
        return true;
    }
    return false;
}

bool RemoteEgoSphere::getCommand(int code, string& s) const {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_GET);
    cmd.addVocab(code);
    bool ok = configPort.write(cmd, response);
    if (EGOSPHERE_CHECK_FAIL(ok, response)) {
        s = string(response.get(2).asString().c_str());
        return true;
    }
    return false;
}

bool RemoteEgoSphere::setDouble (int code, int j, double val) {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(j);
    cmd.addDouble(val);
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
    return true;
}


bool RemoteEgoSphere::setDoubleBottle(int v, Bottle &bot){
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(v);
    for (int i = 0; i < bot.size(); i++)
        if (bot.get(i).isDouble())
            cmd.addDouble(bot.get(i).asDouble());
   //bool ok = configPort.write(cmd, response);
   //return CHECK_FAIL(ok, response);
   configPort.write(cmd);
   return true;
}

bool RemoteEgoSphere::getDouble(int v, int j, double *val) {
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_GET);
    cmd.addVocab(v);
    cmd.addInt(j);
    bool ok = configPort.write(cmd, response);
    if (EGOSPHERE_CHECK_FAIL(ok, response)) {
        // ok
        *val = response.get(2).asDouble();
        return true;
    }
    return false;
}

bool RemoteEgoSphere::getDoubleBottle(int v, Bottle &bot){
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_GET);
    cmd.addVocab(v);
    bool ok = configPort.write(cmd, response);
    if (EGOSPHERE_CHECK_FAIL(ok, response)){
        for (int i = 2; i < response.size(); i++)
            bot.addDouble(response.get(i).asDouble());
        return true;
    }
    return false;
}

bool RemoteEgoSphere::getString(int code, int j, string& s){
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_GET);
    cmd.addVocab(code);
    cmd.addInt(j);
    bool ok = configPort.write(cmd, response);
    if (EGOSPHERE_CHECK_FAIL(ok, response)) {
        // ok
        s = string(response.get(2).asString().c_str());
        return true;
    }
    return false;
}

bool RemoteEgoSphere::setString(int code, int j, string s){
    Bottle cmd, response;
    cmd.addVocab(EGOSPHERE_VOCAB_SET);
    cmd.addVocab(code);
    cmd.addInt(j);
    cmd.addString(s.c_str());
    //bool ok = configPort.write(cmd, response);
    //return CHECK_FAIL(ok, response);
    configPort.write(cmd);
    return true;
}

