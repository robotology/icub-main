// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include<yarp/gui/StdoutCallback.h>

using namespace yarp::gui;
using namespace yarp::os;
using namespace std;

StdoutCallback::StdoutCallback() : TypedReaderCallback<yarp::os::Bottle>() {
	_vecStdout.clear();
}

StdoutCallback::~StdoutCallback(){
	_vecStdout.clear();
}

void StdoutCallback::onRead(Bottle &bot){
	
}

void StdoutCallback::onRead(Bottle &bot, const yarp::os::TypedReader<yarp::os::Bottle> &reader){
	_mutex.wait();
	for(int i = 0; i < bot.size(); i++){
		if(bot.get(i).isString()){	
			_vecStdout.push_back(std::string(bot.get(i).asString().c_str()));
		}
	}
	bot.clear();
	while(_vecStdout.size() > 3000){
		_vecStdout.pop_back();
	}
	_mutex.post();
}

string StdoutCallback::getReadings(){
	std::string strRes("");
	_mutex.wait();
	for(int i = 0; i < (int)_vecStdout.size()-1; i++){
		strRes += _vecStdout[i] + std::string("\n");
	}
	if(_vecStdout.size() > 0){
		strRes += _vecStdout[(int)_vecStdout.size() - 1]; // last one (no newline)
	}
	_vecStdout.clear();
	_mutex.post();
	return strRes;
}
