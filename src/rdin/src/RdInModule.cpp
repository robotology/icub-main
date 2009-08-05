// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iCub/RdInModule.h>

#include <stdio.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

RdInModule::RdInModule(){

}

RdInModule::~RdInModule(){

}

bool RdInModule::open(Searchable& config){
	cout << "RDIN: Input is redirected to: " << getName("stdout") << endl;
	return _prtOut.open(getName("stdout"));
}

bool RdInModule::close(){
    _prtOut.close();
	fflush(stdout);
    return true;
}

bool RdInModule::interruptModule(){
	_prtOut.interrupt();
	fflush(stdout);
    return true;
}

bool RdInModule::updateModule(){
    return true;
}

double RdInModule::getPeriod() {
    return 0.1;
}

bool RdInModule::respond(const Bottle &command, Bottle &reply) 	{
	if(command.get(0).isVocab()){
		return yarp::os::Module::respond(command, reply);
	}
	cout << "RDIN: " << command.toString().c_str();
	Bottle &botOut = _prtOut.prepare();
	botOut.clear();
	botOut.addString(command.toString().c_str());
	_prtOut.write(true);
	fflush(stdout);
	return true;
}

