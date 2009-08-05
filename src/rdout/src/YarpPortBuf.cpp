// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iCub/YarpPortBuf.h>

using namespace iCub::contrib;
using namespace yarp::os;
using namespace std;

const int YARPPORTBUF_SIZE = 1024;

YarpPortBuf::YarpPortBuf() : StringBuf(YARPPORTBUF_SIZE) {
}

YarpPortBuf::~YarpPortBuf(){
	this->close();
}

void YarpPortBuf::writeString(const std::string &str) {
	if(!_prtOut.isClosed()){
		Bottle &bot = _prtOut.prepare();
		bot.addString(str.c_str());
		_prtOut.write(true);
	}
}

bool YarpPortBuf::open(std::string portName){
	return _prtOut.open(portName.c_str());
}

bool YarpPortBuf::close(){
	_prtOut.interrupt();
	_prtOut.close();
	return true;
}
