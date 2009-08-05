// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/Framerate.h>

using namespace iCub::contrib;

Framerate::Framerate(){
	init_internal(1);
}

Framerate::Framerate(int averageOverNumPeriods){
	init_internal(averageOverNumPeriods);
}

void Framerate::init(int averageOverNumPeriods){
	delete [] _vctDiffs;
	init_internal(averageOverNumPeriods);
}

void Framerate::init_internal(int averageOverNumPeriods){
	if(averageOverNumPeriods > 0){
        _vctDiffs = new double[averageOverNumPeriods];
        _periods = averageOverNumPeriods;
    }
    else{
        _periods = 1;
        _vctDiffs = new double[1];
    }
    _counter = 0;
    _framerate = 0.0f;
    for (int i = 0; i < _periods; i++){
        _vctDiffs[i] = 0.0;
    }
}

Framerate::~Framerate(){
    delete [] _vctDiffs;
}

void Framerate::addStartTime(double time){
    _startTime = time;
}

void Framerate::addEndTime(double time){
    _vctDiffs[_counter] = time - _startTime;
    _counter++;
    if (_counter == _periods){
        double sum = 0;
        for (int i = 0; i < _periods; i++)
            sum += _vctDiffs[i];
        _framerate = sum/_periods;
        _framerate = 1.0f/_framerate;
         _counter = 0;
    }
}

float Framerate::getFramerate(){
    return _framerate;
}

bool Framerate::hasNewFramerate(){
    if (_counter == 0)
        return true;
    else
        return false;
}


