// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iCub/StringBuf.h>
#include <stdio.h>

using namespace iCub::contrib;
using namespace std;

StringBuf::StringBuf(int bufferSize){
    if (bufferSize){
        char *ptr = new char[bufferSize];
        setp(ptr, ptr + bufferSize);
    }
	else{
        setp(0, 0);
	}
}
StringBuf::~StringBuf(){
    sync();
    delete[] pbase();
}

int	StringBuf::overflow(int c){
    sync();
    if (c != EOF){
        if (pbase() == epptr()){
            std::string temp;
            temp += char(c);
            writeString(temp);
        }
		else{
            sputc(c);
		}
    }
    return 0;
}

int	StringBuf::sync() {
    if (pbase() != pptr()){
        int len = int(pptr() - pbase());
        std::string temp(pbase(), len);
        writeString(temp);
        setp(pbase(), epptr());
    }
    return 0;
}

