// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Jonas Ruesch (jruesch@jruesch.ch)
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __STRINGBUF__
#define __STRINGBUF__

// std
#include <streambuf>
#include <string>

namespace iCub {
    namespace contrib {
        class StringBuf;
    }
}

class iCub::contrib::StringBuf : std::streambuf {

public:

    StringBuf(int bufferSize) ;
    virtual ~StringBuf();

    

protected:

	virtual void writeString(const std::string &str) = 0;

    int	overflow(int c);
    int	sync();
};
#endif 
