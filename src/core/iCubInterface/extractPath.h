// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef __EXTRACTPATH__
#define __EXTRACTPATH__

#include <string>

// simple code to extract path string from
// a filename

inline bool isSeparator(char c)
{
    if ( (c=='/') || (c=='\\'))
        return true;
    else
        return false;
}

inline std::string extractPath(const std::string &str)
{
    int len=str.length();
    int k=0;

    std::string tmp;
    std::string path;
    for(k=0;k<len;k++)
    {
        char current=str[k];
        tmp+=current;
    
        if (isSeparator(current))
        {
            path+=tmp;
            tmp.clear();
        }
    }
    return path;
}

#endif
