/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

#include "iCub/skinDynLib/dynContactList.h"
#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::skinDynLib;


dynContactList::dynContactList()
:vector<dynContact>(){}

dynContactList::dynContactList(const size_type &n, const dynContact& value)
:vector<dynContact>(n, value){}


//~~~~~~~~~~~~~~~~~~~~~~~~~~
//   SERIALIZATION methods
//~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContactList::read(ConnectionReader& connection){
    int listLength = connection.expectInt();
    if(listLength<0)
        return false;
    if(listLength!=this->size())
        this->resize(listLength);

    for(iterator it=begin(); it!=end(); it++){
        it->read(connection);
    }

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContactList::write(ConnectionWriter& connection){
    connection.appendInt(this->size());
    for(iterator it=begin(); it!=end(); it++){
        if(!it->write(connection))
            return false;
    }

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string dynContactList::toString(const int &precision) const{
    stringstream ss;
    for(const_iterator it=begin();it!=end();it++)
        ss<<"- "<<it->toString(precision)<<";\n";
    return ss.str();
}

