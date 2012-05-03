/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <yarp/math/Math.h>
#include "iCub/skinDynLib/skinContact.h"

using namespace iCub::skinDynLib;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

//~~~~~~~~~~~~~~~~~~~~~~
//   CONSTRUCTORS
//~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const dynContact &c)
    :dynContact(c), skinPart(UNKNOWN_SKIN_PART), geoCenter(zeros(3)), pressure(0.0), activeTaxels(0), normalDir(zeros(3)) {}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), activeTaxels(_activeTaxels), taxelList(vector<unsigned int>(activeTaxels, 0)), pressure(_pressure), normalDir(zeros(3)){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure, const Vector &_normalDir)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), activeTaxels(_activeTaxels), taxelList(vector<unsigned int>(activeTaxels, 0)), pressure(_pressure), normalDir(_normalDir){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, vector<unsigned int> _taxelList, double _pressure)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), taxelList(_taxelList), activeTaxels(_taxelList.size()), pressure(_pressure), normalDir(zeros(3)){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, vector<unsigned int> _taxelList, double _pressure, const Vector &_normalDir)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), taxelList(_taxelList), activeTaxels(_taxelList.size()), pressure(_pressure), normalDir(_normalDir){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact()
: dynContact(), skinPart(UNKNOWN_SKIN_PART), geoCenter(zeros(3)), pressure(0.0), activeTaxels(0), normalDir(zeros(3)) {}
//~~~~~~~~~~~~~~~~~~~~~~
//   SET methods
//~~~~~~~~~~~~~~~~~~~~~~    
bool skinContact::setGeoCenter(const Vector &_geoCenter){
    if(!checkVectorDim(_geoCenter, 3, "Geometric center"))
        return false;
    geoCenter = _geoCenter;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::setNormalDir(const Vector &_normalDir){
    if(!checkVectorDim(_normalDir, 3, "Normal direction"))
        return false;
    normalDir = _normalDir;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::setPressure(double _pressure){
    if(pressure<=0.0)
        return false;
    pressure = _pressure;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setActiveTaxels(unsigned int _activeTaxels){
    activeTaxels = _activeTaxels;
    taxelList.resize(activeTaxels);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setSkinPart(SkinPart _skinPart){
    skinPart = _skinPart;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setTaxelList(const vector<unsigned int> &list){
    taxelList = list;
    activeTaxels = list.size();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//   SERIALIZATION methods
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::write(ConnectionWriter& connection){
    // represent a skinContact as a list of 8 elements that are:
    // - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    // - a list of 3 double, i.e. the geometric center
    // - a list of 3 double, i.e. the normal direction
    // - a list of N int, i.e. the active taxel ids
    // - a double, i.e. the pressure

    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(8);
    // list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_INT);
    connection.appendInt(4);
    connection.appendInt(contactId);
    connection.appendInt(bodyPart);    // left_arm, right_arm, ...
    connection.appendInt(linkNumber);
    connection.appendInt(skinPart);
    // list of 3 double, i.e. the CoP
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(CoP[i]);
    // list of 3 double, i.e. the force
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(F[i]);
    // list of 3 double, i.e. the moment
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(Mu[i]);
    // - a list of 3 double, i.e. the geometric center
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(geoCenter[i]);
    // - a list of 3 double, i.e. the normal direction
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(normalDir[i]);
    // - a list of N int, i.e. the active taxel ids
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_INT);
    connection.appendInt(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++) connection.appendInt(taxelList[i]);
    // - a double, i.e. the pressure
    connection.appendInt(BOTTLE_TAG_DOUBLE);
    connection.appendDouble(pressure);

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();
    
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::read(ConnectionReader& connection){
    // auto-convert text mode interaction
    connection.convertTextMode();

    // represent a skinContact as a list of 8 elements that are:
    // - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    // - a list of 3 double, i.e. the geometric center
    // - a list of 3 double, i.e. the normal direction
    // - a list of N int, i.e. the active taxel ids
    // - a double, i.e. the pressure
    if(connection.expectInt() != BOTTLE_TAG_LIST || connection.expectInt() != 8)
        return false;

    // - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT || connection.expectInt()!=4)
        return false;
    contactId   = connection.expectInt();
    bodyPart    = (BodyPart)connection.expectInt();
    linkNumber  = connection.expectInt();
    skinPart    = (SkinPart) connection.expectInt();

    // - a list of 3 double, i.e. the CoP
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) CoP[i] = connection.expectDouble();

    // - a list of 3 double, i.e. the force
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) F[i] = connection.expectDouble();
    setForce(F);

    // - a list of 3 double, i.e. the moment
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) Mu[i] = connection.expectDouble();

    // - a list of 3 double, i.e. the geometric center
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) geoCenter[i]   = connection.expectDouble();

    // - a list of 3 double, i.e. the normal direction
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) normalDir[i]   = connection.expectDouble();

    // - a list of N int, i.e. the active taxel ids
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT)
        return false;
    activeTaxels                        = connection.expectInt();
    taxelList.resize(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++) taxelList[i] = connection.expectInt();

    // - a double, i.e. the pressure
    if(connection.expectInt()!=BOTTLE_TAG_DOUBLE)
        return false;
    pressure                            = connection.expectDouble();

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string skinContact::toString(int precision) const{
    stringstream res;
    res<< dynContact::toString(precision)<< ", Skin part: "<< SkinPart_s[skinPart]<< ", geometric center: "<< 
        geoCenter.toString(precision)<< ", normal direction: "<< normalDir.toString(precision)<< 
        ", active taxels: "<< activeTaxels<< ", pressure: "<< pressure;
    return res.str();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


